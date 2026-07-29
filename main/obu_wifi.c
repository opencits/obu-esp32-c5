#include "sdkconfig.h"

#include <string.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/unistd.h>
#include <sys/fcntl.h>

#include "argtable3/argtable3.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_private/wifi_os_adapter.h"
#include "esp_app_trace.h"
#include "esp_check.h"
#include "esp_console.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs.h"

#include "pcap.h"
#include "config.h"
#include "events.h"

#include "obu_wifi.h"
#include "obu_comm.h"
#include "usb_stream.h"
#include "bt_stream.h"

#define WIFI_DEFAULT_CHANNEL             (1)
#define WIFI_PAYLOAD_FCS_LEN             (4)
#define WIFI_PROCESS_PACKET_TIMEOUT_MS   (100)
#define WIFI_RX_FCS_ERR                  (0X41)
#define WIFI_MAX_ETH_INTFS               (3)
#define WIFI_DECIMAL_NUM                 (10)
/* fixed 802.11 hw channel the radio is tuned to; 802.11p freq is set separately via phy_change_channel() */
#define WIFI_HW_CHANNEL                  (140)

static const char TAG[] = "wifi";

static bool is_running;
static bool coex_enabled;

static uint32_t channel;
static uint32_t filter;
static bool wifi_paused;
static int tx_datarate;
static int8_t tx_power = -1;
static bool tx_loopback = false;

static wifi_stats_t wifi_stats;

static TaskHandle_t task;
static QueueHandle_t work_queue;
static SemaphoreHandle_t sem_task_over;

// the PHY hooks to enable 802.11p functionality
void phy_change_channel(int,int,int,int);
void phy_11p_set(int,int);

static void queue_packet(void *recv_packet, uint16_t len, const CommFrameHeader *packet_info)
{
    wifi_packet_t packet;
    /* Copy a packet from Link Layer driver and queue the copy to be processed by sniffer task */
    packet.payload = malloc(sizeof(CommFrameHeader) + len);
    packet.length = sizeof(CommFrameHeader) + len;
    if (packet.payload) {
        memcpy(packet.payload, packet_info, sizeof(CommFrameHeader));
        memcpy(packet.payload + sizeof(CommFrameHeader), recv_packet, len);
        if (work_queue) {
            /* send packet_info */
            if (xQueueSend(work_queue, &packet, pdMS_TO_TICKS(WIFI_PROCESS_PACKET_TIMEOUT_MS)) != pdTRUE) {
                wifi_stats.rxErr++;
                ESP_LOGE(TAG, "sniffer work queue full");
                free(packet.payload);
            }
        }
    } else {
        ESP_LOGE(TAG, "No enough memory for promiscuous packet");
    }
}

static void wifi_receive_cb(void *recv_buf, wifi_promiscuous_pkt_type_t type)
{
    CommFrameHeader packet_info;
    uint16_t payloadLen;
    wifi_promiscuous_pkt_t *packet = (wifi_promiscuous_pkt_t *)recv_buf;
    /* prepare packet_info */
    packet_info.seconds = htonl(packet->rx_ctrl.timestamp / 1000000U);
    packet_info.microseconds = htonl(packet->rx_ctrl.timestamp % 1000000U);
    packet_info.channel = (uint8_t)packet->rx_ctrl.channel;
    packet_info.rate = (uint8_t)packet->rx_ctrl.rate;
    packet_info.rssi_power = (int8_t)packet->rx_ctrl.rssi;
    packet_info.noise = (int8_t)packet->rx_ctrl.noise_floor;

#if CONFIG_SOC_WIFI_HE_SUPPORT
    payloadLen = packet->rx_ctrl.dump_len;
#else
    payloadLen = packet->rx_ctrl.sig_len - WIFI_PAYLOAD_FCS_LEN;
#endif

    packet_info.frameLen = htons(payloadLen);
    /* For now, the sniffer only dumps the length of the MISC type frame */
    if (type != WIFI_PKT_MISC && !packet->rx_ctrl.rx_state) {
        queue_packet(packet->payload, payloadLen, &packet_info);
    }
}

static void wifi_loopback(uint8_t* payload,
                        uint16_t payloadLen,
                        uint8_t chan,
                        uint8_t rate) {
    struct timespec tp;
    clock_gettime(CLOCK_REALTIME, &tp);

    CommFrameHeader packet_info;
    packet_info.seconds = htonl(tp.tv_sec);
    packet_info.microseconds = htonl(tp.tv_nsec / 1000);
    packet_info.channel = chan;
    packet_info.rate = rate;
    packet_info.rssi_power = 0;
    packet_info.noise = 0;
    packet_info.frameLen = htons(payloadLen);

    queue_packet(payload, payloadLen, &packet_info);
}

static void wifi_task(void *)
{
    wifi_packet_t packet;

    while (is_running) {
        /* receive packet info from queue */
        if (xQueueReceive(work_queue, &packet, pdMS_TO_TICKS(WIFI_PROCESS_PACKET_TIMEOUT_MS)) != pdTRUE) {
            continue;
        }
        wifi_stats.rx++;
        
        if (obu_comm_usb_connected()) usb_stream_publish_packet(&packet);
        if (obu_comm_ble_connected()) bt_stream_publish_packet(&packet);
        
        esp_event_post(WIFI_EVENT_BASE, WIFI_RECEIVED_PACKET, NULL, 0, 0);
        ESP_LOGI(TAG, "RX: %d bytes", packet.length);
        free(packet.payload);
    }
    /* notify that sniffer task is over */
    xSemaphoreGive(sem_task_over);
    vTaskDelete(NULL);
}

int wifi_stop(void)
{
    esp_err_t ret = ESP_OK;

    ESP_GOTO_ON_FALSE(is_running, ESP_ERR_INVALID_STATE, err, TAG, "WiFi is already stopped");
    /* Disable wifi promiscuous mode */
    ESP_GOTO_ON_ERROR(esp_wifi_set_promiscuous(false), err, TAG, "stop WiFi promiscuous failed");

    ESP_LOGI(TAG, "stop WiFi promiscuous ok");
    /* stop WiFi local task */
    is_running = false;
    /* wait for task over */
    xSemaphoreTake(sem_task_over, portMAX_DELAY);

    /* make sure to free all resources in the left items */
    UBaseType_t left_items = uxQueueMessagesWaiting(work_queue);

    wifi_packet_t packet_info;
    while (left_items--) {
        xQueueReceive(work_queue, &packet_info, pdMS_TO_TICKS(WIFI_PROCESS_PACKET_TIMEOUT_MS));
        free(packet_info.payload);
    }
    esp_event_post(WIFI_EVENT_BASE, WIFI_STOPPED, NULL, 0, 0);
err:
    return ret;
}

int wifi_start(void)
{
    esp_err_t ret = ESP_OK;
    wifi_promiscuous_filter_t wifi_filter;

    ESP_GOTO_ON_FALSE(!is_running, ESP_ERR_INVALID_STATE, err, TAG, "sniffer is already running");

    is_running = true;
    ESP_GOTO_ON_FALSE(xTaskCreate(wifi_task,
        "tWifi",
        CONFIG_SNIFFER_TASK_STACK_SIZE,
        NULL,
        CONFIG_SNIFFER_TASK_PRIORITY,
        &task), ESP_FAIL, err_task, TAG, "create task failed");

    /* Start WiFi Promiscuous Mode */
    wifi_filter.filter_mask = filter;
    esp_wifi_set_promiscuous_filter(&wifi_filter);
    esp_wifi_set_promiscuous_rx_cb(wifi_receive_cb);
    ESP_GOTO_ON_ERROR(esp_wifi_set_promiscuous(true), err_start, TAG, "set promis failed");
    //ESP_GOTO_ON_ERROR(esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE), err_start, TAG, "set channel failed");
    // enable 802.11p mode (enable, unknown (must be 0))
    phy_11p_set(1, 0);
    // set a channel with a frequency close to our desired frequency (not sure if strictly needed)
    ESP_GOTO_ON_ERROR(esp_wifi_set_channel(WIFI_HW_CHANNEL, WIFI_SECOND_CHAN_NONE), err_start, TAG, "set channel failed");
    // switch channel (channel, ignored, ignored, ht_mode?)
    phy_change_channel(channel, 1, 0, 0);

    uint8_t loop;
    if (config_get_u8(CONFIG_INDEX_WIFI_LOOP, &loop) == ESP_OK) {
        tx_loopback = (loop != 0);
    }

    ESP_LOGI(TAG, "start WiFi promiscuous ok");
    esp_event_post(WIFI_EVENT_BASE, WIFI_STARTED, NULL, 0, 0);
    return ret;

err_start:
    // task was already started, need to shut it down gracefully
    is_running = false;
    xSemaphoreTake(sem_task_over, portMAX_DELAY);
    task = NULL;
    err_task:
        is_running = false;
    err:
        return ret;
}

void wifi_autostart(void)
{
  uint32_t conf_channel;
  esp_err_t ret = config_get_u32(CONFIG_INDEX_AUTOSTART_CHAN, &conf_channel);
  if (ret != ESP_OK)
  {
    if (ret != ESP_ERR_NVS_NOT_FOUND)
      ESP_LOGE(TAG, "config_get_u32 failed: %s", ret);
    return;
  }

  if (conf_channel >= 5800 && conf_channel <= 5900)
  {
    channel = conf_channel;
    filter = WIFI_PROMIS_FILTER_MASK_ALL & ~WIFI_PROMIS_FILTER_MASK_FCSFAIL;
    wifi_start();
  }
}

void wifi_pause(void)
{
  if (wifi_paused) return;
  if (!coex_enabled) return;

  ESP_LOGI(TAG, "pause WiFi");
  esp_wifi_set_promiscuous(false);
  phy_11p_set(0, 0);
  esp_wifi_stop();
  wifi_paused = true;
}

void wifi_resume(void)
{
  if (!wifi_paused) return;
  if (!coex_enabled) return;

  ESP_LOGI(TAG, "resume WiFi");
  esp_wifi_start();
  wifi_promiscuous_filter_t wifi_filter = { .filter_mask = filter };
  esp_wifi_set_promiscuous_filter(&wifi_filter);
  esp_wifi_set_promiscuous(true);
  phy_11p_set(1, 0);
  esp_wifi_set_channel(WIFI_HW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  phy_change_channel(channel, 1, 0, 0);
  wifi_paused = false;
}

void wifi_coex_enable(void)
{
    coex_enabled = true;
}

void wifi_coex_disable(void)
{
    coex_enabled = false;
}

void wifi_init(void)
{
  esp_err_t ret = ESP_OK;
  coex_enabled = true;
  tx_datarate = -1;

  sem_task_over = xSemaphoreCreateBinary();
  ESP_GOTO_ON_FALSE(sem_task_over, ESP_FAIL, err, TAG, "create semaphore failed");

  work_queue = xQueueCreate(CONFIG_SNIFFER_WORK_QUEUE_LEN, sizeof(wifi_packet_t));
  ESP_GOTO_ON_FALSE(work_queue, ESP_FAIL, err, TAG, "create work queue failed");

  err:
      ESP_ERROR_CHECK(ret);
}

void wifi_get_stats(wifi_stats_t *stats) {
    memcpy(stats, &wifi_stats, sizeof(wifi_stats_t));
}

void wifi_set_channel(uint32_t chan) {
    channel = chan;
}
void wifi_set_filter(uint32_t filt) {
    filter = filt;
}
void wifi_set_txpower(int powerDbm) {
    int8_t power = powerDbm * 4;
    if (power < 8 || power > 80) {
        ESP_LOGE(TAG, "Invalid tx power value: %d (2..20)", powerDbm);
        return;
    }
    while (wifi_paused) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    esp_err_t ret = esp_wifi_set_max_tx_power(power);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_max_tx_power failed: %s", esp_err_to_name(ret));
        tx_power = -1;
    }
    else {
        tx_power = power / 4;
    }
}

uint32_t wifi_get_channel(void) {
    return channel;
}
uint32_t wifi_get_filter(void) {
    return filter;
}
int8_t wifi_get_txpower(void) {
    if (tx_power < 0) {
        while (wifi_paused) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        esp_err_t ret = esp_wifi_get_max_tx_power(&tx_power);
        if (ret != ESP_OK) {
            tx_power = -1;
            ESP_LOGE(TAG, "Failed read TX power: %s", esp_err_to_name(ret));
        }
        else {
            tx_power = tx_power / 4;
        }
    }
    return tx_power;
}
void wifi_set_datarate(int datarate) {
    tx_datarate = datarate;
}
void wifi_set_loop(uint8_t onOff) {
    tx_loopback = (onOff != 0);
}


// ================================ TRANSMITTER =============================

esp_err_t ieee80211_raw_frame_sanity_check(wifi_interface_t ifx, const void *buffer, int32_t len, bool en_sys_seq);
esp_err_t ieee80211_post_hmac_tx(void *ebuf);
void *ic_ebuf_alloc(const void *packet, uint32_t unknown, uint32_t len);
void *ic_get_default_sched(void);

extern wifi_osi_funcs_t *g_osi_funcs_p;
extern void *g_wifi_global_lock;

typedef struct x_eb_txdesc
{
    uint32_t flags;
    uint32_t field_4;
    uint32_t field_8;
    uint8_t rate;
    uint8_t field_d;
    uint8_t field_e;
    uint8_t field_f;
    uint32_t field_10;
    uint32_t field_14;
    uint32_t timestamp;
    void* sched;
    uint32_t field_20;
    uint32_t field_24;
    uint32_t field_28;
    union {
        uint32_t field_2c_32;
        struct {
            uint8_t field_2c;
            uint8_t field_2d;
            uint8_t field_2e;
            uint8_t field_2f;
        };
    };
    union {
        uint32_t field_30_32;
        struct {
            uint8_t field_30;
            uint8_t field_31;
            uint8_t field_32;
            uint8_t field_33;
        };
    };
    uint32_t field_34;
    uint32_t field_38;
    uint32_t field_3c;
    uint32_t field_40;
    uint32_t field_44;
} x_eb_txdesc_t;
static_assert(sizeof(x_eb_txdesc_t) == 0x48);

typedef struct x_middle_data
{
    uint32_t field_40;
    uint8_t* buf;
    uint32_t field_48;
    uint32_t field_4c;
} x_middle_data_t;
static_assert(sizeof(x_middle_data_t) == 0x10);

typedef struct x_ebuf
{
    uint32_t field_0;
    x_middle_data_t* ds_head;
    x_middle_data_t* ds_tail;
    uint16_t field_c;
    uint16_t field_e;
    uint32_t extra_data_start;
    uint16_t header_length;
    uint32_t data_length;
    uint16_t field_1c;
    uint8_t alloc_type;
    uint8_t field_1f;
    uint32_t field_20;
    uint8_t field_24;
    uint8_t field_25;
    uint8_t field_26;
    uint8_t field_27;
    uint32_t field_28;
    uint8_t field_2c;
    uint32_t field_30;
    uint32_t next_free;
    x_eb_txdesc_t* txdesc;
    uint16_t field_3c;
    uint8_t field_3e;
    uint8_t field_3f;
} x_ebuf_t;
static_assert(sizeof(x_ebuf_t) == 0x40);

esp_err_t esp_wifi_80211_tx_custom(wifi_interface_t ifx,
    const void *buffer,
    int32_t len,
    bool en_sys_seq,
    const wifi_tx_rate_config_t *tx_rate_config,
    wifi_band_t band,
    wifi_bandwidth_t bw)
{
    int result = ESP_OK;
    g_osi_funcs_p->_mutex_lock(g_wifi_global_lock);
    x_ebuf_t* eb = ic_ebuf_alloc(buffer, 1, len);
    if (eb)
    {
        //eb->data_length = len - 0x1a;
        eb->data_length = 0;
        x_eb_txdesc_t *txdesc_1 = eb->txdesc;
        //eb->header_length = 0x1a;
        eb->header_length = len;
        txdesc_1->flags |= 0x4000;
        txdesc_1->sched = ic_get_default_sched();
        wifi_phy_rate_t rate = tx_rate_config->rate;
        x_eb_txdesc_t *txdesc = eb->txdesc;

        if (rate)
            txdesc->rate = (char)rate;
        else if (band != WIFI_BAND_5G)
            txdesc->rate = 0;
        else
            txdesc->rate = (char)WIFI_PHY_RATE_6M;

        wifi_phy_mode_t phymode = tx_rate_config->phymode;

        if (phymode == WIFI_PHY_MODE_HE20)
        {
            txdesc->flags |= 0x80000000;
            txdesc->field_2f =
                (char)((((uint32_t)tx_rate_config->ersu + 6) & 0xf) << 3)
                | (txdesc->field_2f & 0x87);

            if ((uint32_t)tx_rate_config->dcm)
                txdesc->field_31 |= 0x80;
        }
        else if (phymode == WIFI_PHY_MODE_VHT20)
            txdesc->flags |= 0x1000000;

        // No idea if this is correct, but this is what the original code does...
        uint32_t bw_is_bw40 = bw == WIFI_BW40;
        txdesc->field_8 = (bw_is_bw40 << 0xf) | (txdesc->field_8 & 0xffff7fff);

        if (en_sys_seq)
            txdesc->flags |= 1;

        txdesc->field_10 =
            (txdesc->field_10 & 0xfff3ffff) | ((ifx & WIFI_IF_MAX) << 0x12);
        txdesc->field_14 = 0x100;

        ieee80211_post_hmac_tx(eb);
        g_osi_funcs_p->_mutex_unlock(g_wifi_global_lock);
    }
    else
    {
        result = ESP_ERR_NO_MEM;
        g_osi_funcs_p->_mutex_unlock(g_wifi_global_lock);
    }
    return result;
}

wifi_tx_rate_config_t tx_rate_config =
{
    .phymode = WIFI_PHY_MODE_11A,
    .rate = WIFI_PHY_RATE_6M,
    .ersu = false,
    .dcm = false
};

static int datarateKbpsToCode(uint32_t rate)
{
    switch (rate)
    {
    case 3000:
        return WIFI_PHY_RATE_6M; // BPSK 1/2
    case 4500:
        return WIFI_PHY_RATE_9M; // BPSK 3/4
    case 6000:
        return WIFI_PHY_RATE_12M; // QPSK 1/2
    case 9000:
        return WIFI_PHY_RATE_18M; // QPSK 3/4
    case 12000:
        return WIFI_PHY_RATE_24M; // 16-QAM 1/2
    case 18000:
        return WIFI_PHY_RATE_36M; // 16-QAM 3/4
    case 24000:
        return WIFI_PHY_RATE_48M; // 64-QAM 1/2
    case 27000:
        return WIFI_PHY_RATE_54M; // 64-QAM 3/4

    default:
        break;
    }
    return WIFI_PHY_RATE_12M; // return default rate (6Mbps)
}

void wifi_tx_frame(uint8_t *payload, uint16_t length)
{
    if (length < sizeof(CommFrameHeader)) {
        ESP_LOGW(TAG, "frame too short for header: %d < %d", length, (int)sizeof(CommFrameHeader));
        wifi_stats.txErr++;
        return;
    }

    const CommFrameHeader* hdr = (CommFrameHeader *)payload;
    payload += sizeof(CommFrameHeader);
    length -= sizeof(CommFrameHeader);

    if (ntohs(hdr->frameLen) != length) {
        ESP_LOGW(TAG, "frame length mismatch: %d != %d", ntohs(hdr->frameLen), length);
        wifi_stats.txErr++;
        return;
    }

    if (tx_datarate >= 0) {
        tx_rate_config.rate = datarateKbpsToCode(tx_datarate);
    }
    else {
        tx_rate_config.rate = datarateKbpsToCode(6000);
    }

    esp_err_t rc = esp_wifi_80211_tx_custom(WIFI_IF_STA,
        payload,
        length,
        false,
        &tx_rate_config,
        WIFI_BAND_5G,
        WIFI_BW20);

    if (rc != ESP_OK) {
        wifi_stats.txErr++;
        ESP_LOGW(TAG, "frame tx failed: %s", esp_err_to_name(rc));
    }
    else {
        wifi_stats.tx++;
        esp_event_post(WIFI_EVENT_BASE, WIFI_TRANSMIT_PACKET, NULL, 0, 0);
        ESP_LOGD(TAG, "TX: %d bytes", length);
        if (tx_loopback) {
            wifi_loopback(payload, length, hdr->channel, hdr->rate);
        }
    }
}
