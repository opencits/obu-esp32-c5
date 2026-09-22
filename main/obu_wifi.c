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

static uint16_t channel;     // in MHZ : CCH
static uint32_t filter;
static bool wifi_paused;
static uint16_t tx_datarate = 6000; // in kbps: 6000
static int8_t tx_power = -1;
static bool tx_loopback = false;

static wifi_stats_t wifi_stats;

static TaskHandle_t task;
static QueueHandle_t work_queue;
static SemaphoreHandle_t sem_task_over;

// the PHY hooks to enable 802.11p functionality
void phy_change_channel(int,int,int,int);
void phy_11p_set(int,int);

/* phy_get_cca_cnt/phy_set_cca_cnt: undocumented libphy.a symbols, no caller
 * anywhere in esp-idf to confirm against. Reverse-engineered from esp32c5
 * phy_feature.o disassembly, then confirmed against real hardware via
 * `ccatest` (see obu_wifi module CLAUDE notes / session log for the trace):
 *   - phy_set_cca_cnt(value, onOff): low 27 bits of `value` set a cycle-count
 *     ceiling in a control reg (0x600A7C58, top 5 bits preserved elsewhere).
 *     onOff!=0 pulses bits 27-28 (self-clearing — reads back as 0 immediately
 *     after) which resets and (re)starts both counters below from 0.
 *   - phy_get_cca_cnt(out): reads two adjacent 28-bit free-running counters —
 *     0x600A7C5C ("total", confirmed to tick at ~39.8 MHz, i.e. the PHY's
 *     40 MHz ref clock: elapsed cycles since last start) and 0x600A7C60
 *     ("busy", confirmed to grow much slower and independently — consistent
 *     with CCA-busy cycles, though not yet causally tied to known TX/RX
 *     activity). Both mask to the low 27 bits for out->total/out->busy; bit 27
 *     of the total reg is split out as the return value, which is set once
 *     `total` reaches the configured ceiling — at that point both counters
 *     freeze (no wraparound) until the next start pulse. Max single-shot
 *     window at ~40 MHz is 134217727/40e6 ~= 3.35s.
 *   - busy/total ratio == CCA channel-busy ratio, ready for ITS-G5 DCC use
 *     once the causal TX link is confirmed. See `ccatest`.
 */
typedef struct {
    uint32_t total; // elapsed PHY clock cycles since last (re)start, ~40MHz, caps at 0x07FFFFFF
    uint32_t busy;  // CCA-busy cycles accumulated over the same window
} phy_cca_cnt_t;
void phy_set_cca_cnt(uint32_t value, int onOff);
bool phy_get_cca_cnt(phy_cca_cnt_t *out);

// raw MMIO addresses backing the above, for cross-checking the guessed struct
// layout independent of phy_get_cca_cnt's own masking/shifting.
#define PHY_CCA_CTRL_REG   (*(volatile uint32_t *)0x600A7C58)
#define PHY_CCA_CNT_A_REG  (*(volatile uint32_t *)0x600A7C5C)
#define PHY_CCA_CNT_B_REG  (*(volatile uint32_t *)0x600A7C60)

static int datarateKbpsToCode(uint16_t rate)
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

static uint16_t datarateCodeToKbps(int rate)
{
    switch (rate)
    {
    case WIFI_PHY_RATE_6M:
        return 3000;
    case WIFI_PHY_RATE_9M:
        return 4500;
    case WIFI_PHY_RATE_12M:
        return 6000;
    case WIFI_PHY_RATE_18M:
        return 9000;
    case WIFI_PHY_RATE_24M:
        return 12000;
    case WIFI_PHY_RATE_36M:
        return 18000;
    case WIFI_PHY_RATE_48M:
        return 24000;
    case WIFI_PHY_RATE_54M:
        return 27000;
    default:
        break;
    }
    return 6000; // return default rate (6Mbps)
}

static bool check_valid_channel(uint16_t chan)
{
    switch (chan)
    {
    case 5860: // SCH4
    case 5870: // SCH3
    case 5880: // SCH1
    case 5890: // SCH2
    case 5900: // CCH, SCH0
    case 5910: // SCH5
    case 5920: // SCH6
        return true;
    default:
        break;
    }
    return false;
}

static void queue_packet(const void *recv_packet, uint16_t len, const CommFrameHeader *packet_info)
{
    wifi_packet_t packet;
    /* Copy a packet from Link Layer driver and queue the copy to be processed by sniffer task */
    packet.payload = malloc(sizeof(CommFrameHeader) + len);
    if (packet.payload)
    {
        packet.length = sizeof(CommFrameHeader) + len;
        memcpy(packet.payload, packet_info, sizeof(CommFrameHeader));
        memcpy(packet.payload + sizeof(CommFrameHeader), recv_packet, len);
        if (work_queue)
        {
            /* send packet_info */
            if (xQueueSend(work_queue, &packet, pdMS_TO_TICKS(WIFI_PROCESS_PACKET_TIMEOUT_MS)) == pdTRUE)
            {
                return;
            }
            ESP_LOGE(TAG, "sniffer work queue full");
        }
        else
        {
            ESP_LOGE(TAG, "sniffer work queue not initialized");
        }
        free(packet.payload);
    }
    else
    {
        ESP_LOGE(TAG, "No enough memory for promiscuous packet");
    }
    wifi_stats.rxErr++;
}

static void wifi_receive_cb(void *recv_buf, wifi_promiscuous_pkt_type_t type)
{
    CommFrameHeader packet_info;
    uint16_t payloadLen;
    wifi_promiscuous_pkt_t *packet = (wifi_promiscuous_pkt_t *)recv_buf;

    /* prepare packet_info */
    packet_info.seconds = htonl(packet->rx_ctrl.timestamp / 1000000U);
    packet_info.microseconds = htonl(packet->rx_ctrl.timestamp % 1000000U);
    packet_info.rssi_power = (int8_t)packet->rx_ctrl.rssi;
    packet_info.noise = (int8_t)packet->rx_ctrl.noise_floor;

    // donot use the rx_ctrl channel
    // this is the MAC channel used in wifi_resume,
    // --> esp_wifi_set_channel(WIFI_HW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    // then overwritten by
    // --> phy_change_channel(channel, 1, 0, 0);
    packet_info.channel = htons(channel);

    // convert to datarate in kbps
    packet_info.rate = htons(datarateCodeToKbps(packet->rx_ctrl.rate));

    payloadLen = packet->rx_ctrl.sig_len - WIFI_PAYLOAD_FCS_LEN;
    packet_info.frameLen = htons(payloadLen);
    /* For now, the sniffer only dumps the length of the MISC type frame */
    if (type != WIFI_PKT_MISC && !packet->rx_ctrl.rx_state)
    {
        queue_packet(packet->payload, payloadLen, &packet_info);
    }
}

static void wifi_loopback(const uint8_t* payload,
                        uint16_t payloadLen,
                        uint16_t chan,
                        uint16_t rate) {
    struct timespec tp;
    clock_gettime(CLOCK_REALTIME, &tp);

    CommFrameHeader packet_info;
    packet_info.seconds = htonl(tp.tv_sec);
    packet_info.microseconds = htonl(tp.tv_nsec / 1000);
    packet_info.channel = htons(chan);
    packet_info.rate = htons(rate);
    packet_info.rssi_power = 0;
    packet_info.noise = 0;
    packet_info.frameLen = htons(payloadLen);

    queue_packet(payload, payloadLen, &packet_info);
}

static void wifi_task(void *)
{
    wifi_packet_t packet;
    while (is_running)
    {
        /* receive packet info from queue */
        if (xQueueReceive(work_queue, &packet, pdMS_TO_TICKS(WIFI_PROCESS_PACKET_TIMEOUT_MS)) != pdTRUE)
        {
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
    while (left_items--)
    {
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
    uint32_t conf_channel = 5900;
    esp_err_t ret = config_get_u32(CONFIG_INDEX_AUTOSTART_CHAN, &conf_channel);
    if (ret != ESP_OK)
    {
        if (ret != ESP_ERR_NVS_NOT_FOUND)
        ESP_LOGE(TAG, "autostart: config_get_u32 failed: %s", ret);
        return;
    }
    if (!check_valid_channel(conf_channel))
    {
        ESP_LOGE(TAG, "autostart: conf_channel invalid: %u", conf_channel);
        return;
    }

    channel = conf_channel;
    filter = WIFI_PROMIS_FILTER_MASK_ALL & ~WIFI_PROMIS_FILTER_MASK_FCSFAIL;
    wifi_start();
}

void wifi_pause(void)
{
  if (!coex_enabled) return;
  wifi_pause_force();
}

void wifi_pause_force(void)
{
  if (wifi_paused) return;

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

  sem_task_over = xSemaphoreCreateBinary();
  ESP_GOTO_ON_FALSE(sem_task_over, ESP_FAIL, err, TAG, "create semaphore failed");

  work_queue = xQueueCreate(CONFIG_SNIFFER_WORK_QUEUE_LEN, sizeof(wifi_packet_t));
  ESP_GOTO_ON_FALSE(work_queue, ESP_FAIL, err, TAG, "create work queue failed");

  err:
      ESP_ERROR_CHECK(ret);
}

void wifi_get_stats(wifi_stats_t *stats)
{
    memcpy(stats, &wifi_stats, sizeof(wifi_stats_t));
}

void wifi_set_channel(uint16_t chan)
{
    if (!check_valid_channel(chan))
    {
        ESP_LOGE(TAG, "Invalid channel value: %u", chan);
        return;
    }
    channel = chan;
}
void wifi_set_filter(uint32_t filt)
{
    filter = filt;
}
void wifi_set_txpower(int powerDbm)
{
    int8_t power = powerDbm * 4;
    if (power < 8 || power > 80)
    {
        ESP_LOGE(TAG, "Invalid tx power value: %d (2..20)", powerDbm);
        return;
    }
    while (wifi_paused)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    esp_err_t ret = esp_wifi_set_max_tx_power(power);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_wifi_set_max_tx_power failed: %s", esp_err_to_name(ret));
        tx_power = -1;
    }
    else
    {
        tx_power = power / 4;
    }
}

uint32_t wifi_get_channel(void)
{
    return channel;
}
uint32_t wifi_get_filter(void)
{
    return filter;
}

int8_t wifi_get_txpower(void)
{
    if (tx_power < 0)
    {
        while (wifi_paused)
        {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        esp_err_t ret = esp_wifi_get_max_tx_power(&tx_power);
        if (ret != ESP_OK)
        {
            tx_power = -1;
            ESP_LOGE(TAG, "Failed read TX power: %s", esp_err_to_name(ret));
        }
        else
        {
            tx_power = tx_power / 4;
        }
    }
    return tx_power;
}

void wifi_set_datarate(uint16_t datarate)
{
    tx_datarate = datarate;
}
void wifi_set_loop(uint8_t onOff)
{
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

void wifi_tx_frame(uint8_t *payload, uint16_t length)
{
    if (length < sizeof(CommFrameHeader))
    {
        ESP_LOGW(TAG, "frame too short for header: %d < %d", length, (int)sizeof(CommFrameHeader));
        wifi_stats.txErr++;
        return;
    }

    const CommFrameHeader* hdr = (CommFrameHeader *)payload;
    payload += sizeof(CommFrameHeader);
    length -= sizeof(CommFrameHeader);

    if (ntohs(hdr->frameLen) != length)
    {
        ESP_LOGW(TAG, "frame length mismatch: %d != %d", ntohs(hdr->frameLen), length);
        wifi_stats.txErr++;
        return;
    }

    // allow channel in header as zero -> use default channel
    // yet when provided must match currently tuned channel
    uint16_t chan = ntohs(hdr->channel);
    if (chan != 0 && chan != channel)
    {
        ESP_LOGW(TAG, "invalid channel: %u", chan);
        wifi_stats.txErr++;
        return;
    }
    chan = channel;

    // allow rate in header as zero -> use default rate
    uint16_t rate = ntohs(hdr->rate);
    if (rate == 0)
    {
        rate = tx_datarate;
    }
    tx_rate_config.rate = datarateKbpsToCode(rate);

    esp_err_t rc = esp_wifi_80211_tx_custom(WIFI_IF_STA,
        payload,
        length,
        false,
        &tx_rate_config,
        WIFI_BAND_5G,
        WIFI_BW20);

    if (rc != ESP_OK)
    {
        wifi_stats.txErr++;
        ESP_LOGW(TAG, "frame tx failed: %s", esp_err_to_name(rc));
    }
    else
    {
        wifi_stats.tx++;
        esp_event_post(WIFI_EVENT_BASE, WIFI_TRANSMIT_PACKET, NULL, 0, 0);
        ESP_LOGD(TAG, "TX: %d bytes", length);
        if (tx_loopback)
        {
            wifi_loopback(payload, length, chan, rate);
        }
    }
}

// ============================ CCA/DCC TEST (experimental) ==================
// Diagnostic for the phy_get_cca_cnt/phy_set_cca_cnt hooks above. Counter
// semantics (ceiling, self-clearing start pulse, ~40MHz tick rate) confirmed
// against real hardware; the busy counter's causal link to actual channel
// traffic is not yet confirmed. Not for production DCC use until it is.

#define CCA_TEST_CANARY 0xA5A5A5A5u

static struct {
    struct arg_int *window;
    struct arg_int *samples;
    struct arg_int *interval;
    struct arg_lit *no_start;
    struct arg_end *end;
} cca_test_args;

static int do_cca_test_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&cca_test_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, cca_test_args.end, argv[0]);
        return 0;
    }

    uint32_t window = 40000000u; // ~1s at the confirmed ~39.8MHz tick rate (ETSI TS 102 687 default CBR interval)
    if (cca_test_args.window->count)
    {
        window = (uint32_t)cca_test_args.window->ival[0];
    }
    int samples = cca_test_args.samples->count ? cca_test_args.samples->ival[0] : 10;
    int interval_ms = cca_test_args.interval->count ? cca_test_args.interval->ival[0] : 1000;
    bool do_start = !cca_test_args.no_start->count;

    fprintf(stdout, "ccatest: undocumented PHY registers, may destabilize the radio; power-cycle if it hangs\n");
    fprintf(stdout, "ccatest: ctrl=0x%08x cntA=0x%08x cntB=0x%08x (before)\n",
        (unsigned)PHY_CCA_CTRL_REG, (unsigned)PHY_CCA_CNT_A_REG, (unsigned)PHY_CCA_CNT_B_REG);

    if (do_start)
    {
        phy_set_cca_cnt(window, 1);
        fprintf(stdout, "ccatest: phy_set_cca_cnt(0x%08x, 1) called\n", (unsigned)window);
    }
    else
    {
        fprintf(stdout, "ccatest: --no-start, sampling existing counter state only\n");
    }

    phy_cca_cnt_t prev = {0};
    bool have_prev = false;

    for (int i = 0; i < samples; i++)
    {
        vTaskDelay(pdMS_TO_TICKS(interval_ms));

        // guard bytes past the 8 used bytes catch phy_get_cca_cnt writing more than expected
        uint32_t buf[8];
        for (int j = 0; j < 8; j++) buf[j] = CCA_TEST_CANARY;
        phy_cca_cnt_t *out = (phy_cca_cnt_t *)buf;
        bool flag = phy_get_cca_cnt(out);

        int overrun = -1;
        for (int j = 2; j < 8; j++)
        {
            if (buf[j] != CCA_TEST_CANARY) { overrun = j; break; }
        }

        uint32_t raw_ctrl = PHY_CCA_CTRL_REG;
        uint32_t raw_a = PHY_CCA_CNT_A_REG;
        uint32_t raw_b = PHY_CCA_CNT_B_REG;

        fprintf(stdout, "ccatest[%d]: total=%u busy=%u flag=%d | raw ctrl=0x%08x cntA=0x%08x cntB=0x%08x",
            i, (unsigned)out->total, (unsigned)out->busy, flag,
            (unsigned)raw_ctrl, (unsigned)raw_a, (unsigned)raw_b);

        if (overrun >= 0)
        {
            fprintf(stdout, " | WROTE PAST 8 BYTES at word %d = 0x%08x", overrun, (unsigned)buf[overrun]);
        }

        if (have_prev)
        {
            uint32_t d_total = out->total - prev.total; // wraps naturally if counter reset/overflowed
            uint32_t d_busy = out->busy - prev.busy;
            if (d_total > 0)
            {
                fprintf(stdout, " | delta busy/total=%u/%u (%.1f%%)",
                    (unsigned)d_busy, (unsigned)d_total, 100.0 * d_busy / d_total);
            }
            else
            {
                fprintf(stdout, " | delta total=0 (counter idle or reset)");
            }
        }
        fprintf(stdout, "\n");

        prev = *out;
        have_prev = true;
    }

    return 0;
}

void register_cca_test_cmd(void)
{
    cca_test_args.window = arg_int0("w", "window", "<cycles>", "count ceiling passed to phy_set_cca_cnt, PHY clock cycles @ ~39.8MHz, max 0x07FFFFFF (~3.35s) (default ~1s)");
    cca_test_args.samples = arg_int0("n", "samples", "<n>", "number of samples to take (default 10)");
    cca_test_args.interval = arg_int0("i", "interval", "<ms>", "delay between samples in ms (default 1000)");
    cca_test_args.no_start = arg_lit0(NULL, "no-start", "don't call phy_set_cca_cnt, just sample current counter state");
    cca_test_args.end = arg_end(2);

    const esp_console_cmd_t cca_test_cmd = {
        .command = "ccatest",
        .help = "EXPERIMENTAL: probe undocumented PHY CCA busy/total counters (see obu_wifi.c comment above do_cca_test_cmd)",
        .hint = NULL,
        .func = &do_cca_test_cmd,
        .argtable = &cca_test_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cca_test_cmd));
}

// ============================ CCA/DCC MEASUREMENT (background) ============
// Continuous ETSI ITS-G5 DCC channel-busy-ratio measurement, built on the
// hardware validation in docs/esp_dcc_measurements.md (confirmed against a
// reference load generator across idle/~5%/~10%/~45% channel load).

#define CCA_WINDOW_CYCLES     40000000u // ~1s at the confirmed ~39.8MHz tick rate
#define CCA_WINDOW_WAIT_MS    1100      // window + margin: ~39.8MHz needs ~1005ms to reach the ceiling
#define CCA_HISTORY_LEN       5         // rolling window for the filtered average
#define CCA_OUTLIER_LOW_FRAC  0.5f      // drop samples below this fraction of the median: low dropouts were observed whenever a window landed in a gap of the reference generator's duty cycle, never a high spike (high spikes are the separate busy>total race, clamped below, not filtered here)
#define CCA_TASK_STACK        3072
#define CCA_TASK_PRIORITY     3

static SemaphoreHandle_t cca_mutex;
static TaskHandle_t cca_task;
static float cca_history[CCA_HISTORY_LEN];
static uint8_t cca_history_count;
static uint8_t cca_history_next;
static wifi_cca_stats_t cca_stats;

static int cca_float_cmp(const void *a, const void *b)
{
    float fa = *(const float *)a, fb = *(const float *)b;
    return (fa > fb) - (fa < fb);
}

/* Mean of the history, excluding samples below CCA_OUTLIER_LOW_FRAC * median.
 * Falls back to the plain mean if that excludes everything (shouldn't happen:
 * the median itself always qualifies). Caller holds cca_mutex. */
static float cca_filtered_average(void)
{
    float sorted[CCA_HISTORY_LEN];
    memcpy(sorted, cca_history, cca_history_count * sizeof(float));
    qsort(sorted, cca_history_count, sizeof(float), cca_float_cmp);
    float median = sorted[cca_history_count / 2];
    float threshold = median * CCA_OUTLIER_LOW_FRAC;

    float sum = 0;
    int n = 0;
    for (int i = 0; i < cca_history_count; i++)
    {
        if (cca_history[i] >= threshold)
        {
            sum += cca_history[i];
            n++;
        }
    }
    if (n == 0)
    {
        for (int i = 0; i < cca_history_count; i++) sum += cca_history[i];
        n = cca_history_count;
    }
    return sum / n;
}

static void cca_measure_task(void *arg)
{
    uint32_t last_total = 0, last_busy = 0;
    bool have_last = false;

    for (;;)
    {
        phy_set_cca_cnt(CCA_WINDOW_CYCLES, 1);
        vTaskDelay(pdMS_TO_TICKS(CCA_WINDOW_WAIT_MS));

        phy_cca_cnt_t raw;
        bool flag = phy_get_cca_cnt(&raw);

        /* A genuinely completed window always reports exactly the configured
         * ceiling. Anything else — never started, or a stale re-read of an
         * earlier window after a start-pulse miss-fire (see
         * docs/esp_dcc_measurements.md) — is discarded, not averaged. */
        bool duplicate = have_last && raw.total == last_total && raw.busy == last_busy;
        bool valid = flag && raw.total == CCA_WINDOW_CYCLES && !duplicate;

        last_total = raw.total;
        last_busy = raw.busy;
        have_last = true;

        if (!valid)
        {
            xSemaphoreTake(cca_mutex, portMAX_DELAY);
            cca_stats.rejected_samples++;
            xSemaphoreGive(cca_mutex);
            continue;
        }

        uint32_t busy = raw.busy > raw.total ? raw.total : raw.busy; // busy>total race, see doc
        float percent = 100.0f * (float)busy / (float)raw.total;

        xSemaphoreTake(cca_mutex, portMAX_DELAY);
        cca_history[cca_history_next] = percent;
        cca_history_next = (cca_history_next + 1) % CCA_HISTORY_LEN;
        if (cca_history_count < CCA_HISTORY_LEN) cca_history_count++;

        cca_stats.latest_percent = percent;
        cca_stats.avg_percent = cca_filtered_average();
        cca_stats.valid_samples++;
        cca_stats.has_data = true;
        xSemaphoreGive(cca_mutex);
    }
}

void wifi_cca_start(void)
{
    if (cca_task) return;

    cca_mutex = xSemaphoreCreateMutex();
    if (!cca_mutex)
    {
        ESP_LOGE(TAG, "cca: create mutex failed");
        return;
    }
    if (xTaskCreate(cca_measure_task, "tCca", CCA_TASK_STACK, NULL, CCA_TASK_PRIORITY, &cca_task) != pdTRUE)
    {
        ESP_LOGE(TAG, "cca: create task failed");
        cca_task = NULL;
    }
}

void wifi_get_cca_stats(wifi_cca_stats_t *stats)
{
    if (!cca_mutex)
    {
        memset(stats, 0, sizeof(*stats));
        return;
    }
    xSemaphoreTake(cca_mutex, portMAX_DELAY);
    *stats = cca_stats;
    xSemaphoreGive(cca_mutex);
}

static int do_cca_stats_cmd(int argc, char **argv)
{
    wifi_cca_stats_t stats;
    wifi_get_cca_stats(&stats);
    if (!stats.has_data)
    {
        fprintf(stdout, "no valid CBR window yet (valid=%lu rejected=%lu)\n",
            (unsigned long)stats.valid_samples, (unsigned long)stats.rejected_samples);
        return 0;
    }
    fprintf(stdout, "CBR latest=%.3f%% avg=%.3f%% (valid=%lu rejected=%lu)\n",
        stats.latest_percent, stats.avg_percent,
        (unsigned long)stats.valid_samples, (unsigned long)stats.rejected_samples);
    return 0;
}

void register_cca_stats_cmd(void)
{
    const esp_console_cmd_t cmd = {
        .command = "ccastats",
        .help = "show background ETSI DCC channel-busy-ratio measurement (see wifi_cca_start)",
        .hint = NULL,
        .func = &do_cca_stats_cmd,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

// ============================ DCCMON (background printer) =================
// `dccmon` console cmd starts/stops a background task that periodically logs
// wifi_get_cca_stats()'s filtered average — a live view of the CBR measured
// in docs/esp_dcc_measurements.md, without blocking the console like `ccatest`.

static int dccmon_interval_ms = -1; // <=0: task idles, doesn't print
static TaskHandle_t dccmon_task;

static void dccmon_task_fn(void *arg)
{
    for (;;)
    {
        if (dccmon_interval_ms <= 0)
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        wifi_cca_stats_t stats;
        wifi_get_cca_stats(&stats);
        if (stats.has_data)
        {
            /* fprintf, not ESP_LOGI: CONFIG_LOG_DEFAULT_LEVEL=2 (WARN) in this
             * project's sdkconfig drops INFO-level logs silently. */
            fprintf(stdout, "dccmon: CBR avg=%.2f%% latest=%.2f%% (valid=%lu rejected=%lu)\n",
                stats.avg_percent, stats.latest_percent,
                (unsigned long)stats.valid_samples, (unsigned long)stats.rejected_samples);
        }
        else
        {
            fprintf(stdout, "dccmon: no valid CBR window yet (rejected=%lu)\n",
                (unsigned long)stats.rejected_samples);
        }
        vTaskDelay(pdMS_TO_TICKS(dccmon_interval_ms));
    }
}

static struct {
    struct arg_int *interval;
    struct arg_lit *stop;
    struct arg_end *end;
} dccmon_args;

static int do_dccmon_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&dccmon_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, dccmon_args.end, argv[0]);
        return 0;
    }

    if (dccmon_args.stop->count)
    {
        dccmon_interval_ms = -1;
        fprintf(stdout, "dccmon stopped\n");
        return 0;
    }

    dccmon_interval_ms = dccmon_args.interval->count ? dccmon_args.interval->ival[0] : 2000;

    if (!dccmon_task)
    {
        xTaskCreate(dccmon_task_fn, "tDccMon", 3072, NULL, 2, &dccmon_task);
    }
    fprintf(stdout, "dccmon printing every %d ms\n", dccmon_interval_ms);
    return 0;
}

void register_dccmon_cmd(void)
{
    dccmon_args.interval = arg_int0("i", "interval", "<ms>", "print interval in ms (default 2000)");
    dccmon_args.stop = arg_lit0(NULL, "stop", "stop printing");
    dccmon_args.end = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command = "dccmon",
        .help = "periodically print the background ETSI DCC channel-busy-ratio average",
        .hint = NULL,
        .func = &do_dccmon_cmd,
        .argtable = &dccmon_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

// ============================ CBR_IND (wire protocol push) =================
// Same data as `dccmon`, pushed as COMM_FRAME_CBR_IND over USB/BLE (whichever
// is connected) instead of printed — gated by COMM_FRAME_CMD_CBR_START/STOP
// (obu_comm.c). Mirrors wifi_task's existing dual-publish-if-connected pattern.

static int cbr_ind_interval_ms = -1; // <=0: task idles, doesn't publish
static TaskHandle_t cbr_ind_task;

static void cbr_ind_publish(void)
{
    wifi_cca_stats_t stats;
    wifi_get_cca_stats(&stats);

    CbrIndPayload ind = {
        .avgPercentMilli = htonl((uint32_t)(stats.avg_percent * 1000.0f)),
        .latestPercentMilli = htonl((uint32_t)(stats.latest_percent * 1000.0f)),
        .validSamples = htonl(stats.valid_samples),
        .rejectedSamples = htonl(stats.rejected_samples),
    };

    if (obu_comm_usb_connected()) usb_stream_publish_frame(COMM_FRAME_CBR_IND, (const uint8_t *)&ind, sizeof(ind));
    if (obu_comm_ble_connected()) bt_stream_publish_frame(COMM_FRAME_CBR_IND, (const uint8_t *)&ind, sizeof(ind));
}

static void cbr_ind_task_fn(void *arg)
{
    for (;;)
    {
        if (cbr_ind_interval_ms <= 0)
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        cbr_ind_publish();
        vTaskDelay(pdMS_TO_TICKS(cbr_ind_interval_ms));
    }
}

void wifi_cbr_ind_start(uint32_t interval_ms)
{
    cbr_ind_interval_ms = (int)interval_ms;
    if (!cbr_ind_task)
    {
        xTaskCreate(cbr_ind_task_fn, "tCbrInd", 3072, NULL, 2, &cbr_ind_task);
    }
}

void wifi_cbr_ind_stop(void)
{
    cbr_ind_interval_ms = -1;
}
