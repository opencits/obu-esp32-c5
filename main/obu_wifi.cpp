#include "sdkconfig.h"

#include <cstdlib>
#include <cstring>
#include <ctime>
#include <sys/time.h>
#include <sys/unistd.h>
#include <sys/fcntl.h>

#include "argtable3/argtable3.h"
#include "esp_private/wifi_os_adapter.h"
#include "esp_app_trace.h"
#include "esp_check.h"
#include "esp_console.h"
#include "esp_log.h"
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

// Closed-source blob symbols (libphy.a / libnet80211.a): C linkage.
extern "C" {
// the PHY hooks to enable 802.11p functionality
void phy_change_channel(int, int, int, int);
void phy_11p_set(int, int);

void phy_set_cca_cnt(uint32_t value, int onOff);
bool phy_get_cca_cnt(phy_cca_cnt_t *out);

esp_err_t ieee80211_post_hmac_tx(void *ebuf);
void *ic_ebuf_alloc(const void *packet, uint32_t unknown, uint32_t len);
void *ic_get_default_sched(void);

extern wifi_osi_funcs_t *g_osi_funcs_p;
extern void *g_wifi_global_lock;
}

// raw MMIO addresses backing the above, for cross-checking the guessed struct
// layout independent of phy_get_cca_cnt's own masking/shifting.
#define PHY_CCA_CTRL_REG   (*(volatile uint32_t *)0x600A7C58)
#define PHY_CCA_CNT_A_REG  (*(volatile uint32_t *)0x600A7C5C)
#define PHY_CCA_CNT_B_REG  (*(volatile uint32_t *)0x600A7C60)

constinit WifiController WifiController::s_instance;

namespace {

wifi_phy_rate_t datarateKbpsToCode(uint16_t rate)
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

uint16_t datarateCodeToKbps(int rate)
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

bool checkValidChannel(uint16_t chan)
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

} // namespace

// ================================ SNIFFER =================================

void WifiController::queuePacket(const void *data, uint16_t len, const CommFrameHeader *info)
{
    wifi_packet_t packet;
    /* Copy a packet from Link Layer driver and queue the copy to be processed by sniffer task */
    packet.payload = static_cast<uint8_t *>(malloc(sizeof(CommFrameHeader) + len));
    if (packet.payload)
    {
        packet.length = sizeof(CommFrameHeader) + len;
        memcpy(packet.payload, info, sizeof(CommFrameHeader));
        memcpy(packet.payload + sizeof(CommFrameHeader), data, len);
        if (workQueue_)
        {
            /* send packet_info */
            if (xQueueSend(workQueue_, &packet, pdMS_TO_TICKS(WIFI_PROCESS_PACKET_TIMEOUT_MS)) == pdTRUE)
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
    stats_.rxErr++;
}

void WifiController::rxCallback(void *recvBuf, wifi_promiscuous_pkt_type_t type)
{
    WifiController &self = instance();
    CommFrameHeader packet_info;
    uint16_t payloadLen;
    auto *packet = static_cast<wifi_promiscuous_pkt_t *>(recvBuf);

    /* prepare packet_info */
    packet_info.seconds = htonl(packet->rx_ctrl.timestamp / 1000000U);
    packet_info.microseconds = htonl(packet->rx_ctrl.timestamp % 1000000U);
    packet_info.rssi_power = (int8_t)packet->rx_ctrl.rssi;
    packet_info.noise = (int8_t)packet->rx_ctrl.noise_floor;

    // donot use the rx_ctrl channel
    // this is the MAC channel used in resume(),
    // --> esp_wifi_set_channel(WIFI_HW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    // then overwritten by
    // --> phy_change_channel(channel, 1, 0, 0);
    packet_info.channel = htons(self.channel_);

    // convert to datarate in kbps
    packet_info.rate = htons(datarateCodeToKbps(packet->rx_ctrl.rate));

    payloadLen = packet->rx_ctrl.sig_len - WIFI_PAYLOAD_FCS_LEN;
    packet_info.frameLen = htons(payloadLen);
    /* For now, the sniffer only dumps the length of the MISC type frame */
    if (type != WIFI_PKT_MISC && !packet->rx_ctrl.rx_state)
    {
        self.queuePacket(packet->payload, payloadLen, &packet_info);
    }
}

void WifiController::loopback(const uint8_t *payload, uint16_t len, uint16_t chan, uint16_t rate)
{
    struct timespec tp;
    clock_gettime(CLOCK_REALTIME, &tp);

    CommFrameHeader packet_info;
    packet_info.seconds = htonl(tp.tv_sec);
    packet_info.microseconds = htonl(tp.tv_nsec / 1000);
    packet_info.channel = htons(chan);
    packet_info.rate = htons(rate);
    packet_info.rssi_power = 0;
    packet_info.noise = 0;
    packet_info.frameLen = htons(len);

    queuePacket(payload, len, &packet_info);
}

void WifiController::sniffTaskEntry(void *arg)
{
    static_cast<WifiController *>(arg)->sniffTask();
}

void WifiController::sniffTask()
{
    wifi_packet_t packet;
    while (running_)
    {
        /* receive packet info from queue */
        if (xQueueReceive(workQueue_, &packet, pdMS_TO_TICKS(WIFI_PROCESS_PACKET_TIMEOUT_MS)) != pdTRUE)
        {
            continue;
        }
        stats_.rx++;

        if (obu_comm_usb_connected()) UsbStream::instance().publishPacket(&packet);
        if (obu_comm_ble_connected()) BtStream::instance().publishPacket(&packet);

        esp_event_post(WIFI_EVENT_BASE, WIFI_RECEIVED_PACKET, NULL, 0, 0);
        ESP_LOGI(TAG, "RX: %d bytes", packet.length);
        free(packet.payload);
    }
    /* notify that sniffer task is over */
    xSemaphoreGive(semTaskOver_);
    vTaskDelete(NULL);
}

esp_err_t WifiController::stop()
{
    ESP_RETURN_ON_FALSE(running_, ESP_ERR_INVALID_STATE, TAG, "WiFi is already stopped");
    /* Disable wifi promiscuous mode */
    ESP_RETURN_ON_ERROR(esp_wifi_set_promiscuous(false), TAG, "stop WiFi promiscuous failed");

    ESP_LOGI(TAG, "stop WiFi promiscuous ok");
    /* stop WiFi local task */
    running_ = false;
    /* wait for task over */
    xSemaphoreTake(semTaskOver_, portMAX_DELAY);

    /* make sure to free all resources in the left items */
    UBaseType_t left_items = uxQueueMessagesWaiting(workQueue_);

    wifi_packet_t packet_info;
    while (left_items--)
    {
        xQueueReceive(workQueue_, &packet_info, pdMS_TO_TICKS(WIFI_PROCESS_PACKET_TIMEOUT_MS));
        free(packet_info.payload);
    }
    esp_event_post(WIFI_EVENT_BASE, WIFI_STOPPED, NULL, 0, 0);
    return ESP_OK;
}

esp_err_t WifiController::tuneRadio()
{
    /* Start WiFi Promiscuous Mode */
    wifi_promiscuous_filter_t wifi_filter = { .filter_mask = filter_ };
    esp_wifi_set_promiscuous_filter(&wifi_filter);
    esp_wifi_set_promiscuous_rx_cb(&WifiController::rxCallback);
    ESP_RETURN_ON_ERROR(esp_wifi_set_promiscuous(true), TAG, "set promis failed");
    //ESP_RETURN_ON_ERROR(esp_wifi_set_channel(channel_, WIFI_SECOND_CHAN_NONE), TAG, "set channel failed");
    // enable 802.11p mode (enable, unknown (must be 0))
    phy_11p_set(1, 0);
    // set a channel with a frequency close to our desired frequency (not sure if strictly needed)
    ESP_RETURN_ON_ERROR(esp_wifi_set_channel(WIFI_HW_CHANNEL, WIFI_SECOND_CHAN_NONE), TAG, "set channel failed");
    // switch channel (channel, ignored, ignored, ht_mode?)
    phy_change_channel(channel_, 1, 0, 0);
    return ESP_OK;
}

esp_err_t WifiController::start()
{
    ESP_RETURN_ON_FALSE(!running_, ESP_ERR_INVALID_STATE, TAG, "sniffer is already running");

    running_ = true;
    if (xTaskCreate(&WifiController::sniffTaskEntry,
        "tWifi",
        CONFIG_SNIFFER_TASK_STACK_SIZE,
        this,
        CONFIG_SNIFFER_TASK_PRIORITY,
        &task_) != pdPASS)
    {
        running_ = false;
        ESP_LOGE(TAG, "create task failed");
        return ESP_FAIL;
    }

    esp_err_t ret = tuneRadio();
    if (ret != ESP_OK)
    {
        // task was already started, need to shut it down gracefully
        running_ = false;
        xSemaphoreTake(semTaskOver_, portMAX_DELAY);
        task_ = nullptr;
        return ret;
    }

    uint8_t loop;
    if (config_get_u8(CONFIG_INDEX_WIFI_LOOP, &loop) == ESP_OK) {
        txLoopback_ = (loop != 0);
    }

    ESP_LOGI(TAG, "start WiFi promiscuous ok");
    esp_event_post(WIFI_EVENT_BASE, WIFI_STARTED, NULL, 0, 0);
    return ESP_OK;
}

void WifiController::autostart()
{
    uint32_t conf_channel = 5900;
    esp_err_t ret = config_get_u32(CONFIG_INDEX_AUTOSTART_CHAN, &conf_channel);
    if (ret != ESP_OK)
    {
        if (ret != ESP_ERR_NVS_NOT_FOUND)
        ESP_LOGE(TAG, "autostart: config_get_u32 failed: %s", esp_err_to_name(ret));
        return;
    }
    if (!checkValidChannel(conf_channel))
    {
        ESP_LOGE(TAG, "autostart: conf_channel invalid: %u", (unsigned)conf_channel);
        return;
    }

    channel_ = conf_channel;
    filter_ = WIFI_PROMIS_FILTER_MASK_ALL & ~WIFI_PROMIS_FILTER_MASK_FCSFAIL;
    start();
}

void WifiController::pause()
{
  if (coexHolds_.load() > 0) return;
  pauseForce();
}

void WifiController::pauseForce()
{
  if (paused_) return;

  ESP_LOGI(TAG, "pause WiFi");
  esp_wifi_set_promiscuous(false);
  phy_11p_set(0, 0);
  esp_wifi_stop();
  paused_ = true;
}

void WifiController::resume()
{
  if (coexHolds_.load() > 0) return;
  resumeForce();
}

void WifiController::resumeForce()
{
  if (!paused_) return;

  ESP_LOGI(TAG, "resume WiFi");
  esp_wifi_start();
  wifi_promiscuous_filter_t wifi_filter = { .filter_mask = filter_ };
  esp_wifi_set_promiscuous_filter(&wifi_filter);
  esp_wifi_set_promiscuous(true);
  phy_11p_set(1, 0);
  esp_wifi_set_channel(WIFI_HW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  phy_change_channel(channel_, 1, 0, 0);
  paused_ = false;
}

void WifiController::coexRelease()
{
    int n = coexHolds_.load();
    do {
        if (n <= 0) {
            ESP_LOGE(TAG, "coexRelease without matching hold");
            return;
        }
    } while (!coexHolds_.compare_exchange_weak(n, n - 1));
}

void WifiController::init()
{
  semTaskOver_ = xSemaphoreCreateBinary();
  if (!semTaskOver_)
  {
      ESP_LOGE(TAG, "create semaphore failed");
      ESP_ERROR_CHECK(ESP_FAIL);
  }

  workQueue_ = xQueueCreate(CONFIG_SNIFFER_WORK_QUEUE_LEN, sizeof(wifi_packet_t));
  if (!workQueue_)
  {
      ESP_LOGE(TAG, "create work queue failed");
      ESP_ERROR_CHECK(ESP_FAIL);
  }
}

void WifiController::setChannel(uint16_t chan)
{
    if (!checkValidChannel(chan))
    {
        ESP_LOGE(TAG, "Invalid channel value: %u", chan);
        return;
    }
    channel_ = chan;
}

void WifiController::setTxPower(int powerDbm)
{
    int8_t power = powerDbm * 4;
    if (power < 8 || power > 80)
    {
        ESP_LOGE(TAG, "Invalid tx power value: %d (2..20)", powerDbm);
        return;
    }
    while (paused_)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    esp_err_t ret = esp_wifi_set_max_tx_power(power);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_wifi_set_max_tx_power failed: %s", esp_err_to_name(ret));
        txPower_ = -1;
    }
    else
    {
        txPower_ = power / 4;
    }
}

int8_t WifiController::txPower()
{
    if (txPower_ < 0)
    {
        while (paused_)
        {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        esp_err_t ret = esp_wifi_get_max_tx_power(&txPower_);
        if (ret != ESP_OK)
        {
            txPower_ = -1;
            ESP_LOGE(TAG, "Failed read TX power: %s", esp_err_to_name(ret));
        }
        else
        {
            txPower_ = txPower_ / 4;
        }
    }
    return txPower_;
}


// ================================ TRANSMITTER =============================

namespace {

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
    esp_err_t result = ESP_OK;
    g_osi_funcs_p->_mutex_lock(g_wifi_global_lock);
    auto *eb = static_cast<x_ebuf_t *>(ic_ebuf_alloc(buffer, 1, len));
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

} // namespace

void WifiController::txFrame(uint8_t *payload, uint16_t length)
{
    if (length < sizeof(CommFrameHeader))
    {
        ESP_LOGW(TAG, "frame too short for header: %d < %d", length, (int)sizeof(CommFrameHeader));
        stats_.txErr++;
        return;
    }

    const auto *hdr = reinterpret_cast<const CommFrameHeader *>(payload);
    payload += sizeof(CommFrameHeader);
    length -= sizeof(CommFrameHeader);

    if (ntohs(hdr->frameLen) != length)
    {
        ESP_LOGW(TAG, "frame length mismatch: %d != %d", ntohs(hdr->frameLen), length);
        stats_.txErr++;
        return;
    }

    // allow channel in header as zero -> use default channel
    // yet when provided must match currently tuned channel
    uint16_t chan = ntohs(hdr->channel);
    if (chan != 0 && chan != channel_)
    {
        ESP_LOGW(TAG, "invalid channel: %u", chan);
        stats_.txErr++;
        return;
    }
    chan = channel_;

    // allow rate in header as zero -> use default rate
    uint16_t rate = ntohs(hdr->rate);
    if (rate == 0)
    {
        rate = txDatarate_;
    }
    txRateConfig_.rate = datarateKbpsToCode(rate);

    esp_err_t rc = esp_wifi_80211_tx_custom(WIFI_IF_STA,
        payload,
        length,
        false,
        &txRateConfig_,
        WIFI_BAND_5G,
        WIFI_BW20);

    if (rc != ESP_OK)
    {
        stats_.txErr++;
        ESP_LOGW(TAG, "frame tx failed: %s", esp_err_to_name(rc));
    }
    else
    {
        stats_.tx++;
        esp_event_post(WIFI_EVENT_BASE, WIFI_TRANSMIT_PACKET, NULL, 0, 0);
        ESP_LOGD(TAG, "TX: %d bytes", length);
        if (txLoopback_)
        {
            loopback(payload, length, chan, rate);
        }
    }
}

bool WifiController::txFrameRaw(uint8_t *payload, uint16_t length)
{
    // txRateConfig_ is otherwise only refreshed per header in txFrame()
    txRateConfig_.rate = datarateKbpsToCode(txDatarate_);
    esp_err_t rc = esp_wifi_80211_tx_custom(WIFI_IF_STA,
        payload,
        length,
        false,
        &txRateConfig_,
        WIFI_BAND_5G,
        WIFI_BW20);
    if (rc != ESP_OK)
    {
        stats_.txErr++;
        return false;
    }
    stats_.tx++;
    esp_event_post(WIFI_EVENT_BASE, WIFI_TRANSMIT_PACKET, NULL, 0, 0);
    return true;
}

// ============================ CCA/DCC MEASUREMENT (background) ============
// Continuous ETSI ITS-G5 DCC channel-busy-ratio measurement, built on the
// hardware validation in docs/esp_dcc_measurements.md (confirmed against a
// reference load generator across idle/~5%/~10%/~45% channel load).

#define CCA_WINDOW_CYCLES     40000000u // ~1s at the confirmed ~39.8MHz tick rate
#define CCA_WINDOW_WAIT_MS    1100      // window + margin: ~39.8MHz needs ~1005ms to reach the ceiling
#define CCA_OUTLIER_LOW_FRAC  0.5f      // drop samples below this fraction of the median: low dropouts were observed whenever a window landed in a gap of the reference generator's duty cycle, never a high spike (high spikes are the separate busy>total race, clamped below, not filtered here)
#define CCA_TASK_STACK        3072
#define CCA_TASK_PRIORITY     3

/* Mean of the history, excluding samples below CCA_OUTLIER_LOW_FRAC * median.
 * Falls back to the plain mean if that excludes everything (shouldn't happen:
 * the median itself always qualifies). Caller holds ccaMutex_. */
float WifiController::ccaFilteredAverage() const
{
    float sorted[CCA_HISTORY_LEN];
    memcpy(sorted, ccaHistory_, ccaHistoryCount_ * sizeof(float));
    qsort(sorted, ccaHistoryCount_, sizeof(float), [](const void *a, const void *b) {
        float fa = *static_cast<const float *>(a), fb = *static_cast<const float *>(b);
        return (fa > fb) - (fa < fb);
    });
    float median = sorted[ccaHistoryCount_ / 2];
    float threshold = median * CCA_OUTLIER_LOW_FRAC;

    float sum = 0;
    int n = 0;
    for (int i = 0; i < ccaHistoryCount_; i++)
    {
        if (ccaHistory_[i] >= threshold)
        {
            sum += ccaHistory_[i];
            n++;
        }
    }
    if (n == 0)
    {
        for (int i = 0; i < ccaHistoryCount_; i++) sum += ccaHistory_[i];
        n = ccaHistoryCount_;
    }
    return sum / n;
}

void WifiController::ccaTaskEntry(void *arg)
{
    static_cast<WifiController *>(arg)->ccaTask();
}

void WifiController::ccaTask()
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
            xSemaphoreTake(ccaMutex_, portMAX_DELAY);
            ccaStats_.rejected_samples++;
            xSemaphoreGive(ccaMutex_);
            continue;
        }

        uint32_t busy = raw.busy > raw.total ? raw.total : raw.busy; // busy>total race, see doc
        float percent = 100.0f * (float)busy / (float)raw.total;

        xSemaphoreTake(ccaMutex_, portMAX_DELAY);
        ccaHistory_[ccaHistoryNext_] = percent;
        ccaHistoryNext_ = (ccaHistoryNext_ + 1) % CCA_HISTORY_LEN;
        if (ccaHistoryCount_ < CCA_HISTORY_LEN) ccaHistoryCount_++;

        ccaStats_.latest_percent = percent;
        ccaStats_.avg_percent = ccaFilteredAverage();
        ccaStats_.valid_samples++;
        ccaStats_.has_data = true;
        xSemaphoreGive(ccaMutex_);
    }
}

void WifiController::ccaStart()
{
    if (ccaTask_) return;

    ccaMutex_ = xSemaphoreCreateMutex();
    if (!ccaMutex_)
    {
        ESP_LOGE(TAG, "cca: create mutex failed");
        return;
    }
    if (xTaskCreate(&WifiController::ccaTaskEntry, "tCca", CCA_TASK_STACK, this, CCA_TASK_PRIORITY, &ccaTask_) != pdTRUE)
    {
        ESP_LOGE(TAG, "cca: create task failed");
        ccaTask_ = nullptr;
    }
}

wifi_cca_stats_t WifiController::ccaStats()
{
    if (!ccaMutex_)
    {
        return {};
    }
    xSemaphoreTake(ccaMutex_, portMAX_DELAY);
    wifi_cca_stats_t stats = ccaStats_;
    xSemaphoreGive(ccaMutex_);
    return stats;
}

// ============================ CBR_IND (wire protocol push) =================
// Same data as `dccmon`, pushed as COMM_FRAME_CBR_IND over USB/BLE (whichever
// is connected) instead of printed — gated by COMM_FRAME_CMD_CBR_START/STOP
// (obu_comm.cpp). Mirrors sniffTask's existing dual-publish-if-connected pattern.

void WifiController::cbrIndPublish()
{
    wifi_cca_stats_t stats = ccaStats();

    CbrIndPayload ind = {
        .avgPercentMilli = htonl((uint32_t)(stats.avg_percent * 1000.0f)),
        .latestPercentMilli = htonl((uint32_t)(stats.latest_percent * 1000.0f)),
        .validSamples = htonl(stats.valid_samples),
        .rejectedSamples = htonl(stats.rejected_samples),
    };

    const auto *bytes = reinterpret_cast<const uint8_t *>(&ind);
    if (obu_comm_usb_connected()) UsbStream::instance().publishFrame(COMM_FRAME_CBR_IND, bytes, sizeof(ind));
    if (obu_comm_ble_connected()) BtStream::instance().publishFrame(COMM_FRAME_CBR_IND, bytes, sizeof(ind));
}

void WifiController::cbrIndTaskEntry(void *arg)
{
    static_cast<WifiController *>(arg)->cbrIndTask();
}

void WifiController::cbrIndTask()
{
    for (;;)
    {
        if (cbrIndIntervalMs_ <= 0)
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        cbrIndPublish();
        vTaskDelay(pdMS_TO_TICKS(cbrIndIntervalMs_));
    }
}

void WifiController::cbrIndStart(uint32_t intervalMs)
{
    cbrIndIntervalMs_ = (int)intervalMs;
    if (!cbrIndTask_)
    {
        xTaskCreate(&WifiController::cbrIndTaskEntry, "tCbrInd", 3072, this, 2, &cbrIndTask_);
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

    phy_cca_cnt_t prev = {};
    bool have_prev = false;

    for (int i = 0; i < samples; i++)
    {
        vTaskDelay(pdMS_TO_TICKS(interval_ms));

        // guard bytes past the 8 used bytes catch phy_get_cca_cnt writing more than expected
        uint32_t buf[8];
        for (int j = 0; j < 8; j++) buf[j] = CCA_TEST_CANARY;
        auto *out = reinterpret_cast<phy_cca_cnt_t *>(buf);
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

static int do_cca_stats_cmd(int argc, char **argv)
{
    wifi_cca_stats_t stats = WifiController::instance().ccaStats();
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

// ============================ DCCMON (background printer) =================
// `dccmon` console cmd starts/stops a background task that periodically logs
// WifiController::ccaStats()'s filtered average — a live view of the CBR measured
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

        wifi_cca_stats_t stats = WifiController::instance().ccaStats();
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

// ============================ console registration =========================

void register_cca_test_cmd(void)
{
    cca_test_args.window = arg_int0("w", "window", "<cycles>", "count ceiling passed to phy_set_cca_cnt, PHY clock cycles @ ~39.8MHz, max 0x07FFFFFF (~3.35s) (default ~1s)");
    cca_test_args.samples = arg_int0("n", "samples", "<n>", "number of samples to take (default 10)");
    cca_test_args.interval = arg_int0("i", "interval", "<ms>", "delay between samples in ms (default 1000)");
    cca_test_args.no_start = arg_lit0(NULL, "no-start", "don't call phy_set_cca_cnt, just sample current counter state");
    cca_test_args.end = arg_end(2);

    const esp_console_cmd_t cca_test_cmd = {
        .command = "ccatest",
        .help = "EXPERIMENTAL: probe undocumented PHY CCA busy/total counters (see obu_wifi.cpp comment above do_cca_test_cmd)",
        .hint = NULL,
        .func = &do_cca_test_cmd,
        .argtable = &cca_test_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cca_test_cmd));
}

void register_cca_stats_cmd(void)
{
    const esp_console_cmd_t cmd = {
        .command = "ccastats",
        .help = "show background ETSI DCC channel-busy-ratio measurement (see WifiController::ccaStart)",
        .hint = NULL,
        .func = &do_cca_stats_cmd,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
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
