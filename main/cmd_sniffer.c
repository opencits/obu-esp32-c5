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
#include "esp_app_trace.h"
#include "esp_check.h"
#include "esp_console.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs.h"

#include "pcap.h"
#include "config.h"
#include "events.h"

#include "obu_comm.h"
#include "usb_stream.h"
#include "bt_stream.h"

#include "cmd_sniffer.h"

#define SNIFFER_DEFAULT_CHANNEL             (1)
#define SNIFFER_PAYLOAD_FCS_LEN             (4)
#define SNIFFER_PROCESS_PACKET_TIMEOUT_MS   (100)
#define SNIFFER_RX_FCS_ERR                  (0X41)
#define SNIFFER_MAX_ETH_INTFS               (3)
#define SNIFFER_DECIMAL_NUM                 (10)


static const char TAG[] = "SNIFFER";

typedef struct {
    char *filter_name;
    uint32_t filter_val;
} wlan_filter_table_t;

static bool is_running;

static sniffer_intf_t interf;
static uint32_t channel;
static uint32_t filter;
static bool only_capture_broadcast;
static TaskHandle_t task;
static QueueHandle_t work_queue;
static SemaphoreHandle_t sem_task_over;

static esp_err_t sniffer_stop();

static void queue_packet(void *recv_packet, uint16_t len, const CommRecvHeader *packet_info)
{
    sniffer_packet_t packet;
    /* Copy a packet from Link Layer driver and queue the copy to be processed by sniffer task */
    packet.payload = malloc(sizeof(CommRecvHeader) + len);
    packet.length = sizeof(CommRecvHeader) + len;
    if (packet.payload) {
        memcpy(packet.payload, packet_info, sizeof(CommRecvHeader));
        memcpy(packet.payload + sizeof(CommRecvHeader), recv_packet, len);
        if (work_queue) {
            /* send packet_info */
            if (xQueueSend(work_queue, &packet, pdMS_TO_TICKS(SNIFFER_PROCESS_PACKET_TIMEOUT_MS)) != pdTRUE) {
                ESP_LOGE(TAG, "sniffer work queue full");
                free(packet.payload);
            }
        }
    } else {
        ESP_LOGE(TAG, "No enough memory for promiscuous packet");
    }
}

static void wifi_sniffer_cb(void *recv_buf, wifi_promiscuous_pkt_type_t type)
{
    CommRecvHeader packet_info;
    uint16_t payloadLen;
    wifi_promiscuous_pkt_t *packet = (wifi_promiscuous_pkt_t *)recv_buf;
    /* prepare packet_info */
    packet_info.seconds = htonl(packet->rx_ctrl.timestamp / 1000000U);
    packet_info.microseconds = htonl(packet->rx_ctrl.timestamp % 1000000U);
    packet_info.channel = (uint8_t)packet->rx_ctrl.channel;
    packet_info.rate = (uint8_t)packet->rx_ctrl.rate;
    packet_info.rssi = (int8_t)packet->rx_ctrl.rssi;

#if CONFIG_SOC_WIFI_HE_SUPPORT
    payloadLen = packet->rx_ctrl.dump_len;
#else
    payloadLen = packet->rx_ctrl.sig_len - SNIFFER_PAYLOAD_FCS_LEN;
#endif

    packet_info.frameLen = htons(payloadLen);
    /* For now, the sniffer only dumps the length of the MISC type frame */
    if (type != WIFI_PKT_MISC && !packet->rx_ctrl.rx_state) {
        // Ignore non-broadcast frames
        if (only_capture_broadcast &&
                (payloadLen < (4 + 6) || memcmp(&packet->payload[4], "\xFF\xFF\xFF\xFF\xFF\xFF", 6)))
            return;

        queue_packet(packet->payload, payloadLen, &packet_info);
    }
}

static void sniffer_task(void *)
{
    sniffer_packet_t packet;

    while (is_running) {
        /* receive packet info from queue */
        if (xQueueReceive(work_queue, &packet, pdMS_TO_TICKS(SNIFFER_PROCESS_PACKET_TIMEOUT_MS)) != pdTRUE) {
            continue;
        }

        usb_stream_publish_packet(&packet);

        bt_stream_publish_packet(&packet);

        esp_event_post(SNIFFER_EVENT_BASE, SNIFFER_RECEIVED_PACKET, NULL, 0, 0);

        free(packet.payload);
    }
    /* notify that sniffer task is over */
    xSemaphoreGive(sem_task_over);
    vTaskDelete(NULL);
}

static esp_err_t sniffer_stop()
{
    esp_err_t ret = ESP_OK;

    ESP_GOTO_ON_FALSE(is_running, ESP_ERR_INVALID_STATE, err, TAG, "sniffer is already stopped");

    switch (interf) {
    case SNIFFER_INTF_WLAN:
        /* Disable wifi promiscuous mode */
        ESP_GOTO_ON_ERROR(esp_wifi_set_promiscuous(false), err, TAG, "stop wifi promiscuous failed");
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_INVALID_ARG, err, TAG, "unsupported interface");
        break;
    }
    ESP_LOGI(TAG, "stop promiscuous ok");

    /* stop sniffer local task */
    is_running = false;
    /* wait for task over */
    xSemaphoreTake(sem_task_over, portMAX_DELAY);

    /* make sure to free all resources in the left items */
    UBaseType_t left_items = uxQueueMessagesWaiting(work_queue);

    sniffer_packet_t packet_info;
    while (left_items--) {
        xQueueReceive(work_queue, &packet_info, pdMS_TO_TICKS(SNIFFER_PROCESS_PACKET_TIMEOUT_MS));
        free(packet_info.payload);
    }

    esp_event_post(SNIFFER_EVENT_BASE, SNIFFER_STOPPED, NULL, 0, 0);

err:
    return ret;
}

void phy_change_channel(int,int,int,int);
void phy_11p_set(int,int);

static esp_err_t sniffer_start()
{
    esp_err_t ret = ESP_OK;
    wifi_promiscuous_filter_t wifi_filter;

    ESP_GOTO_ON_FALSE(!is_running, ESP_ERR_INVALID_STATE, err, TAG, "sniffer is already running");

    is_running = true;
    ESP_GOTO_ON_FALSE(xTaskCreate(sniffer_task, "snifferT", CONFIG_SNIFFER_TASK_STACK_SIZE,
                                  NULL, CONFIG_SNIFFER_TASK_PRIORITY, &task), ESP_FAIL,
                      err_task, TAG, "create task failed");

    switch (interf) {
    case SNIFFER_INTF_WLAN:
        /* Start WiFi Promiscuous Mode */
        wifi_filter.filter_mask = filter;
        esp_wifi_set_promiscuous_filter(&wifi_filter);
        esp_wifi_set_promiscuous_rx_cb(wifi_sniffer_cb);
        ESP_GOTO_ON_ERROR(esp_wifi_set_promiscuous(true), err_start, TAG, "set promis failed");
        //ESP_GOTO_ON_ERROR(esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE), err_start, TAG, "set channel failed");
        // enable 802.11p mode (enable, unknown (must be 0))
        phy_11p_set(1, 0);
        // set a channel with a frequency close to our desired frequency (not sure if strictly needed)
        ESP_GOTO_ON_ERROR(esp_wifi_set_channel(140, WIFI_SECOND_CHAN_NONE), err_start, TAG, "set channel failed");
        // switch channel (channel, ignored, ignored, ht_mode?)
        phy_change_channel(channel, 1, 0, 0);
        ESP_LOGI(TAG, "start WiFi promiscuous ok");
        break;
    default:
        break;
    }

    esp_event_post(SNIFFER_EVENT_BASE, SNIFFER_STARTED, NULL, 0, 0);

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

static struct {
    struct arg_str *interface;
    struct arg_lit *fcsfail;
    struct arg_int *channel;
    struct arg_lit *stop;
    struct arg_end *end;
} sniffer_args;

static int do_sniffer_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&sniffer_args);

    if (nerrors != 0) {
        arg_print_errors(stderr, sniffer_args.end, argv[0]);
        return 0;
    }

    /* Check whether or not to stop sniffer: "--stop" option */
    if (sniffer_args.stop->count) {
        /* stop sniffer */
        sniffer_stop();
        return 0;
    }

    /* Check interface: "-i" option */
    if (sniffer_args.interface->count) {
        if (!strncmp(sniffer_args.interface->sval[0], "wlan", 4)) {
            interf = SNIFFER_INTF_WLAN;
        } else {
            ESP_LOGE(TAG, "interface %s not found", sniffer_args.interface->sval[0]);
            return 1;
        }
    } else {
        interf = SNIFFER_INTF_WLAN;
        ESP_LOGI(TAG, "sniffing interface set to wlan by default");
    }

    /* Check channel: "-c" option */
    switch (interf) {
    case SNIFFER_INTF_WLAN:
        channel = SNIFFER_DEFAULT_CHANNEL;
        if (sniffer_args.channel->count) {
            channel = sniffer_args.channel->ival[0];
        }
        break;
    default:
        break;
    }

    /* Check filter setting: "-F" option */
    switch (interf) {
    case SNIFFER_INTF_WLAN:
            filter = WIFI_PROMIS_FILTER_MASK_ALL;
            if (!sniffer_args.fcsfail->count)
                filter &= ~WIFI_PROMIS_FILTER_MASK_FCSFAIL;
        break;
    default:
        break;
    }

    uint8_t broadcast_only;
    esp_err_t ret = config_get_u8(CONFIG_INDEX_BROADCAST_ONLY, &broadcast_only);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "config_get_u8 failed: %s", ret);
        return 1;
    }

    only_capture_broadcast = broadcast_only;

    /* start sniffer */
    sniffer_start();
    return 0;
}

void register_sniffer_cmd(void)
{
    sniffer_args.interface = arg_str0("i", "interface", "wlan|...",
                                      "which interface to capture packet");
    sniffer_args.fcsfail = arg_lit0("F", "fcsfail", "include corrupted packets with wrong FCS");
    sniffer_args.channel = arg_int0("c", "channel", "<channel freq>", "frequency of communication channel to use (G5CC = 5900, G5SC2 = 5890, G5SC1 = 5880, G5SC3 = 5870, G5SC4 = 5860)");
    sniffer_args.stop = arg_lit0(NULL, "stop", "stop running sniffer");
    sniffer_args.end = arg_end(1);
    const esp_console_cmd_t sniffer_cmd = {
        .command = "sniffer",
        .help = "Capture specific packet",
        .hint = NULL,
        .func = &do_sniffer_cmd,
        .argtable = &sniffer_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&sniffer_cmd));
}

void sniffer_init(void)
{
    esp_err_t ret = ESP_OK;

    sem_task_over = xSemaphoreCreateBinary();
    ESP_GOTO_ON_FALSE(sem_task_over, ESP_FAIL, err, TAG, "create semaphore failed");

    work_queue = xQueueCreate(CONFIG_SNIFFER_WORK_QUEUE_LEN, sizeof(sniffer_packet_t));
    ESP_GOTO_ON_FALSE(work_queue, ESP_FAIL, err, TAG, "create work queue failed");

err:
    ESP_ERROR_CHECK(ret);
}

void sniffer_autostart(void)
{
    uint32_t conf_channel;
    esp_err_t ret = config_get_u32(CONFIG_INDEX_AUTOSTART_CHAN, &conf_channel);
    if (ret != ESP_OK)
    {
        if (ret != ESP_ERR_NVS_NOT_FOUND)
            ESP_LOGE(TAG, "config_get_u32 failed: %s", ret);
        return;
    }

    uint8_t broadcast_only;
    ret = config_get_u8(CONFIG_INDEX_BROADCAST_ONLY, &broadcast_only);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "config_get_u8 failed: %s", ret);
        return;
    }

    if (conf_channel >= 5800 && conf_channel <= 5900)
    {
        interf = SNIFFER_INTF_WLAN;
        channel = conf_channel;
        filter = WIFI_PROMIS_FILTER_MASK_ALL & ~WIFI_PROMIS_FILTER_MASK_FCSFAIL;
        only_capture_broadcast = broadcast_only;
        sniffer_start();
    }
}

static bool s_paused;

void sniffer_pause(void)
{
    if (s_paused) return;
    if (interf != SNIFFER_INTF_WLAN) return;
    esp_wifi_set_promiscuous(false);
    phy_11p_set(0, 0);
    esp_wifi_stop();
    s_paused = true;
}

void sniffer_resume(void)
{
    if (!s_paused) return;
    if (interf != SNIFFER_INTF_WLAN) return;
    esp_wifi_start();
    wifi_promiscuous_filter_t wifi_filter = { .filter_mask = filter };
    esp_wifi_set_promiscuous_filter(&wifi_filter);
    esp_wifi_set_promiscuous(true);
    phy_11p_set(1, 0);
    esp_wifi_set_channel(140, WIFI_SECOND_CHAN_NONE);
    phy_change_channel(channel, 1, 0, 0);
    s_paused = false;
}
