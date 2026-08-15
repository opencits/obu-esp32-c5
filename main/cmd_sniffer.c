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
#include "obu_wifi.h"
#include "usb_stream.h"
#include "bt_stream.h"

#include "cmd_sniffer.h"

#define SNIFFER_PAYLOAD_FCS_LEN             (4)
#define SNIFFER_PROCESS_PACKET_TIMEOUT_MS   (100)
#define SNIFFER_RX_FCS_ERR                  (0X41)
#define SNIFFER_MAX_ETH_INTFS               (3)
#define SNIFFER_DECIMAL_NUM                 (10)

typedef struct {
    char *filter_name;
    uint32_t filter_val;
} wlan_filter_table_t;

void sniffer_init(void) {
    // Yet empty
}

static struct {
    struct arg_lit *fcsfail;
    struct arg_int *channel;
    struct arg_int *power;
    struct arg_lit *status;
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
        wifi_stop();
        return 0;
    }

    if (sniffer_args.power->count) {
        int tx_power = sniffer_args.power->ival[0];
        wifi_set_txpower(tx_power);
        return 0;
    }

    uint32_t channel = wifi_get_channel();
    if (sniffer_args.channel->count) {
        channel = sniffer_args.channel->ival[0];
        wifi_set_channel(channel);
    }

    if (sniffer_args.status->count) {
        int tx_power = wifi_get_txpower();
        fprintf(stdout, "tx power %d dBm, frequency %lu MHz\n", tx_power, channel);
        return 0;
    }

    uint32_t filter = WIFI_PROMIS_FILTER_MASK_ALL;
    if (!sniffer_args.fcsfail->count) {
        filter &= ~WIFI_PROMIS_FILTER_MASK_FCSFAIL;
    }
    wifi_set_filter(filter);


    /* start sniffer */
    wifi_start();
    return 0;
}

void register_sniffer_cmd(void)
{
    sniffer_args.fcsfail = arg_lit0("F", "fcsfail", "include corrupted packets with wrong FCS");
    sniffer_args.channel = arg_int0("c", "channel", "<channel freq>", "frequency of communication channel to use (G5CC = 5900, G5SC2 = 5890, G5SC1 = 5880, G5SC3 = 5870, G5SC4 = 5860)");
    sniffer_args.power = arg_int0("p", "power", "<tx power>", "transmit power in dBm (2..20)");
    sniffer_args.status = arg_lit0(NULL, "status", "show sniffer status");
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
