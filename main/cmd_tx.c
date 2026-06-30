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

#include "obu_wifi.h"
#include "bt_stream.h"
#include "cmd_tx.h"

static const char TAG[] = "SNIFFER";

static const char* SAMPLE_PACKET_HEX = "b47000000000000000000000007988000000ffffffffffff000000000000ffffffffffff60002100aaaa0300000089471100140120500280002f010014000064cafebabe000000001cc4ff8409c98951000000000000000007d1000002020000000000004056b49d200d693a401ffffffc23b7743e00e11fc0007e3fe9ed0737feebfff6000000";

static int txInterval = -1;
static TaskHandle_t xTxHandle = NULL;

static struct {
  struct arg_int *power;
  struct arg_int *datarate;
  struct arg_int *interval;
  struct arg_lit *stop;
  struct arg_end *end;
} tx_args;

static bool hex_decode(const char *hex, uint8_t *out, size_t *outlen)
{
    size_t hexlen = strlen(hex);
    if (hexlen & 1) return false;
    size_t len = hexlen / 2;
    if (len > *outlen) return false;
    for (size_t i = 0; i < len; i++) {
        uint8_t hi = hex[2*i],     lo = hex[2*i+1];
        if      (hi >= '0' && hi <= '9') hi -= '0';
        else if (hi >= 'a' && hi <= 'f') hi -= 'a' - 10;
        else if (hi >= 'A' && hi <= 'F') hi -= 'A' - 10;
        else return false;
        if      (lo >= '0' && lo <= '9') lo -= '0';
        else if (lo >= 'a' && lo <= 'f') lo -= 'a' - 10;
        else if (lo >= 'A' && lo <= 'F') lo -= 'A' - 10;
        else return false;
        out[i] = (hi << 4) | lo;
    }
    *outlen = len;
    return true;
}

static void tx_sample_frame() {
  uint8_t frame_buffer[2*1024];
  size_t frame_len = 2*1024;
  if (!hex_decode(SAMPLE_PACKET_HEX, frame_buffer, &frame_len))
  {
    ESP_LOGE(TAG, "Failed to decode sample packet hex");
    return;
  }
  wifi_tx_frame(frame_buffer, frame_len);
}

static void tx_task(void *arg) {
  int countDown = -1;
  for (;;) {
    if (countDown > 0) {
        countDown -= 100;
    }
    if (countDown <= 0 && txInterval > 0) {
        tx_sample_frame();
        countDown = txInterval;
    }
    vTaskDelay( 100 / portTICK_PERIOD_MS );
  }
}

static int do_tx_cmd(int argc, char **argv) {
  int nerrors = arg_parse(argc, argv, (void **)&tx_args);

  if (nerrors != 0) {
    arg_print_errors(stderr, tx_args.end, argv[0]);
    return 0;
  }

  txInterval = 1000;
  if (tx_args.interval->count) {
    txInterval = tx_args.interval->ival[0];
  }
  if (tx_args.datarate->count) {
    wifi_set_datarate(tx_args.datarate->ival[0]);
  }
  if (tx_args.power->count) {
    wifi_set_txpower(tx_args.power->ival[0]);
  }
  if (tx_args.stop->count) {
    txInterval = -1;
  }

  if (xTxHandle ==  NULL) {
    wifi_coex_disable();
    xTaskCreate( tx_task, "xTx", 4096, NULL, 4, &xTxHandle);
  }
  if (txInterval > 0)
    fprintf(stdout, "Transmitting every %d millis (%d * 0.25dBm)...\n",
      txInterval,
      wifi_get_txpower());
  else
    fprintf(stdout, "Transmission stopped.\n");
  return 0;
}

void register_tx_cmd(void)
{
  tx_args.datarate = arg_int0("r", "datarate", "<n>", "datarate");
  tx_args.power = arg_int0("p", "power", "<dbm>", "power");
  tx_args.interval = arg_int0("i", "interval", "<ms>", "interval");
  tx_args.stop = arg_lit0(NULL, "stop", "stop running transmitter");
  tx_args.end = arg_end(1);

  const esp_console_cmd_t tx_cmd = {
    .command = "tx",
    .help = "Transmit sample packet",
    .hint = NULL,
    .func = &do_tx_cmd,
    .argtable = &tx_args
  };
  ESP_ERROR_CHECK(esp_console_cmd_register(&tx_cmd));
}
