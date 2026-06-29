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

#include "tx_custom.h"
#include "cmd_tx.h"

static const char TAG[] = "SNIFFER";

static const char* SAMPLE_PACKET_HEX = "b47000000000000000000000007988000000ffffffffffff000000000000ffffffffffff60002100aaaa0300000089471100140120500280002f010014000064cafebabe000000001cc4ff8409c98951000000000000000007d1000002020000000000004056b49d200d693a401ffffffc23b7743e00e11fc0007e3fe9ed0737feebfff6000000";

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
  handle_tx_frame(frame_buffer, frame_len);
}

static struct {
  struct arg_int *count;
  struct arg_end *end;
} tx_args;

static int do_tx_cmd(int argc, char **argv) {
  int nerrors = arg_parse(argc, argv, (void **)&tx_args);

  if (nerrors != 0) {
    arg_print_errors(stderr, tx_args.end, argv[0]);
    return 0;
  }

  int count = 1;
  if (tx_args.count->count) {
    count = tx_args.count->ival[0];
  }

  tx_sample_frame();

  fprintf(stdout, "Transmitting %d packets...\n", count);
  return 0;
}

void register_tx_cmd(void)
{
  tx_args.count = arg_int0("c", "count", "<n>", "number of packets to transmit");
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

bool set_tx_power(int8_t tx_power) {
  return esp_wifi_set_max_tx_power(tx_power * 4) == ESP_OK;
}
