#include "sdkconfig.h"

#include <atomic>
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
#include "esp_random.h"
#include "nvs.h"

#include "obu_wifi.h"
#include "bt_stream.h"
#include "cmd_tx.h"

static const char TAG[] = "TX";

/* Bare 34-byte 802.11 QoS-data header (addr1/addr3 broadcast, addr2 zero) +
 * LLC/SNAP for the GeoNetworking ethertype 0x8947; random payload follows. */
static const char* ITS_HEADER_HEX =
    "88000000ffffffffffff000000000000ffffffffffff60002100aaaa030000008947";

#define TX_DEFAULT_INTERVAL_MS 1000
#define TX_MAX_INTERVAL_MS     3600000u  // keeps pdMS_TO_TICKS' ms*HZ product in 32 bit
#define TX_DEFAULT_PAYLOAD_LEN 128
#define TX_STATS_WINDOW_MS     1000

/* ms between frames, <= 0 = stopped; tick-quantized (CONFIG_FREERTOS_HZ). */
static std::atomic<int> txInterval{-1};
/* random payload bytes after the header, 0..txMaxPayload(). */
static std::atomic<int> txLength{TX_DEFAULT_PAYLOAD_LEN};

static uint8_t txFrame[WIFI_TX_MAX_FRAME_LEN];
static size_t txHeaderLen = 0;  // 0 until tx_header_init() decoded ITS_HEADER_HEX

static TaskHandle_t xTxHandle = NULL;

/* written by tx_task only, averaged over TX_STATS_WINDOW_MS; 0 while stopped */
static float framesPerSecond = 0.0f;
static float bytesPerSecond = 0.0f;
static std::atomic<uint32_t> errorCount{0};

static struct
{
    arg_int* power;
    arg_int* datarate;
    arg_int* interval;
    arg_int* length;
    arg_lit* stop;
    struct arg_end* end;
} tx_args;

static bool hex_decode(const char* hex, uint8_t* out, size_t* outlen)
{
    size_t hexlen = strlen(hex);
    if (hexlen & 1)
        return false;
    size_t len = hexlen / 2;
    if (len > *outlen)
        return false;
    for (size_t i = 0; i < len; i++)
    {
        uint8_t hi = hex[2 * i], lo = hex[2 * i + 1];
        if (hi >= '0' && hi <= '9')
            hi -= '0';
        else if (hi >= 'a' && hi <= 'f')
            hi -= 'a' - 10;
        else if (hi >= 'A' && hi <= 'F')
            hi -= 'A' - 10;
        else
            return false;
        if (lo >= '0' && lo <= '9')
            lo -= '0';
        else if (lo >= 'a' && lo <= 'f')
            lo -= 'a' - 10;
        else if (lo >= 'A' && lo <= 'F')
            lo -= 'A' - 10;
        else
            return false;
        out[i] = (hi << 4) | lo;
    }
    *outlen = len;
    return true;
}

static bool tx_header_init()
{
    if (txHeaderLen != 0)
        return true;
    size_t len = sizeof(txFrame);
    if (!hex_decode(ITS_HEADER_HEX, txFrame, &len))
    {
        ESP_LOGE(TAG, "Failed to decode sample header hex");
        return false;
    }
    txHeaderLen = len;
    return true;
}

static int txMaxPayload()
{
    return (int)(WIFI_TX_MAX_FRAME_LEN - txHeaderLen);
}

/* xorshift32: esp_fill_random busy-waits per word, too slow to refill every frame */
static void fill_payload(uint8_t* p, size_t n, uint32_t* state)
{
    uint32_t x = *state;
    for (size_t i = 0; i < n; i += 4)
    {
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        memcpy(p + i, &x, (n - i < 4) ? n - i : 4);
    }
    *state = x;
}

static void tx_task(void* arg)
{
    uint32_t rng = esp_random() | 1;  // xorshift state must be non-zero
    uint32_t frames = 0, bytes = 0;
    TickType_t last = xTaskGetTickCount();
    TickType_t windowStart = last;

    for (;;)
    {
        int interval = txInterval.load();
        if (interval <= 0)
        {
            framesPerSecond = 0.0f;
            bytesPerSecond = 0.0f;
            frames = bytes = 0;
            vTaskDelay(pdMS_TO_TICKS(50));
            last = windowStart = xTaskGetTickCount();
            continue;
        }
        TickType_t period = pdMS_TO_TICKS(interval);
        vTaskDelayUntil(&last, period ? period : 1);

        size_t payloadLen = (size_t)txLength.load();
        size_t len = txHeaderLen + payloadLen;
        fill_payload(txFrame + txHeaderLen, payloadLen, &rng);
        if (WifiController::instance().txFrameRaw(txFrame, (uint16_t)len))
        {
            frames++;
            bytes += len;
        }
        else
        {
            errorCount++;
        }

        TickType_t now = xTaskGetTickCount();
        if (now - windowStart >= pdMS_TO_TICKS(TX_STATS_WINDOW_MS))
        {
            float dt = (float)((now - windowStart) * portTICK_PERIOD_MS) / 1000.0f;
            framesPerSecond = (float)frames / dt;
            bytesPerSecond = (float)bytes / dt;
            frames = bytes = 0;
            windowStart = now;
        }
    }
}

/* Must run before setTxPower(): that spins while the radio is paused. */
static bool tx_task_ensure()
{
    if (xTxHandle != NULL)
        return true;
    if (!tx_header_init())
        return false;
    /* never released: tx task lives until reboot. Force the radio active too —
     * a hold landing in a BLE window would otherwise freeze it paused. */
    WifiController::instance().coexHold();
    WifiController::instance().resumeForce();
    if (xTaskCreate(tx_task, "xTx", 4096, NULL, 4, &xTxHandle) != pdPASS)
    {
        ESP_LOGE(TAG, "create tx task failed");
        WifiController::instance().coexRelease();
        xTxHandle = NULL;
        return false;
    }
    return true;
}

static bool tx_power_valid(int dbm)
{
    return dbm >= 2 && dbm <= 20;  // WifiController::setTxPower range
}

static int do_tx_cmd(int argc, char** argv)
{
    int nerrors = arg_parse(argc, argv, (void**)&tx_args);

    if (nerrors != 0)
    {
        arg_print_errors(stderr, tx_args.end, argv[0]);
        return 0;
    }

    if (tx_args.stop->count)
    {
        txInterval = -1;
        fprintf(stdout, "Transmission stopped.\n");
        return 0;
    }

    if (!tx_header_init())
        return 1;
    int length = tx_args.length->count ? tx_args.length->ival[0] : txLength.load();
    int interval = tx_args.interval->count ? tx_args.interval->ival[0] : TX_DEFAULT_INTERVAL_MS;
    if (length < 0 || length > txMaxPayload())
    {
        fprintf(stderr, "length must be 0..%d\n", txMaxPayload());
        return 1;
    }
    if (interval <= 0 || (uint32_t)interval > TX_MAX_INTERVAL_MS)
    {
        fprintf(stderr, "interval must be 1..%lu ms\n", (unsigned long)TX_MAX_INTERVAL_MS);
        return 1;
    }
    if (tx_args.power->count && !tx_power_valid(tx_args.power->ival[0]))
    {
        fprintf(stderr, "power must be 2..20 dBm\n");
        return 1;
    }

    if (!tx_task_ensure())
        return 1;
    if (tx_args.datarate->count)
    {
        WifiController::instance().setDatarate(tx_args.datarate->ival[0]);
    }
    if (tx_args.power->count)
    {
        WifiController::instance().setTxPower(tx_args.power->ival[0]);
    }
    txLength = length;
    txInterval = interval;

    fprintf(stdout,
        "Transmitting %d bytes every %d millis (%d * 0.25dBm)...\n",
        (int)txHeaderLen + length,
        interval,
        WifiController::instance().txPower());
    return 0;
}

void register_tx_cmd(void)
{
    tx_args.datarate = arg_int0("r", "datarate", "<kbps>", "datarate");
    tx_args.power = arg_int0("p", "power", "<dbm>", "power 2..20");
    tx_args.interval = arg_int0("i", "interval", "<ms>", "interval");
    tx_args.length = arg_int0("l", "length", "<bytes>", "random payload bytes after the 34-byte header");
    tx_args.stop = arg_lit0(NULL, "stop", "stop running transmitter");
    tx_args.end = arg_end(1);

    const esp_console_cmd_t tx_cmd = {.command = "tx",
        .help = "Transmit sample packets with random payload",
        .hint = NULL,
        .func = &do_tx_cmd,
        .argtable = &tx_args};
    ESP_ERROR_CHECK(esp_console_cmd_register(&tx_cmd));
}

void start_tx_loop(uint8_t* frame, uint16_t frame_len, obu_comm_reply_fn reply, void* ctx)
{
    if (frame_len != sizeof(TxStartPayload) || !tx_header_init())
    {
        reply(COMM_FRAME_CMD_FAIL, NULL, 0, ctx);
        return;
    }
    TxStartPayload req;
    memcpy(&req, frame, sizeof(req));
    uint32_t interval = ntohl(req.intervalMs);
    uint32_t length = ntohl(req.frameLen);
    int32_t power = (int32_t)ntohl((uint32_t)req.txPower);
    uint32_t datarate = ntohl(req.datarate);

    if (interval > TX_MAX_INTERVAL_MS || length > (uint32_t)txMaxPayload() ||
        (power != 0 && !tx_power_valid(power)) || datarate > UINT16_MAX)
    {
        ESP_LOGW(TAG, "TX_LOOP_START out of range: %lu ms, %lu bytes, %ld dBm, %lu kbps",
                 (unsigned long)interval, (unsigned long)length, (long)power, (unsigned long)datarate);
        reply(COMM_FRAME_CMD_FAIL, NULL, 0, ctx);
        return;
    }
    if (!tx_task_ensure())
    {
        reply(COMM_FRAME_CMD_FAIL, NULL, 0, ctx);
        return;
    }
    if (datarate != 0)
    {
        WifiController::instance().setDatarate((uint16_t)datarate);
    }
    if (power != 0)
    {
        WifiController::instance().setTxPower(power);
    }
    if (length != 0)
    {
        txLength = (int)length;
    }
    txInterval = interval ? (int)interval : TX_DEFAULT_INTERVAL_MS;
    reply(COMM_FRAME_CMD_GOOD, NULL, 0, ctx);
}

void stop_tx_loop(obu_comm_reply_fn reply, void* ctx)
{
    txInterval = -1;
    reply(COMM_FRAME_CMD_GOOD, NULL, 0, ctx);
}

void status_tx_loop(obu_comm_reply_fn reply, void* ctx)
{
    char buffer[256];

    int len = snprintf(buffer, sizeof(buffer), "{\"fps\":%.02f", framesPerSecond);
    len += snprintf(buffer + len, sizeof(buffer) - len, ",\"bps\":%.02f", bytesPerSecond);
    len += snprintf(buffer + len, sizeof(buffer) - len, ",\"err\":%lu", (unsigned long)errorCount.load());
    len += snprintf(buffer + len, sizeof(buffer) - len, ",\"interval\":%d", txInterval.load());
    len += snprintf(buffer + len, sizeof(buffer) - len, ",\"len\":%d", (int)txHeaderLen + txLength.load());
    len += snprintf(buffer + len, sizeof(buffer) - len, "}");

    // own type, not CMD_GOOD: those aren't request-correlated (see FW_ACK/FW_NAK)
    reply(COMM_FRAME_TX_LOOP_STATUS, (const uint8_t*)buffer, len, ctx);
}
