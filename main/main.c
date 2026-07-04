#include "sdkconfig.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "hal/modem_syscon_ll.h"
#include "linenoise/linenoise.h"

#include "esp_console.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "esp_wifi.h"

#include "cmd_sniffer.h"
#include "cmd_tx.h"

#include "config.h"
#include "events.h"
#include "led.h"
#include "ota.h"
#include "spi.h"
#include "obu_wifi.h"
#include "usb_stream.h"
#include "bt_stream.h"

#if CONFIG_SNIFFER_STORE_HISTORY
#define HISTORY_MOUNT_POINT "/data"
#define HISTORY_FILE_PATH HISTORY_MOUNT_POINT "/history.txt"
#endif

static const char TAG[] = "MAIN";

/* Initialize filesystem */
static void initialize_filesystem(void)
{
#if CONFIG_SNIFFER_STORE_HISTORY
    static wl_handle_t wl_handle;
    const esp_vfs_fat_mount_config_t mount_config = {
        .max_files = 4,
        .format_if_mount_failed = true
    };
    esp_err_t err = esp_vfs_fat_spiflash_mount_rw_wl(HISTORY_MOUNT_POINT, "storage", &mount_config, &wl_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount FATFS (%s)", esp_err_to_name(err));
        return;
    }
#endif
}

/* Initialize wifi with tcp/ip adapter */
static void initialize_wifi(void)
{
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    modem_syscon_ll_enable_fe_40m_clock(&MODEM_SYSCON, 1);
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void setup_wifi_power(void)
{
    int8_t tx_power;
    esp_err_t ret = config_get_i8(CONFIG_INDEX_WIFI_TX_POWER, &tx_power);
    if (ret == ESP_OK)
    {
        if (tx_power < 2 || tx_power > 20) {
            ESP_LOGE(TAG, "TX power invalid: %d", tx_power);
        }
        else {
            wifi_set_txpower(tx_power);
        }
    }
    fprintf(stdout, "uOBU (%d dBm) version: %s\n", wifi_get_txpower(), COMM_VERSION);
}

static esp_console_repl_t *repl;
static void console_init(void)
{
    // 31 buffer size - length of colors and 1 space after prompt - 1 closing angle bracket - 1 null byte
    const size_t max_nodeid_length = 31 - (sizeof(LOG_COLOR_I " " LOG_RESET_COLOR) - 1) - 1 - 1;

    char prompt[CONFIG_NODEID_BUFFER_SIZE];
    size_t size = sizeof(prompt);
    ESP_ERROR_CHECK(config_get_str(CONFIG_INDEX_NODEID, prompt, &size));

    if (size - 1 > max_nodeid_length)
        strcpy(&prompt[max_nodeid_length - 3], "...");

    strcat(prompt, ">");

    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
#if CONFIG_SNIFFER_STORE_HISTORY
    repl_config.history_save_path = HISTORY_FILE_PATH;
#endif
    repl_config.prompt = prompt;

    // install console REPL environment
#if CONFIG_ESP_CONSOLE_UART
    esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_config, &repl_config, &repl));
#elif CONFIG_ESP_CONSOLE_USB_CDC
    esp_console_dev_usb_cdc_config_t cdc_config = ESP_CONSOLE_DEV_CDC_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_cdc(&cdc_config, &repl_config, &repl));
#elif CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    esp_console_dev_usb_serial_jtag_config_t usbjtag_config = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&usbjtag_config, &repl_config, &repl));
#endif
}

int cmd_reboot(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    esp_restart();

    return 1;
}

void register_reboot(void)
{
    const esp_console_cmd_t cmd = {
        .command = "reboot",
        .help = "reboot the device",
        .hint = NULL,
        .func = &cmd_reboot,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

int cmd_log_level(int argc, char **argv)
{
    if (argc > 1) {
        const char* tag = argv[1];
        esp_log_level_t level = esp_log_level_get(tag);
        if (level > ESP_LOG_NONE) {
            if (argc > 2) {
                esp_log_level_t levelNew = atoi(argv[2]);
                if (levelNew < ESP_LOG_ERROR || levelNew > ESP_LOG_VERBOSE) {
                    fprintf(stderr, "Invalid log level %d\n", levelNew);
                    return 1;
                }
                if (levelNew != level) {
                    esp_log_level_set(tag, levelNew);
                }
            }
            fprintf(stdout, "Level(%s) = %d\n", tag, esp_log_level_get(tag));
        }
        else {
            fprintf(stderr, "Tag %s is unknown\n", tag);
            return 1;
        }
    }
    return 0;
}

void register_loglevel(void)
{
    const esp_console_cmd_t cmd = {
        .command = "loglevel",
        .help = "set log level",
        .hint = "Usage: loglevel TAG <ERROR=1 .. ESP_LOG_VERBOSE=5> ",
        .func = &cmd_log_level,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

void app_main(void)
{
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_timer_init();

    config_init();

    led_init();

    initialize_filesystem();

    initialize_spi();

    /*--- Initialize Network ---*/
    /* Initialize WiFi */
    initialize_wifi();

    /*--- Initialize Console ---*/
    console_init();

    wifi_init();
    sniffer_init();
    ota_init();

    /* Register commands */
    register_sniffer_cmd();

    config_register_commands();

    register_reboot();

    register_loglevel();

    register_ota_cmd();

    register_tx_cmd();

    usb_stream_init();
    usb_config_rx_start();   /* USB RX config channel for setup wizard */
    bt_stream_init();

    wifi_autostart();     /* start WIFI — sets phy_11p_set(1,0) */

    usb_stream_send_ping();

    // start console REPL
    ESP_ERROR_CHECK(esp_console_start_repl(repl));

    led_update();

    setup_wifi_power();
}
