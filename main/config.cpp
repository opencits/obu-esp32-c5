#include "sdkconfig.h"

#include <array>
#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "esp_console.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "mbedtls/base64.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "argtable3/argtable3.h"

#include "events.h"
#include "obu_wifi.h"

#include "config.h"

static const char TAG[] = "CONFIG";

static nvs_handle_t handle;

namespace
{

/* Type-erased override hook; cast back to the key's typed getter/setter
 * signature (see Getter/Setter below) before calling. */
using AnyFn = void (*)();

struct ConfigKey
{
    char name[NVS_KEY_NAME_MAX_SIZE];
    nvs_type_t type;
    size_t buffer_size; // STR/BLOB only: max bytes incl. terminator
    AnyFn getter;       // optional override of the plain NVS read
    AnyFn setter;       // optional override of the plain NVS write
};

template <typename F>
AnyFn erase(F* fn)
{
    return reinterpret_cast<AnyFn>(fn);
}

/* Per-type NVS binding so one get/set template covers all integer widths. */
template <typename T>
struct NvsTraits;
#define NVS_TRAITS(T, TYPE, SUFFIX)                                                                \
    template <>                                                                                    \
    struct NvsTraits<T>                                                                            \
    {                                                                                              \
        static constexpr nvs_type_t type = TYPE;                                                   \
        static constexpr const char* name = #SUFFIX;                                               \
        static esp_err_t get(const char* k, T* out) { return nvs_get_##SUFFIX(handle, k, out); }   \
        static esp_err_t set(const char* k, T v) { return nvs_set_##SUFFIX(handle, k, v); }        \
    };
NVS_TRAITS(uint8_t, NVS_TYPE_U8, u8)
NVS_TRAITS(uint16_t, NVS_TYPE_U16, u16)
NVS_TRAITS(uint32_t, NVS_TYPE_U32, u32)
NVS_TRAITS(uint64_t, NVS_TYPE_U64, u64)
NVS_TRAITS(int8_t, NVS_TYPE_I8, i8)
NVS_TRAITS(int16_t, NVS_TYPE_I16, i16)
NVS_TRAITS(int32_t, NVS_TYPE_I32, i32)
NVS_TRAITS(int64_t, NVS_TYPE_I64, i64)
#undef NVS_TRAITS

template <typename T>
using Getter = esp_err_t (*)(T* out);
template <typename T>
using Setter = esp_err_t (*)(T val);
using StrGetter = esp_err_t (*)(char* out, size_t* size);
using StrSetter = esp_err_t (*)(const char* val);
using BlobGetter = esp_err_t (*)(uint8_t* out, size_t* size);
using BlobSetter = esp_err_t (*)(const uint8_t* val, size_t size);

esp_err_t config_get_node_id(char* out, size_t* size);
esp_err_t config_get_autostart_chan(uint32_t* autostart_chan);
esp_err_t config_get_led_brightness(uint8_t* led_brightness);

esp_err_t config_set_tx_power(int8_t tx_power);
esp_err_t config_set_loop(uint8_t loop);

constexpr size_t CONFIG_KEY_COUNT = CONFIG_INDEX_WIFI_LOOP + 1;

// Filled by index so entries can't drift out of sync with config_index_t order.
const std::array<ConfigKey, CONFIG_KEY_COUNT> config_keys = []
{
    std::array<ConfigKey, CONFIG_KEY_COUNT> k{};
    k[CONFIG_INDEX_NODEID] = {
        "nodeid", NVS_TYPE_STR, CONFIG_NODEID_BUFFER_SIZE, erase(config_get_node_id), nullptr};
    k[CONFIG_INDEX_ETH_IP] = {"ethip", NVS_TYPE_STR, CONFIG_IPV4_BUFFER_SIZE, nullptr, nullptr};
    k[CONFIG_INDEX_ETH_NETMASK] = {
        "ethnm", NVS_TYPE_STR, CONFIG_IPV4_BUFFER_SIZE, nullptr, nullptr};
    k[CONFIG_INDEX_ETH_GATEWAY] = {
        "ethgw", NVS_TYPE_STR, CONFIG_IPV4_BUFFER_SIZE, nullptr, nullptr};
    k[CONFIG_INDEX_ETH_DNS0] = {"ethdns0", NVS_TYPE_STR, CONFIG_IPV4_BUFFER_SIZE, nullptr, nullptr};
    k[CONFIG_INDEX_ETH_DNS1] = {"ethdns1", NVS_TYPE_STR, CONFIG_IPV4_BUFFER_SIZE, nullptr, nullptr};
    k[CONFIG_INDEX_ETH_DNS2] = {"ethdns2", NVS_TYPE_STR, CONFIG_IPV4_BUFFER_SIZE, nullptr, nullptr};
    k[CONFIG_INDEX_AUTOSTART_CHAN] = {
        "autostartchan", NVS_TYPE_U32, 0, erase(config_get_autostart_chan), nullptr};
    k[CONFIG_INDEX_LED_BRIGHTNESS] = {
        "ledbrightness", NVS_TYPE_U8, 0, erase(config_get_led_brightness), nullptr};
    k[CONFIG_INDEX_WIFI1_SSID] = {
        "wifi1ssid", NVS_TYPE_STR, CONFIG_WIFI_SSID_BUFFER_SIZE, nullptr, nullptr};
    k[CONFIG_INDEX_WIFI1_PASS] = {
        "wifi1pass", NVS_TYPE_STR, CONFIG_WIFI_PASS_BUFFER_SIZE, nullptr, nullptr};
    k[CONFIG_INDEX_WIFI2_SSID] = {
        "wifi2ssid", NVS_TYPE_STR, CONFIG_WIFI_SSID_BUFFER_SIZE, nullptr, nullptr};
    k[CONFIG_INDEX_WIFI2_PASS] = {
        "wifi2pass", NVS_TYPE_STR, CONFIG_WIFI_PASS_BUFFER_SIZE, nullptr, nullptr};
    k[CONFIG_INDEX_WIFI_OPEN] = {"wifiopen", NVS_TYPE_U8, 0, nullptr, nullptr};
    k[CONFIG_INDEX_CONN_MODE] = {"connmode", NVS_TYPE_U8, 0, nullptr, nullptr};
    k[CONFIG_INDEX_WIFI_IP] = {"wifiip", NVS_TYPE_STR, CONFIG_IPV4_BUFFER_SIZE, nullptr, nullptr};
    k[CONFIG_INDEX_WIFI_NM] = {"wifinm", NVS_TYPE_STR, CONFIG_IPV4_BUFFER_SIZE, nullptr, nullptr};
    k[CONFIG_INDEX_WIFI_GW] = {"wifigw", NVS_TYPE_STR, CONFIG_IPV4_BUFFER_SIZE, nullptr, nullptr};
    k[CONFIG_INDEX_WIFI_DNS] = {"wifidns", NVS_TYPE_STR, CONFIG_IPV4_BUFFER_SIZE, nullptr, nullptr};
    k[CONFIG_INDEX_SNIFF_MODE] = {"sniffmode", NVS_TYPE_U8, 0, nullptr, nullptr};
    k[CONFIG_INDEX_SNIFF_MS] = {"sniffms", NVS_TYPE_U32, 0, nullptr, nullptr};
    k[CONFIG_INDEX_WIFI_MS] = {"wifims", NVS_TYPE_U32, 0, nullptr, nullptr};
    k[CONFIG_INDEX_WIFI_TX_POWER] = {
        "txpower", NVS_TYPE_I8, 0, nullptr, erase(config_set_tx_power)};
    k[CONFIG_INDEX_WIFI_LOOP] = {"loop", NVS_TYPE_U8, 0, nullptr, erase(config_set_loop)};
    return k;
}();

void post_changed(config_index_t index)
{
    esp_err_t post_result = esp_event_post(CONFIG_EVENT_BASE, (int32_t)index, NULL, 0, 0);
    if (post_result != ESP_OK)
        ESP_LOGE(TAG, "esp_event_post failed: %s", esp_err_to_name(post_result));
}

template <typename T>
esp_err_t config_get(config_index_t index, T* out)
{
    const ConfigKey& key = config_keys[index];

    if (key.type != NvsTraits<T>::type)
    {
        ESP_LOGE(TAG, "Attempt to get key %s with wrong type %s", key.name, NvsTraits<T>::name);
        return ESP_ERR_INVALID_ARG;
    }

    if (key.getter)
        return reinterpret_cast<Getter<T>>(key.getter)(out);
    else
        return NvsTraits<T>::get(key.name, out);
}

template <typename T>
esp_err_t config_set(config_index_t index, T value)
{
    const ConfigKey& key = config_keys[index];

    if (key.type != NvsTraits<T>::type)
    {
        ESP_LOGE(TAG, "Attempt to set key %s with wrong type %s", key.name, NvsTraits<T>::name);
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t result;
    if (key.setter)
        result = reinterpret_cast<Setter<T>>(key.setter)(value);
    else
        result = NvsTraits<T>::set(key.name, value);

    if (result == ESP_OK)
        post_changed(index);

    return result;
}

esp_err_t config_get_node_id(char* out, size_t* size)
{
    size_t size_copy = *size;

    esp_err_t res = nvs_get_str(handle, "nodeid", out, &size_copy);
    if (res != ESP_OK)
    {
        if (res != ESP_ERR_NVS_NOT_FOUND)
            ESP_LOGW(TAG, "nvs_get_str failed: %s", esp_err_to_name(res));

        int print_res;
        {
            uint8_t eth_mac[6];
            ESP_ERROR_CHECK(esp_read_mac(eth_mac, ESP_MAC_ETH));

            print_res = snprintf(out,
                *size,
                "%02hhx%02hhx%02hhx%02hhx%02hhx%02hhx",
                eth_mac[0],
                eth_mac[1],
                eth_mac[2],
                eth_mac[3],
                eth_mac[4],
                eth_mac[5]);
        }

        if (print_res > (int)*size - 1)
        {
            ESP_LOGE(TAG, "buffer too small to print");
            return ESP_ERR_INVALID_SIZE;
        }

        if (print_res < 0)
        {
            ESP_LOGE(TAG, "snprintf failed");
            return ESP_ERR_INVALID_STATE;
        }

        size_copy = print_res + 1;
    }
    *size = size_copy;
    return ESP_OK;
}

esp_err_t config_get_autostart_chan(uint32_t* autostart_chan)
{
    uint32_t out;
    esp_err_t res = nvs_get_u32(handle, "autostartchan", &out);
    if (res != ESP_OK)
    {
        if (res != ESP_ERR_NVS_NOT_FOUND)
            ESP_LOGE(TAG, "nvs_get_u32 failed: %s", esp_err_to_name(res));
        out = 5900;
    }

    *autostart_chan = out;
    return ESP_OK;
}

esp_err_t config_get_led_brightness(uint8_t* led_brightness)
{
    uint8_t out;
    esp_err_t res = nvs_get_u8(handle, "ledbrightness", &out);
    if (res != ESP_OK)
    {
        if (res != ESP_ERR_NVS_NOT_FOUND)
            ESP_LOGE(TAG, "nvs_get_u8 failed: %s", esp_err_to_name(res));
        out = 255;
    }
    *led_brightness = out;
    return ESP_OK;
}

esp_err_t config_set_tx_power(int8_t tx_power)
{
    if (tx_power < 2 || tx_power > 20)
    {
        ESP_LOGE(TAG, "Invalid tx power value: %d", tx_power);
        return ESP_ERR_INVALID_ARG;
    }
    WifiController::instance().setTxPower(
        tx_power); // takes dBm, does the *4 quarter-dBm scaling itself
    return nvs_set_i8(handle, config_keys[CONFIG_INDEX_WIFI_TX_POWER].name, tx_power);
}

esp_err_t config_set_loop(uint8_t loop)
{
    WifiController::instance().setLoopback(loop != 0);
    return nvs_set_u8(handle, config_keys[CONFIG_INDEX_WIFI_LOOP].name, loop);
}

} // namespace

void config_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(nvs_open("its", NVS_READWRITE, &handle));
}

esp_err_t config_get_u8(config_index_t index, uint8_t* out)
{
    return config_get(index, out);
}
esp_err_t config_set_u8(config_index_t index, uint8_t value)
{
    return config_set(index, value);
}
esp_err_t config_get_u16(config_index_t index, uint16_t* out)
{
    return config_get(index, out);
}
esp_err_t config_set_u16(config_index_t index, uint16_t value)
{
    return config_set(index, value);
}
esp_err_t config_get_u32(config_index_t index, uint32_t* out)
{
    return config_get(index, out);
}
esp_err_t config_set_u32(config_index_t index, uint32_t value)
{
    return config_set(index, value);
}
esp_err_t config_get_u64(config_index_t index, uint64_t* out)
{
    return config_get(index, out);
}
esp_err_t config_set_u64(config_index_t index, uint64_t value)
{
    return config_set(index, value);
}
esp_err_t config_get_i8(config_index_t index, int8_t* out)
{
    return config_get(index, out);
}
esp_err_t config_set_i8(config_index_t index, int8_t value)
{
    return config_set(index, value);
}
esp_err_t config_get_i16(config_index_t index, int16_t* out)
{
    return config_get(index, out);
}
esp_err_t config_set_i16(config_index_t index, int16_t value)
{
    return config_set(index, value);
}
esp_err_t config_get_i32(config_index_t index, int32_t* out)
{
    return config_get(index, out);
}
esp_err_t config_set_i32(config_index_t index, int32_t value)
{
    return config_set(index, value);
}
esp_err_t config_get_i64(config_index_t index, int64_t* out)
{
    return config_get(index, out);
}
esp_err_t config_set_i64(config_index_t index, int64_t value)
{
    return config_set(index, value);
}

esp_err_t config_get_str(config_index_t index, char* out, size_t* size)
{
    const ConfigKey& key = config_keys[index];

    if (key.type != NVS_TYPE_STR)
    {
        ESP_LOGE(TAG, "Attempt to get key %s with wrong type str", key.name);
        return ESP_ERR_INVALID_ARG;
    }

    if (key.getter)
        return reinterpret_cast<StrGetter>(key.getter)(out, size);
    else
        return nvs_get_str(handle, key.name, out, size);
}

esp_err_t config_set_str(config_index_t index, const char* value)
{
    const ConfigKey& key = config_keys[index];

    if (key.type != NVS_TYPE_STR)
    {
        ESP_LOGE(TAG, "Attempt to set key %s with wrong type str", key.name);
        return ESP_ERR_INVALID_ARG;
    }

    if (strlen(value) >= key.buffer_size)
        return ESP_ERR_INVALID_SIZE;

    esp_err_t result;
    if (key.setter)
        result = reinterpret_cast<StrSetter>(key.setter)(value);
    else
        result = nvs_set_str(handle, key.name, value);

    if (result == ESP_OK)
        post_changed(index);

    return result;
}

esp_err_t config_get_blob(config_index_t index, uint8_t* out, size_t* size)
{
    const ConfigKey& key = config_keys[index];

    if (key.type != NVS_TYPE_BLOB)
    {
        ESP_LOGE(TAG, "Attempt to get key %s with wrong type blob", key.name);
        return ESP_ERR_INVALID_ARG;
    }

    if (key.getter)
        return reinterpret_cast<BlobGetter>(key.getter)(out, size);
    else
        return nvs_get_blob(handle, key.name, out, size);
}

esp_err_t config_set_blob(config_index_t index, const uint8_t* val, size_t size)
{
    const ConfigKey& key = config_keys[index];

    if (key.type != NVS_TYPE_BLOB)
    {
        ESP_LOGE(TAG, "Attempt to set key %s with wrong type blob", key.name);
        return ESP_ERR_INVALID_ARG;
    }

    if (size > key.buffer_size)
        return ESP_ERR_INVALID_SIZE;

    esp_err_t result;
    if (key.setter)
        result = reinterpret_cast<BlobSetter>(key.setter)(val, size);
    else
        result = nvs_set_blob(handle, key.name, val, size);

    if (result == ESP_OK)
        post_changed(index);

    return result;
}

static const ConfigKey* config_key_find(const char* name)
{
    for (const ConfigKey* it = config_keys.begin(); it != config_keys.end(); it++)
    {
        if (strcmp(name, it->name))
            continue;

        return it;
    }

    return NULL;
}

static struct
{
    arg_str_t* key;
    arg_str_t* value;
    arg_end_t* end;
} config_set_args;

static int cmd_config_set(int argc, char** argv)
{
    int nerrors = arg_parse(argc, argv, (void**)&config_set_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, config_set_args.end, argv[0]);
        return 1;
    }

    const char* key_str = config_set_args.key->sval[0];
    const char* value_str = config_set_args.value->sval[0];

    const ConfigKey* key = config_key_find(key_str);
    if (key == NULL)
    {
        ESP_LOGE(TAG, "No such config key: %s", key_str);
        return 1;
    }

    config_index_t index = (config_index_t)(key - config_keys.data());

    esp_err_t res;
    switch (key->type)
    {
        case NVS_TYPE_U8:
        case NVS_TYPE_U16:
        case NVS_TYPE_U32:
        case NVS_TYPE_U64:
        {
            char* end;
            uint64_t parsed = strtoull(value_str, &end, 0);
            if (*end != '\0')
            {
                ESP_LOGE(TAG, "Invalid unsigned integer");
                return 1;
            }

            if ((key->type == NVS_TYPE_U8 && parsed > UINT8_MAX) ||
                (key->type == NVS_TYPE_U16 && parsed > UINT16_MAX) ||
                (key->type == NVS_TYPE_U32 && parsed > UINT32_MAX))
            {
                ESP_LOGE(TAG, "Value out of range for type");
                return 1;
            }

            switch (key->type)
            {
                case NVS_TYPE_U8:
                    res = config_set_u8(index, (uint8_t)parsed);
                    break;
                case NVS_TYPE_U16:
                    res = config_set_u16(index, (uint16_t)parsed);
                    break;
                case NVS_TYPE_U32:
                    res = config_set_u32(index, (uint32_t)parsed);
                    break;
                case NVS_TYPE_U64:
                    res = config_set_u64(index, parsed);
                    break;
                default:
                    __builtin_unreachable();
            }
        }

        break;
        case NVS_TYPE_I8:
        case NVS_TYPE_I16:
        case NVS_TYPE_I32:
        case NVS_TYPE_I64:
        {
            char* end;
            int64_t parsed = strtoll(value_str, &end, 0);
            if (*end != '\0')
            {
                ESP_LOGE(TAG, "Invalid signed integer");
                return 1;
            }

            if ((key->type == NVS_TYPE_I8 && (parsed < INT8_MIN || parsed > INT8_MAX)) ||
                (key->type == NVS_TYPE_I16 && (parsed < INT16_MIN || parsed > INT16_MAX)) ||
                (key->type == NVS_TYPE_I32 && (parsed < INT32_MIN || parsed > INT32_MAX)))
            {
                ESP_LOGE(TAG, "Value out of range for type");
                return 1;
            }

            switch (key->type)
            {
                case NVS_TYPE_I8:
                    res = config_set_i8(index, (int8_t)parsed);
                    break;
                case NVS_TYPE_I16:
                    res = config_set_i16(index, (int16_t)parsed);
                    break;
                case NVS_TYPE_I32:
                    res = config_set_i32(index, (int32_t)parsed);
                    break;
                case NVS_TYPE_I64:
                    res = config_set_i64(index, parsed);
                    break;
                default:
                    __builtin_unreachable();
            }
        }

        break;
        case NVS_TYPE_STR:
            if (strlen(value_str) >= key->buffer_size)
            {
                ESP_LOGE(TAG, "Value too long");
                return 1;
            }

            res = config_set_str(index, value_str);
            break;
        case NVS_TYPE_BLOB:
        {
            unsigned char* buf = static_cast<unsigned char*>(malloc(key->buffer_size));
            size_t olen;
            int result = mbedtls_base64_decode(
                buf, key->buffer_size, &olen, (const uint8_t*)value_str, strlen(value_str));
            if (result != 0)
            {
                ESP_LOGE(TAG, "mbedtls_base64_decode failed: %#x", result);
                free(buf);
                return 1;
            }

            if (olen > key->buffer_size)
            {
                ESP_LOGE(TAG, "Value too long");
                free(buf);
                return 1;
            }

            res = config_set_blob(index, buf, olen);
            free(buf);
        }
        break;
        default:
            __builtin_unreachable();
            // avoid compiler warning
            res = ESP_ERR_INVALID_STATE;
    }

    if (res != ESP_OK)
    {
        ESP_LOGE(TAG, "set failed: %s", esp_err_to_name(res));
        return 1;
    }

    return 0;
}

static void register_config_set(void)
{
    config_set_args.key = arg_str1(NULL, NULL, "key", "the key");
    config_set_args.value = arg_str1(NULL, NULL, "value", "the value to set");
    config_set_args.end = arg_end(1);

    const esp_console_cmd_t cmd = {.command = "config_set",
        .help = "set a config key",
        .hint = NULL,
        .func = &cmd_config_set,
        .argtable = &config_set_args};
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

static struct
{
    arg_str_t* key;
    arg_end_t* end;
} config_get_args;

static int _print_config_key(const ConfigKey* key)
{
    config_index_t index = (config_index_t)(key - config_keys.data());

    esp_err_t res;

    switch (key->type)
    {
        case NVS_TYPE_U8:
        case NVS_TYPE_U16:
        case NVS_TYPE_U32:
        case NVS_TYPE_U64:
        {
            uint64_t value = 0;

            switch (key->type)
            {
                case NVS_TYPE_U8:
                    res = config_get_u8(index, (uint8_t*)&value);
                    break;
                case NVS_TYPE_U16:
                    res = config_get_u16(index, (uint16_t*)&value);
                    break;
                case NVS_TYPE_U32:
                    res = config_get_u32(index, (uint32_t*)&value);
                    break;
                case NVS_TYPE_U64:
                    res = config_get_u64(index, &value);
                    break;
                default:
                    __builtin_unreachable();
            }

            if (res == ESP_OK)
                printf("%" PRIu64 "\n", value);
        }

        break;
        case NVS_TYPE_I8:
        case NVS_TYPE_I16:
        case NVS_TYPE_I32:
        case NVS_TYPE_I64:
        {
            int64_t value = 0;

            switch (key->type)
            {
                case NVS_TYPE_I8:
                    res = config_get_i8(index, (int8_t*)&value);
                    value = (int64_t)(int8_t)value;
                    break;
                case NVS_TYPE_I16:
                    res = config_get_i16(index, (int16_t*)&value);
                    value = (int64_t)(int16_t)value;
                    break;
                case NVS_TYPE_I32:
                    res = config_get_i32(index, (int32_t*)&value);
                    value = (int64_t)(int32_t)value;
                    break;
                case NVS_TYPE_I64:
                    res = config_get_i64(index, &value);
                    break;
                default:
                    __builtin_unreachable();
            }

            if (res == ESP_OK)
                printf("%" PRIi64 "\n", value);
        }

        break;
        case NVS_TYPE_STR:
        {
            char* value = static_cast<char*>(malloc(key->buffer_size));
            size_t size = key->buffer_size;

            res = config_get_str(index, value, &size);

            if (res == ESP_OK)
                printf("%s\n", value);

            free(value);
        }
        break;
        case NVS_TYPE_BLOB:
        {
            unsigned char* buf = static_cast<unsigned char*>(malloc(key->buffer_size));
            size_t actual_size = key->buffer_size;

            res = config_get_blob(index, buf, &actual_size);

            if (res != ESP_OK)
            {
                free(buf);
                break;
            }

            const size_t base64_buffer_size = (((4 * key->buffer_size / 3) + 3) & ~3) + 1;
            unsigned char* base64_buf = static_cast<unsigned char*>(malloc(base64_buffer_size));
            size_t olen;
            int result = mbedtls_base64_encode(
                base64_buf, base64_buffer_size, &olen, (const uint8_t*)buf, actual_size);
            if (result != 0)
            {
                ESP_LOGE(TAG, "mbedtls_base64_encode failed: %#x", result);
                free(buf);
                free(base64_buf);
                return 1;
            }

            printf("%s\n", base64_buf);
            free(buf);
            free(base64_buf);
        }
        break;
        default:
            __builtin_unreachable();
            // avoid compiler warning
            res = ESP_ERR_INVALID_STATE;
    }

    if (res != ESP_OK)
    {
        ESP_LOGE(TAG, "get failed: %s", esp_err_to_name(res));
        return 1;
    }

    return 0;
}

static int cmd_config_get(int argc, char** argv)
{
    int nerrors = arg_parse(argc, argv, (void**)&config_get_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, config_get_args.end, argv[0]);
        return 1;
    }

    const char* key_str = config_get_args.key->sval[0];
    if (key_str[0] == '*')
    {
        for (const ConfigKey* it = config_keys.begin(); it != config_keys.end(); it++)
        {
            int rc;
            printf("%s = ", it->name);
            rc = _print_config_key(it);
            if (rc != ESP_OK)
            {
                printf("  FAILED: %d\n", rc);
            }
        }
        return 0;
    }

    const ConfigKey* key = config_key_find(key_str);
    if (key == NULL)
    {
        ESP_LOGE(TAG, "No such config key: %s", key_str);
        return 1;
    }
    return _print_config_key(key);
}

static void register_config_get(void)
{
    config_get_args.key = arg_str1(NULL, NULL, "key", "the key");
    config_get_args.end = arg_end(1);

    const esp_console_cmd_t cmd = {.command = "config_get",
        .help = "get a config key",
        .hint = NULL,
        .func = &cmd_config_get,
        .argtable = &config_get_args};
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

static struct
{
    arg_str_t* key;
    arg_end_t* end;
} config_clear_args;

static int cmd_config_clear(int argc, char** argv)
{
    int nerrors = arg_parse(argc, argv, (void**)&config_clear_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, config_clear_args.end, argv[0]);
        return 1;
    }

    int ret = 0;
    for (int i = 0; i < config_clear_args.key->count; i++)
    {
        int res = nvs_erase_key(handle, config_clear_args.key->sval[i]);
        if (res != ESP_OK && res != ESP_ERR_NVS_NOT_FOUND)
        {
            ESP_LOGE(TAG, "nvs_erase_key failed: %s", esp_err_to_name(res));
            ret = 1;
        }
    }

    return ret;
}

static void register_config_clear(void)
{
    config_clear_args.key = arg_strn(NULL, NULL, "key", 1, 20, "the key(s) to clear");
    config_clear_args.end = arg_end(1);

    const esp_console_cmd_t cmd = {.command = "config_clear",
        .help = "clear one or more config keys",
        .hint = NULL,
        .func = &cmd_config_clear,
        .argtable = &config_clear_args};
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

void config_register_commands(void)
{
    register_config_set();
    register_config_get();
    register_config_clear();
}
