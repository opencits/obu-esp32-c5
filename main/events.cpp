#include "sdkconfig.h"

#include "esp_event.h"

#include "events.h"

ESP_EVENT_DEFINE_BASE(CONFIG_EVENT_BASE);

ESP_EVENT_DEFINE_BASE(WIFI_EVENT_BASE);

ESP_EVENT_DEFINE_BASE(COMM_EVENT_BASE);
