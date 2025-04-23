#include "WiFiManager.h"

#include <WiFi.h>

#include "Config.h"
#include "esp_log.h"

void connectToWiFi() {
    WiFi.begin(ssid, password);
    WiFi.setSleep(false);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
    }
    ESP_LOGI(mainLogTag, "Connected to WiFi");
}