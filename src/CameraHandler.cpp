#include "CameraHandler.h"

#if CAMERA_ENABLED
#include <HTTPClient.h>

#include "Config.h"
#include "OV7670.h"
#include "esp_log.h"

static HTTPClient client;
static OV7670 camera;

void setupCamera() {
    if (gpio_install_isr_service(ESP_INTR_FLAG_IRAM) != ESP_OK) {
        ESP_LOGE(mainLogTag, "Failed to install ISR service");
    }
    if (!camera.init(CAM_VSYNC, CAM_HREF, CAM_XCLK, CAM_PCLK, CAM_D0, CAM_D1,
                     CAM_D2, CAM_D3, CAM_D4, CAM_D5, CAM_D6, CAM_D7)) {
        ESP_LOGE(mainLogTag, "Camera initialization failed!");
        return;
    }
    ESP_LOGI(mainLogTag, "Camera: Successful");
}

void takeImageAndSendPostRequest() {
    camera.oneFrame();
    client.begin(endpoint);
    int httpResponseCode =
        client.sendRequest("POST", camera.frame, camera.frameBytes);
    if (httpResponseCode > 0) {
        String payload = client.getString();
        ESP_LOGI(mainLogTag, "HTTP: [%d] %s", httpResponseCode,
                 payload.c_str());
    } else {
        ESP_LOGW(mainLogTag, "HTTP-response error");
    }
    client.end();
}
#endif