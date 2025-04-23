#include "MainLoops.h"

#include "AutoMode.h"
#include "CameraHandler.h"
#include "Navo.h"
#include "Sensors.h"
#include "WebSocketClient.h"
#include "esp_log.h"

void loopCore0(void *pvParameters);
void loopCore1(void *pvParameters);

void startMainLoops() {
    xTaskCreatePinnedToCore(loopCore0, "lc0", 8192, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(loopCore1, "lc1", 8192, NULL, 1, NULL, 1);
}

void stepLoop0() {
    double now = micros() / 1000000.0;
    navo.dt_loop0 = static_cast<float>(now - navo.t_loop0);
    navo.t_loop0 = now;

    if (!(navo.dt_loop0 > 0)) {
        navo.dt_loop0 = 0;
    }
}

void loopCore0(void *pvParameters) {
    while (true) {
        stepLoop0();
        pollAndSendData();
#if CAMERA_ENABLED
        takeImageAndSendPostRequest();
#endif
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    vTaskDelete(NULL);
}

void stepLoop1() {
    double now = micros() / 1000000.0;
    navo.dt_loop1 = static_cast<float>(now - navo.t_loop1);
    navo.t_loop1 = now;

    if (!(navo.dt_loop1 > 0)) {
        navo.dt_loop1 = 0;
    }
}

void loopCore1(void *pvParameters) {
    while (true) {
        stepLoop1();
        updateYaw();
        updateDistances();
        autoMode();
        updateVoltageAndCurrent();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    vTaskDelete(NULL);
}
