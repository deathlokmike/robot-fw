#include "AutoMode.h"

#include "Navo.h"

void performYawCorrection() {
    static const bool enableSmooth = false;

    if (navo.correction == Correction::NO and
        navo.yaw - navo.yawReference >= 2.0) {
        ESP_LOGD(mainLogTag, "Start correction, yaw: %f, yawRef: %f", navo.yaw,
                 navo.yawReference);
        navo.correction = Correction::TO_RIGHT;
    } else if (navo.correction == Correction::TO_RIGHT) {
        navo.wheels.correction(true);
        navo.correction = Correction::IN_PROGRESS;
    } else if (navo.correction == Correction::IN_PROGRESS) {
        ESP_LOGD(mainLogTag, "yaw: %f", navo.yaw);
        if (navo.yaw - navo.yawReference <= 0.5) {
            navo.wheels.forward(enableSmooth);
            navo.correction = Correction::NO;
            ESP_LOGI(mainLogTag, "STOP CORRECTION");
        }
    }
}

void autoMode() {
    if (navo.autoMode == AutoMode::ENABLE) {
        navo.wheels.forward(true);
        navo.yawReference = navo.yaw;
        navo.autoMode = AutoMode::ACTIVE;
    } else if (navo.autoMode == AutoMode::DISABLE or
               navo.autoMode == AutoMode::SUSPEND) {
        navo.wheels.stop();
    }
    if (navo.autoMode == AutoMode::ACTIVE) {
        performYawCorrection();
    }
}