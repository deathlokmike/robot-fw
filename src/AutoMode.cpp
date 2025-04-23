#include "AutoMode.h"

#include "Navo.h"

void doCorrection() {
    static const bool enableSmooth = false;
    static float correctionTime = 0.0;
    if (navo.autoMode != AutoMode::ACTIVE ||
        navo.correction == Correction::IN_PROGRESS ||
        fabs(navo.yawReference - navo.yaw) <= 1.5)
        return;
    ESP_LOGI(mainLogTag, "Start correction, diff: %f",
             navo.yawReference - navo.yaw);
    navo.correction = navo.yawReference < navo.yaw ? Correction::TO_RIGHT
                                                   : Correction::TO_LEFT;
    if (navo.correction == Correction::NO)
        return;
    else if (navo.correction == Correction::TO_LEFT) {
        navo.wheels.correction(false);
        navo.correction = Correction::IN_PROGRESS;
    } else if (navo.correction == Correction::TO_RIGHT) {
        navo.wheels.correction(true);
        navo.correction = Correction::IN_PROGRESS;
    } else {
        ESP_LOGD(mainLogTag, "correction time: %f", correctionTime);
        correctionTime += navo.dt_loop1;
        if (correctionTime <= 0.2f) return;
        navo.wheels.forward(enableSmooth);
        navo.correction = Correction::NO;
        correctionTime = 0.0f;
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
        doCorrection();
    }
}