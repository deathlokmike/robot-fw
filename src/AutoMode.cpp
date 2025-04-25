#include "AutoMode.h"

#include "CornerDetector.h"
#include "Navo.h"

CornerDetector cornerDetector;

void performYawCorrection() {
    if (navo.wheels.direction != Direction::FORWARD) return;
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
            navo.wheels.forward(false);
            navo.correction = Correction::NO;
            ESP_LOGI(mainLogTag, "STOP CORRECTION");
        }
    }
}

void sideDistanceCorrection() {
    if (navo.distanceHall <= 0.0) return;
}

bool rotate(double angle) {
    ESP_LOGD(mainLogTag, "Rotate, yaw: %f, set angle: %f", navo.yaw,
             navo.yawReference + angle);
    if (angle > 0.0)
        navo.wheels.left();
    else
        navo.wheels.right();
    if (navo.yaw >= navo.yawReference + angle) {
        navo.yawReference = navo.yaw;
        navo.wheels.stop();
        return true;
    }
    return false;
}

void detectCorner() {
    CornerCheckState checkState =
        cornerDetector.getState(navo.distanceFront, navo.distanceSide);
    if (checkState == CornerCheckState::CONFIRMING) {
        navo.wheels.stop();
        return;
    } else if (checkState == CornerCheckState::UNCONFIRMED) {
        navo.wheels.forward(true);
        return;
    } else if (checkState == CornerCheckState::IDLE)
        return;
    if (!rotate(90)) return;
    ESP_LOGI(mainLogTag, "Rotate finished");
    cornerDetector.reset();
    navo.wheels.forward(true);
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
        sideDistanceCorrection();
        performYawCorrection();
        detectCorner();
    }
}