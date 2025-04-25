#include "AutoMode.h"

#include "CornerDetector.h"
#include "Navo.h"
#include "cmath"
#define FIXED_SIDE_DISTANCE 70.0

CornerDetector cornerDetector;
double angle = 0.0;

double normilizeDiff(double current, double target) {
    return fabs(fmod((target - current + 540.0), 360.0) - 180.0);
}

void performYawCorrection() {
    if (navo.wheels.direction != Direction::FORWARD) return;

    double diff = normilizeDiff(navo.yaw, navo.targetYaw);

    if (navo.correction == Correction::NO) {
        if (fabs(diff) < 1.0) return;

        ESP_LOGI(mainLogTag, "START CORRECTION, yaw: %f, target: %f", navo.yaw,
                 navo.targetYaw);

        if (diff < 0.0)
            navo.correction = Correction::TO_LEFT;
        else
            navo.correction = Correction::TO_RIGHT;

    } else if (navo.correction == Correction::TO_RIGHT) {
        navo.wheels.correction(true);
        navo.correction = Correction::IN_PROGRESS;

    } else if (navo.correction == Correction::TO_LEFT) {
        navo.wheels.correction(false);
        navo.correction = Correction::IN_PROGRESS;

    } else if (navo.correction == Correction::IN_PROGRESS) {
        double currentDiff = normilizeDiff(navo.yaw, navo.targetYaw);
        ESP_LOGD(mainLogTag, "yaw: %f, diff: %f", navo.yaw, currentDiff);

        if (fabs(currentDiff) <= 0.5) {
            navo.wheels.forward(false, false);
            navo.correction = Correction::NO;
            ESP_LOGI(mainLogTag, "STOP CORRECTION");
        }
    }
}

void sideDistanceCorrection() {
    if (navo.hallState != HallState::HALL_TRIGGERED) return;
    double correctionAngle =
        asin((navo.distanceSide - FIXED_SIDE_DISTANCE) / navo.distanceHall) *
        180.0 / M_PI;
    ESP_LOGD(mainLogTag,
             "correction angle: %f, side distance: %f, hall distance: %f",
             correctionAngle, navo.distanceSide, navo.distanceHall);
    navo.targetYaw = navo.targetYaw - correctionAngle;
    navo.hallState = HallState::HALL_USED;
}

bool rotate() {
    static bool targetSet = false;
    static double targetAngle = 0.0;

    if (!targetSet) {
        targetAngle = fmod(navo.yaw + angle + 360.0, 360.0);
        ESP_LOGD(mainLogTag, "Rotate, yaw: %f, target angle: %f", navo.yaw,
                 targetAngle);
        targetSet = true;

        if (angle > 0.0)
            navo.wheels.left();
        else
            navo.wheels.right();
    }

    double diff = normilizeDiff(navo.yaw, targetAngle);
    ESP_LOGD(mainLogTag, "Rotate diff: %f", diff);
    if (diff < 1.0) {
        navo.targetYaw = navo.yaw;
        navo.wheels.stop(false);
        ESP_LOGI(mainLogTag, "Rotate finished");
        targetSet = false;
        return true;
    }

    return false;
}

void detectCorner() {
    CornerCheckState checkState =
        cornerDetector.getState(navo.distanceFront, navo.distanceSide);
    if (checkState == CornerCheckState::CONFIRMING_INNER) {
        navo.wheels.stop(false);
        angle = 90.0;
        return;
    } else if (checkState == CornerCheckState::CONFIRMING_OUTER) {
        navo.wheels.stop(false);
        angle = -90.0;
        return;
    } else if (checkState == CornerCheckState::UNCONFIRMED) {
        navo.wheels.forward(true, false);
        return;
    } else if (checkState == CornerCheckState::IDLE)
        return;
    if (!rotate()) return;
    cornerDetector.reset();
    navo.wheels.forward(true, false);
}

void setStartingPoint() {
    static uint8_t startPoint = StartPoint::START_POINT_NOT_SET;
    if (startPoint == StartPoint::START_POINT_NOT_SET) {
        ESP_LOGI(mainLogTag, "Auto mode enabled, set start point");
        navo.targetYaw = navo.yaw;
        angle = -90.0;
        startPoint = StartPoint::START_POINT_ROTATING;
    } else if (startPoint == StartPoint::START_POINT_ROTATING) {
        if (!rotate()) return;
        startPoint = angle < 0.0 ? StartPoint::START_POINT_SETTING
                                 : StartPoint::START_POINT_SET;
    } else if (startPoint == StartPoint::START_POINT_SETTING) {
        navo.wheels.forward(false, true);
        if (navo.distanceFront > 80.0) return;
        navo.wheels.stop(false);
        angle = 90.0;
        startPoint = StartPoint::START_POINT_ROTATING;
    } else if (startPoint == StartPoint::START_POINT_SET) {
        navo.wheels.forward(true, false);
        navo.autoMode = AutoMode::ACTIVE;
        ESP_LOGD(mainLogTag, "Initial target yaw: %f", navo.targetYaw);
        return;
    }
    return;
}

void autoMode() {
    if (navo.autoMode == AutoMode::ENABLE) {
        setStartingPoint();
    } else if (navo.autoMode == AutoMode::DISABLE or
               navo.autoMode == AutoMode::SUSPEND) {
        navo.wheels.stop(true);
    }
    if (navo.autoMode == AutoMode::ACTIVE) {
        sideDistanceCorrection();
        performYawCorrection();
        detectCorner();
    }
}