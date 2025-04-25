#pragma once
#include <Arduino.h>

enum StartPoint : uint8_t {
    START_POINT_NOT_SET,
    START_POINT_SETTING,
    START_POINT_ROTATING,
    START_POINT_SET
};

void autoMode();