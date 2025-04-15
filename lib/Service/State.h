#pragma once
#include <Arduino.h>

#include "WheelControl.h"

enum Correction : uint8_t { TO_RIGHT, TO_LEFT, NO };

class State {
   public:
    bool handleMode;
    bool autoMode;
    bool isAngle;

    double distanceFront;
    double distanceSide;
    double distanceHall;
    double angle;
    double referenceAngle;
    double t;

    float dt;
    float busVoltage;
    float current_mA;

    Correction correction = Correction::NO;
    Direction direction = Direction::STOP;

    inline String getStr() {
        return "vol:" + String(busVoltage) + ",cur:" + String(current_mA) +
               ",df:" + String(distanceFront) + ",ds:" + String(distanceSide) +
               ",ang:" + String(angle) + ",t:" + String(t) +
               ",dh:" + String(distanceHall);
    }
};
