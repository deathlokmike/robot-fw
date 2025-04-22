#pragma once
#include <Arduino.h>

#include "WheelControl.h"

enum Correction : uint8_t { TO_RIGHT, TO_LEFT, NO, IN_PROGRESS };

enum AutoMode : uint8_t { ENABLE, ACTIVE, DISABLE, SUSPEND, MANUAL };

class Navo {
   public:
    bool isAngle;

    double distanceFront;
    double distanceSide;
    double distanceHall;
    double yaw;
    double yawReference;
    double t_loop0;
    double t_loop1;

    float dt_loop0;
    float dt_loop1;
    float voltage;
    float current;

    uint8_t autoMode = AutoMode::DISABLE;
    uint8_t correction = Correction::NO;
    uint8_t previousDirection = Direction::STOP;

    inline String getStr() {
        return "vol:" + String(voltage) + ",cur:" + String(current) +
               ",df:" + String(distanceFront) + ",ds:" + String(distanceSide) +
               ",ang:" + String(yaw) + ",t:" + String(t_loop1) +
               ",dh:" + String(distanceHall);
    }
};
