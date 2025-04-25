#pragma once

#include <Arduino.h>

enum Direction : uint8_t {
    FORWARD = 190,
    BACKWARD = FORWARD - 60,
    RIGHT = FORWARD - 120,
    LEFT = FORWARD - 119,
    CORRECTION = FORWARD - 100,
    STOP = 0
};

class WheelControl {
   private:
    uint8_t in1;
    uint8_t in2;
    uint8_t in3;
    uint8_t in4;

    void smoothControl(uint8_t gpio1, uint8_t gpio2, bool isStarting);
    void setup();

   public:
    uint8_t direction = Direction::STOP;
    WheelControl();
    void attach(uint8_t in1_, uint8_t in2_, uint8_t in3_, uint8_t in4_);
    void forward(bool enableSmoothStart, bool enableSlowMode);
    void backward();
    void left();
    void right();
    void stop(bool enableSmoothStop);
    void correction(bool toRight);
};