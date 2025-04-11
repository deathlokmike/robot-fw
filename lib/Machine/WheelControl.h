#pragma once
#include <Arduino.h>

enum wheel_direction : uint8_t {
    forward = 190,
    backward = 130,
    right = forward - 120,
    left = forward - 119
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
    uint8_t direction = -1;
    WheelControl();
    void attach(uint8_t in1_, uint8_t in2_, uint8_t in3_, uint8_t in4_);
    void forward();
    void backward();
    void left();
    void right();
    void stop();
};