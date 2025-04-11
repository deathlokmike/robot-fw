#include "WheelControl.h"

WheelControl::WheelControl() {};

void WheelControl::attach(uint8_t in1_, uint8_t in2_, uint8_t in3_,
                          uint8_t in4_) {
    in1 = in1_;
    in2 = in2_;
    in3 = in3_;
    in4 = in4_;
    setup();
}

void WheelControl::setup() {
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
}

void WheelControl::smoothControl(uint8_t gpio1, uint8_t gpio2,
                                 bool isStarting) {
    const uint8_t steps = 10;
    const uint8_t delay_time = 10;
    int step = direction / steps;

    if (!isStarting) step = -step;

    for (int i = isStarting ? 0 : direction;
         isStarting ? (i <= direction) : (i >= 0); i += step) {
        analogWrite(gpio1, i);
        analogWrite(gpio2, i);
        delay(delay_time);
    }

    analogWrite(gpio1, isStarting ? direction : 0);
    analogWrite(gpio2, isStarting ? direction : 0);
}

void WheelControl::forward() {
    analogWrite(in1, 0);
    analogWrite(in3, 0);
    direction = wheel_direction::forward;
    smoothControl(in2, in4, true);
}

void WheelControl::backward() {
    analogWrite(in2, 0);
    analogWrite(in4, 0);
    direction = wheel_direction::backward;
    smoothControl(in1, in3, true);
}

void WheelControl::left() {
    analogWrite(in2, 0);
    analogWrite(in3, 0);
    direction = wheel_direction::left;
    smoothControl(in1, in4, true);
}
void WheelControl::right() {
    analogWrite(in1, 0);
    analogWrite(in4, 0);
    direction = wheel_direction::right;
    smoothControl(in2, in3, true);
}

void WheelControl::stop() {
    switch (direction) {
        case wheel_direction::forward:
            smoothControl(in2, in4, false);
            break;
        case wheel_direction::backward:
            smoothControl(in1, in3, false);
            break;
        case wheel_direction::left:
            smoothControl(in1, in4, false);
            break;
        case wheel_direction::right:
            smoothControl(in2, in3, false);
            break;
        default:
            break;
    }
    direction = -1;
    analogWrite(in1, 0);
    analogWrite(in2, 0);
    analogWrite(in3, 0);
    analogWrite(in4, 0);
}
