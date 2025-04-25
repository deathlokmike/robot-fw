#pragma once
#include <Arduino.h>

enum class CornerCheckState : uint8_t {
    IDLE,
    CONFIRMING_INNER,
    CONFIRMING_OUTER,
    CONFIRMED,
    UNCONFIRMED
};

class CornerDetector {
   public:
    CornerDetector(double threshold = 200);  // millimeters

    CornerCheckState getState(double distanceFront, double distanceSide);

    void reset();

   private:
    double threshold;
    CornerCheckState state;
    uint8_t confirmationCount;
};
