#include "CornerDetector.h"

CornerDetector::CornerDetector(double threshold)
    : threshold(threshold),
      state(CornerCheckState::IDLE),
      confirmationCount(0) {}

CornerCheckState CornerDetector::getState(double distanceFront,
                                          double distanceSide) {
    switch (state) {
        case CornerCheckState::IDLE:
            if (distanceFront < threshold && distanceFront > 0.0 &&
                distanceSide < threshold && distanceSide > 0.0) {
                state = CornerCheckState::CONFIRMING_INNER;
                confirmationCount = 0;
            }
            break;

        case CornerCheckState::CONFIRMING_INNER:
            if (distanceFront < threshold && distanceFront > 0.0 &&
                distanceSide < threshold && distanceSide > 0.0) {
                confirmationCount++;
            } else {
                state = CornerCheckState::UNCONFIRMED;
                break;
            }

            if (confirmationCount >= 3) {
                state = CornerCheckState::CONFIRMED;
            }
            break;

        case CornerCheckState::CONFIRMED:
            break;
        case CornerCheckState::UNCONFIRMED:
            state = CornerCheckState::IDLE;
            break;
    }
    return state;
}

void CornerDetector::reset() {
    state = CornerCheckState::IDLE;
    confirmationCount = 0;
}
