#include "KalmanFilter.h"

double KalmanFilter::update(double U) {
    double K = P * H / (H * P * H + R);
    U_hat += K * (U - H * U_hat);
    P = (1 - K * H) * P + Q;
    return U_hat;
}