#include "Sensors.h"

#include <HCSR04.h>
#include <INA219_WE.h>
#include <MPU6050.h>

#include "KalmanFilter.h"
#include "LowPassFilter.h"
#include "Navo.h"

INA219_WE ina;
MPU6050 imu;

int16_t gyroZ;
int16_t gyroZBias = 0;

void IRAM_ATTR hallSensorISR() {
    static double tInterrupt = 0.0;
    ESP_LOGI(mainLogTag, "Interrupt by Hall");

    if (navo.wheels.direction == Direction::LEFT or
        navo.wheels.direction == Direction::RIGHT) {
        navo.previousDirection = navo.wheels.direction;
        return;
    }

    if (navo.t_loop1 - tInterrupt <= 1.0) return;
    tInterrupt = navo.t_loop1;
    // The robot's going straight
    if (navo.previousDirection == Direction::LEFT or
        navo.previousDirection == Direction::RIGHT) {
        navo.previousDirection = navo.wheels.direction;
        return;
    }

    if (navo.wheels.direction == Direction::FORWARD)
        navo.distanceHall += WHEEL_DISTANCE_MM;
    else if (navo.wheels.direction == Direction::BACKWARD)
        navo.distanceHall -= WHEEL_DISTANCE_MM;
    navo.hallState = HallState::HALL_TRIGGERED;
}

void setupPowerSensor() {
    if (!ina.init()) {
        ESP_LOGE(mainLogTag, "INA219 initialization failed!");
    }
    ESP_LOGI(mainLogTag, "INA219: Successful");
}

void setupIMU() {
    imu.initialize();
    imu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    ESP_LOGI(mainLogTag, "Calibrate MPU");
    imu.CalibrateGyro(20);
    imu.CalibrateAccel(5);
    ESP_LOGI(mainLogTag, "MPU6050: Successful");
}

void setupUltrasonicSensors() {
    byte *echoPins = new byte[2]{FRONT_ECHO, SIDE_ECHO};
    HCSR04.begin(TRIG, echoPins, 2);
    pinMode(HALL_SENSOR, INPUT_PULLUP);
}

void setupHallSensor() {
    pinMode(HALL_SENSOR, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(HALL_SENSOR), hallSensorISR, FALLING);
}

void updateVoltageAndCurrent() {
    navo.voltage = ina.getBusVoltage_V();
    navo.current = ina.getCurrent_mA();
}

void updateDistances() {
    static KalmanFilter kalmanFront;
    static KalmanFilter kalmanSide;
    static float distanceTime = 0.0;
    distanceTime += navo.dt_loop1;
    if (distanceTime <= 0.1f) return;

    distanceTime = 0.0f;
    double *distances = HCSR04.measureDistanceMm();
    if (distances[0] != -1)
        navo.distanceFront = kalmanFront.update(distances[0]);
    else
        navo.distanceFront = distances[0];
    if (distances[1] != -1)
        navo.distanceSide = kalmanSide.update(distances[1]);
    else
        navo.distanceSide = distances[1];

    return;
}

// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix
void calibrateGyroOnce() {
    static float stopTime = 0.0f;
    stopTime = navo.wheels.direction == Direction::STOP
                   ? stopTime + navo.dt_loop1
                   : 0.0f;
    if (stopTime < 2.0f) return;
    ESP_LOGV(mainLogTag, "Callibrate gyro");
    static LowPassFilter<int16_t> gyroZBiasFilter(0.01);
    gyroZBias = gyroZBiasFilter.update(gyroZ);
}

void updateYaw() {
    static double yaw = 0.0;
    static double tmp = 0.0;
    gyroZ = imu.getRotationZ() - gyroZBias;

    yaw += static_cast<double>(gyroZ) * navo.dt_loop1 / GYRO_SENSITIVITY;
    tmp = fmod(yaw, 360.0);
    navo.yaw = tmp < 0 ? tmp += 360.0 : tmp;
    calibrateGyroOnce();
}