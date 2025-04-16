#include <ArduinoWebsockets.h>
#include <HCSR04.h>
#include <INA219_WE.h>
#include <MPU6050.h>
#include <State.h>
#include <WiFi.h>
#include <Wire.h>

#include <cmath>

#include "Config.h"
#include "Globals.h"
#include "KalmanFilter.h"
#include "WheelControl.h"
#include "esp_log.h"

websockets::WebsocketsClient wsClient;

INA219_WE ina;
MPU6050 mpu;
WheelControl wheels;

State state;

void IRAM_ATTR hallSensorISR();
void loopCore0(void *pvParameters);
void loopCore1(void *pvParameters);

void connectToWifi();
void connectToServer();

void setup() {
    Serial.begin(USB_SPEED);
    wheels.attach(IN1, IN2, IN3, IN4);
    wheels.stop();

    pinMode(HALL_SENSOR, INPUT_PULLUP);

    Wire.begin();
    ESP_LOGD(mainLogTag, "Memory free: %d", ESP.getFreeHeap());

    if (!ina.init()) {
        ESP_LOGE(mainLogTag, "INA219 initialization failed!");
        return;
    }
    ESP_LOGI(mainLogTag, "INA219: Successful");
    connectToWifi();
    connectToServer();

    mpu.initialize();
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    ESP_LOGI(mainLogTag, "Calibrate MPU (20)");
    mpu.CalibrateGyro(20);
    mpu.CalibrateAccel(5);
    ESP_LOGI(mainLogTag, "MPU6050: Successful");

    byte *echoPins = new byte[2]{FRONT_ECHO, SIDE_ECHO};
    HCSR04.begin(TRIG, echoPins, 2);
    pinMode(HALL_SENSOR, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(HALL_SENSOR), hallSensorISR, FALLING);

    xTaskCreatePinnedToCore(loopCore0, "lc0", 8192, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(loopCore1, "yaw", 8192, NULL, 1, NULL, 1);
}

void onEventsCallback(websockets::WebsocketsEvent event, String data) {
    using websockets::WebsocketsEvent;
    if (event == WebsocketsEvent::ConnectionOpened) {
        ESP_LOGI(mainLogTag, "WebSocket connection opened");
    } else if (event == WebsocketsEvent::ConnectionClosed) {
        ESP_LOGW(mainLogTag, "WebSocket connection closed");
    } else if (event == WebsocketsEvent::GotPing) {
        ESP_LOGD(mainLogTag, "Got a Ping!");
    } else if (event == WebsocketsEvent::GotPong) {
        ESP_LOGD(mainLogTag, "Got a Pong!");
    }
}

void onMessageCallback(websockets::WebsocketsMessage message) {
    if (message.data() == "start") {
        ESP_LOGI(mainLogTag, "Machine started");
        state.handleMode = false;
        state.autoMode = true;
        state.yawReference = state.yaw;
        wheels.forward(true);
        
    } else if (message.data() == "stop") {
        ESP_LOGI(mainLogTag, "Machine stopped");
        state.handleMode = false;
        state.autoMode = false;
        wheels.stop();

    } else if (message.data() == "suspend") {
        ESP_LOGI(mainLogTag, "Machine suspended");
    } else if (message.data() == "resume") {
        ESP_LOGI(mainLogTag, "Machine resumed");
    } else if (message.data() == "remote_forward") {
        state.handleMode = true;
        wheels.forward(true);
    } else if (message.data() == "remote_backward") {
        state.handleMode = true;
        wheels.backward();
    } else if (message.data() == "remote_left") {
        state.handleMode = true;
        wheels.left();
    } else if (message.data() == "remote_right") {
        state.handleMode = true;
        wheels.right();
    } else if (message.data() == "remote_stop") {
        state.handleMode = true;
        wheels.stop();
    }
}

void connectToServer() {
    wsClient = websockets::WebsocketsClient();
    wsClient.onMessage(onMessageCallback);
    wsClient.onEvent(onEventsCallback);
    ESP_LOGD(mainLogTag, "Connect to server");
    while (!wsClient.connect(websocket_server)) {
        ESP_LOGW(mainLogTag, "Failed to connect to server, retrying...");
        vTaskDelay(5000);
    }

    ESP_LOGD(mainLogTag, "Connected to server");
    String mac = "mac:" + String(WiFi.macAddress());
    wsClient.send(mac);
}

void connectToWifi() {
    WiFi.begin(ssid, password);
    WiFi.setSleep(false);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
    }
    ESP_LOGI(mainLogTag, "Connected to WiFi");
}

void IRAM_ATTR hallSensorISR() {
    static double tInterrupt = 0.0;
    ESP_LOGI(mainLogTag, "Interrupt by Hall");

    if (wheels.direction == Direction::LEFT or
        wheels.direction == Direction::RIGHT) {
        state.previousDirection = wheels.direction;
        return;
    }

    if (state.t - tInterrupt <= 1.0) return;
    tInterrupt = state.t;
    // The robot's going straight
    if (state.previousDirection == Direction::LEFT or
        state.previousDirection == Direction::RIGHT) {
        state.previousDirection = wheels.direction;
        return;
    }

    if (wheels.direction == Direction::FORWARD)
        state.distanceHall += WHEEL_DISTANCE_MM;
    else if (wheels.direction == Direction::BACKWARD)
        state.distanceHall -= WHEEL_DISTANCE_MM;
}

void loopCore0(void *pvParameters) {
    long previousTime = 0;
    while (true) {
        long currentTime = millis();
        if (wsClient.available()) {
            wsClient.poll();
            if (state.distanceHall != 0 or millis() - previousTime > 1000) {
                String data = state.getStr();
                wsClient.send(data);
                previousTime = millis();
                state.distanceHall = 0;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelete(NULL);
}

void updateDistances() {
    static KalmanFilter kalmanFront;
    static KalmanFilter kalmanSide;
    static float distanceTime = 0.0;
    distanceTime += state.dt;
    if (distanceTime <= 0.1f) return;

    distanceTime = 0.0f;
    double *distances = HCSR04.measureDistanceMm();
    if (distances[0] != -1)
        state.distanceFront = kalmanFront.update(distances[0]);
    else
        state.distanceFront = distances[0];
    if (distances[1] != -1)
        state.distanceSide = kalmanSide.update(distances[1]);
    else
        state.distanceSide = distances[1];

    return;
}

void handleCorrection() {
    static const bool enableSmooth = false;
    static float correctionTime = 0.0;
    if (state.correction == Correction::TO_LEFT)
        wheels.correction(false);
    else if (state.correction == Correction::TO_RIGHT)
        wheels.correction(true);
    else if (state.correction == Correction::IN_PROGRESS) {
        correctionTime += state.dt;
        if (correctionTime <= 0.3f) return;
        wheels.forward(enableSmooth);
        state.correction = Correction::NO;
        correctionTime = 0.0f;
    }
    return;
}

void step() {
    double now = micros() / 1000000.0;
    state.dt = static_cast<float>(now - state.t);
    state.t = now;

    if (!(state.dt > 0)) {
        state.dt = 0;
    }
}

void calibrateGyroOnce() {
    static float stopTime = 0.0f;
    stopTime = wheels.direction == Direction::STOP ? stopTime + state.dt : 0.0f;
    if (stopTime < 2.0f) return;

    mpu.CalibrateGyro(1);
}

void updateYaw() {
    static int16_t ax, ay, az, gx, gy, gz;
    static double yaw = 0.0;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    yaw += static_cast<double>(gz) * state.dt / GYRO_SENSITIVITY;
    state.yaw = fmod(yaw, 360.0);
    if (state.yaw < 0) state.yaw += 360.0;

    calibrateGyroOnce();

    if (state.handleMode || state.correction != Correction::IN_PROGRESS ||
        wheels.direction != Direction::FORWARD ||
        fabs(state.yawReference - state.yaw) <= 2.0)
        return;

    state.correction = state.yawReference < state.yaw ? Correction::TO_RIGHT
                                                      : Correction::TO_LEFT;
}

void loopCore1(void *pvParameters) {
    while (true) {
        step();
        updateYaw();
        handleCorrection();
        updateDistances();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void loop() { vTaskDelay(portMAX_DELAY); }