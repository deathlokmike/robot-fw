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

SemaphoreHandle_t xCorrectSemaphore;
QueueHandle_t xCorrectDirectionQueue;
TaskHandle_t wallFollowTaskHandle;

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
        if (wallFollowTaskHandle != NULL) {
            state.referenceAngle = state.angle;
            wheels.forward(true);
            // vTaskResume(wallFollowTaskHandle);
        }
    } else if (message.data() == "stop") {
        ESP_LOGI(mainLogTag, "Machine stopped");
        state.handleMode = false;
        state.autoMode = false;
        if (wallFollowTaskHandle != NULL) {
            // vTaskSuspend(wallFollowTaskHandle);
            wheels.stop();
        }
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
    ESP_LOGI(mainLogTag, "Interrupt by Hall");
    state.direction = wheels.direction;
    if (wheels.direction == Direction::LEFT or
        wheels.direction == Direction::RIGHT)
        return;

    unsigned long currentTime = millis();
    if (currentTime - lastInterruptTime <= 1000) return;
    lastInterruptTime = currentTime;
    if (lastDirection == Direction::LEFT or
        lastDirection == Direction::RIGHT)
        return;
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

void handleCorrection() {
    bool toRight;
    const bool enableSmooth = false;

    if (xQueueReceive(xCorrectDirectionQueue, &toRight, 0) == pdTRUE) {
        ESP_LOGD(mainLogTag, "Correction to right=%s",
                 toRight ? "true" : "false");
        wheels.correction(toRight);
        vTaskDelay(pdMS_TO_TICKS(300));
        wheels.forward(enableSmooth);
        isCorrecting = false;
    }
}

void step() {
    double now = micros() / 1000000.0;
    state.dt = now - state.t;
    state.t = now;

    if (!(state.dt > 0)) {
        state.dt = 0;
    }
}

void loopCore1(void *pvParameters) {
    while (true) {
        step();
        updateYaw();
        handleCorrection();
        updateDistances();
        computeLoopRate();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void loop() { vTaskDelay(portMAX_DELAY); }