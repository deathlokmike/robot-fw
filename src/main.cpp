#include <ArduinoWebsockets.h>
#include <HCSR04.h>
#include <INA219_WE.h>
#include <MPU6050.h>
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

volatile unsigned long lastInterruptTime = 0;
volatile uint8_t lastDirection = -1;
volatile bool isCorrecting = false;

volatile State state;

void IRAM_ATTR hallSensorISR();
void wsPolling(void *pvParameters);
void setYawTask(void *pvParameters);
void setDistanceTask(void *pvParameters);
void wallFollowTask(void *pvParameters);
void correctionTask(void *pvParameters);

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
    mpu.CalibrateAccel(20);
    ESP_LOGI(mainLogTag, "MPU6050: Successful");

    byte *echoPins = new byte[2]{FRONT_ECHO, SIDE_ECHO};
    HCSR04.begin(TRIG, echoPins, 2);
    attachInterrupt(digitalPinToInterrupt(HALL_SENSOR), hallSensorISR, FALLING);
    xCorrectDirectionQueue = xQueueCreate(5, sizeof(bool));

    xTaskCreatePinnedToCore(wsPolling, "ws", 8192, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(setYawTask, "yaw", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(correctionTask, "corr", 2048, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(setDistanceTask, "dt", 8192, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(wallFollowTask, "movement", 4096, NULL, 1,
                            &wallFollowTaskHandle, 0);

    if (wallFollowTaskHandle == NULL) {
        ESP_LOGW(mainLogTag, "movement task: failed to create");
    } else {
        vTaskSuspend(wallFollowTaskHandle);
        ESP_LOGD(mainLogTag, "Memory free: %d", ESP.getFreeHeap());
    }
}

String getSensorData() {
    float busVoltage, current_mA;
    busVoltage = ina.getBusVoltage_V();
    current_mA = ina.getCurrent_mA();

    return "vol:" + String(busVoltage) + ",cur:" + String(current_mA) +
           ",df:" + String(state.distanceFront) +
           ",ds:" + String(state.distanceSide) + ",ang:" + String(state.angle) +
           ",ts:" + String(millis()) + ",dh:" + String(state.distanceHall);
}

void wsPolling(void *pvParameters) {
    long previousTime = 0;
    while (true) {
        long currentTime = millis();
        if (wsClient.available()) {
            wsClient.poll();
            if (state.distanceHall != 0 or millis() - previousTime > 1000) {
                String data = getSensorData();
                wsClient.send(data);
                previousTime = millis();
                state.distanceHall = 0;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelete(NULL);
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
        if (wallFollowTaskHandle != NULL) {
            state.referenceAngle = state.angle;
            wheels.forward(true);
            // vTaskResume(wallFollowTaskHandle);
        }
    } else if (message.data() == "stop") {
        ESP_LOGI(mainLogTag, "Machine stopped");
        if (wallFollowTaskHandle != NULL) {
            // vTaskSuspend(wallFollowTaskHandle);
            wheels.stop();
        }
    } else if (message.data() == "suspend") {
        ESP_LOGI(mainLogTag, "Machine suspended");
    } else if (message.data() == "resume") {
        ESP_LOGI(mainLogTag, "Machine resumed");
    } else if (message.data() == "remote_forward") {
        wheels.forward(true);
    } else if (message.data() == "remote_backward") {
        wheels.backward();
    } else if (message.data() == "remote_left") {
        wheels.left();
    } else if (message.data() == "remote_right") {
        wheels.right();
    } else if (message.data() == "remote_stop") {
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
    lastDirection = wheels.direction;
    if (wheels.direction == wheelDirection::LEFT or
        wheels.direction == wheelDirection::RIGHT)
        return;

    unsigned long currentTime = millis();
    if (currentTime - lastInterruptTime <= 1000) return;
    lastInterruptTime = currentTime;
    if (lastDirection == wheelDirection::LEFT or
        lastDirection == wheelDirection::RIGHT)
        return;
    if (wheels.direction == wheelDirection::FORWARD)
        state.distanceHall += WHEEL_DISTANCE_MM;
    else if (wheels.direction == wheelDirection::BACKWARD)
        state.distanceHall -= WHEEL_DISTANCE_MM;
}

void setYawTask(void *pvParameters) {
    long previousTime = millis();
    double yaw = 0.0;
    double correctedYaw = 0.0;
    const double second = 1000.0;
    int16_t ax, ay, az, gx, gy, gz;

    while (true) {
        long currentTime = millis();
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        double dt = (currentTime - previousTime) / second;
        previousTime = currentTime;

        yaw += static_cast<double>(gz) * dt / GYRO_SENSITIVITY;
        correctedYaw = fmod(yaw, 360.0);
        if (correctedYaw < 0) correctedYaw += 360.0;

        state.angle = correctedYaw;
        if (!isCorrecting and wheels.direction == wheelDirection::FORWARD and
            fabs(state.referenceAngle - correctedYaw) > 3.0) {
            bool toRight = (state.referenceAngle < correctedYaw);
            xQueueSend(xCorrectDirectionQueue, &toRight, 0);
            isCorrecting = true;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    vTaskDelete(NULL);
}

void correctionTask(void *pvParameters) {
    bool direction;
    while (true) {
        if (xQueueReceive(xCorrectDirectionQueue, &direction, portMAX_DELAY) ==
            pdTRUE) {
            ESP_LOGD(mainLogTag, "Correction");
            wheels.correction(direction);
            vTaskDelay(pdMS_TO_TICKS(10));
            wheels.forward(false);
            isCorrecting = false;
        }
    }
}

void setDistanceTask(void *pvParameters) {
    KalmanFilter kalmanFront;
    KalmanFilter kalmanSide;
    while (true) {
        double *distances = HCSR04.measureDistanceMm();
        if (distances[0] != -1)
            state.distanceFront = kalmanFront.update(distances[0]);
        else
            state.distanceFront = distances[0];
        if (distances[1] != -1)
            state.distanceSide = kalmanSide.update(distances[1]);
        else
            state.distanceSide = distances[1];

        vTaskDelay(pdMS_TO_TICKS(100));
    }
    vTaskDelete(NULL);
}

double getStableDistance(const volatile double *distancePtr) {
    double sum = 0;
    double minVal = 1e9, maxVal = 0;
    uint8_t num_samples = 3;

    for (uint8_t i = 0; i < num_samples; i++) {
        double d = *distancePtr;
        sum += d;
        if (d < minVal) minVal = d;
        if (d > maxVal) maxVal = d;
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    if ((maxVal - minVal) > 10.0) {
        return -1;
    }

    return sum / num_samples;
}
void wallFollowTask(void *pvParameters) { vTaskDelay(portMAX_DELAY); }
void loop() { vTaskDelay(portMAX_DELAY); }