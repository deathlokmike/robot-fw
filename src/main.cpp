#include <ArduinoWebsockets.h>
#include <HCSR04.h>
#include <INA219_WE.h>
#include <MPU6050.h>
#include <WiFi.h>
#include <Wire.h>

#include "Config.h"
#include "Globals.h"
#include "OV7670.h"
#include "WheelControl.h"
#include "esp_log.h"

websockets::WebsocketsClient wsClient;
HTTPClient client;

INA219_WE ina;
MPU6050 mpu;
OV7670 camera;
WheelControl wheels;

TaskHandle_t movementTaskHandle = NULL;
TaskHandle_t cameraTaskHandler = NULL;

int16_t ax, ay, az, gx, gy, gz;
float currentAngle = 0;

void takeImageTask(void *pvParameters);
void sensorTask(void *pvParameters);
void movementTask(void *pvParameters);

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
        if (movementTaskHandle != NULL) {
            vTaskResume(cameraTaskHandler);
            vTaskResume(movementTaskHandle);
        }
    } else if (message.data() == "stop") {
        ESP_LOGI(mainLogTag, "Machine stopped");
        if (movementTaskHandle != NULL) {
            vTaskSuspend(movementTaskHandle);
            vTaskSuspend(cameraTaskHandler);
            wheels.stop();
        }
    } else if (message.data() == "suspend") {
        ESP_LOGI(mainLogTag, "Machine suspended");
    } else if (message.data() == "resume") {
        ESP_LOGI(mainLogTag, "Machine resumed");
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

void setup() {
    Serial.begin(USB_SPEED);
    wheels.attach(IN1, IN2, IN3, IN4);
    wheels.stop();

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
    if (!camera.init(CAM_VSYNC, CAM_HREF, CAM_XCLK, CAM_PCLK, CAM_D0, CAM_D1,
                     CAM_D2, CAM_D3, CAM_D4, CAM_D5, CAM_D6, CAM_D7)) {
        ESP_LOGE(mainLogTag, "Camera initialization failed!");
        return;
    }
    ESP_LOGI(mainLogTag, "Camera: Successful");
    byte *echoPins = new byte[2]{FRONT_ECHO, SIDE_ECHO};
    HCSR04.begin(TRIG, echoPins, 2);

    xTaskCreatePinnedToCore(sensorTask, "sensor", 8192, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(movementTask, "movement", 4096, NULL, 1,
                            &movementTaskHandle, 1);
    xTaskCreatePinnedToCore(takeImageTask, "camera", 98304, NULL, 1,
                            &cameraTaskHandler, 0);
    if (movementTaskHandle == NULL) {
        ESP_LOGW(mainLogTag, "movement task: failed to create");
    } else if (cameraTaskHandler == NULL) {
        ESP_LOGW(mainLogTag, "camera task: failed to create");
    } else {
        vTaskSuspend(cameraTaskHandler);
        vTaskSuspend(movementTaskHandle);
        ESP_LOGD(mainLogTag, "Memory free: %d", ESP.getFreeHeap());
    }
}

void takeImageAndSendPostRequest() {
    camera.oneFrame();
    client.begin(endpoint);
    int httpResponseCode =
        client.sendRequest("POST", camera.frame, camera.frameBytes);
    if (httpResponseCode > 0) {
        String payload = client.getString();
        ESP_LOGI(mainLogTag, "HTTP: [%d] %s", httpResponseCode, payload);
    } else {
        ESP_LOGW(mainLogTag, "HTTP-response error");
    }
    client.end();
}

void takeImageTask(void *pvParameters) {
    ESP_LOGD(mainLogTag, "[TIT]: started");
    while (true) {
        ESP_LOGD(mainLogTag, "[TIT]: iter");
        takeImageAndSendPostRequest();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    vTaskDelete(NULL);
}

void sensorTask(void *pvParameters) {
    ESP_LOGD(mainLogTag, "[sensorTask]: started");
    float busVoltage, current_mA;
    uint8_t poolLoop = 0;

    String data;
    while (true) {
        busVoltage = ina.getBusVoltage_V();
        current_mA = ina.getCurrent_mA();
        double *distances = HCSR04.measureDistanceMm();

        data = "vol:" + String(busVoltage) + ",cur:" + String(current_mA) +
               ",ang:" + String(currentAngle) + ",df:" + String(distances[0]) +
               ",ds:" + String(distances[1]);

        poolLoop += 1;
        if (wsClient.available()) {
            if (poolLoop == 4) {
                poolLoop = 0;
                wsClient.poll();
            } else {
                wsClient.send(data);
            }
        } else {
            ESP_LOGD(mainLogTag, "Wi-fi status: %d", WiFi.status());
            if (WiFi.status() != WL_CONNECTED) {
                connectToWifi();
            }
            wsClient.close();
            connectToServer();
        }
        vTaskDelay(pdMS_TO_TICKS(250));
    }
    vTaskDelete(NULL);
}

void rotate(float angle) {
    // Alpha - beta filter
    static float alpha = 0.9;
    static float beta = 0.0025;
    float estimatedRate = 0;
    float dt = 0;
    float gyroZ = 0;
    bool once = true;
    long previousTime = 0;
    currentAngle = 0;
    while (true) {
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        if (once) {
            wheels.left();
            once = false;
        }

        gyroZ = gz / 131;
        long currentTime = millis();
        dt = (currentTime - previousTime) / 1000.0;
        previousTime = currentTime;

        float predictedAngle = currentAngle + estimatedRate * dt;
        float predictedRate = estimatedRate;

        float residual = gyroZ - predictedRate;
        currentAngle = predictedAngle + alpha * residual * dt;
        estimatedRate = predictedRate + beta * residual;
        ESP_LOGD(mainLogTag, "currentAngle: %f", currentAngle);
        if (currentAngle >= angle) {
            wheels.stop();
            return;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void movementTask(void *pvParameters) {
    while (true) {
        wheels.forward();
        vTaskDelay(pdMS_TO_TICKS(2000));
        wheels.stop();
        vTaskDelay(pdMS_TO_TICKS(1000));
        rotate(69.5);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    vTaskDelete(NULL);
}

void loop() { vTaskDelay(portMAX_DELAY); }