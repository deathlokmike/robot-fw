#define CAMERA_ENABLED 0

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
#include "OV7670.h"
#include "WheelControl.h"
#include "esp_log.h"

websockets::WebsocketsClient wsClient;

#if CAMERA_ENABLED
HTTPClient client;
OV7670 camera;
#endif

INA219_WE ina;
MPU6050 imu;
WheelControl wheels;

Navo navo;

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

    imu.initialize();
    imu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    ESP_LOGI(mainLogTag, "Calibrate MPU");
    imu.CalibrateGyro(20);
    imu.CalibrateAccel(5);
    ESP_LOGI(mainLogTag, "MPU6050: Successful");

    byte *echoPins = new byte[2]{FRONT_ECHO, SIDE_ECHO};
    HCSR04.begin(TRIG, echoPins, 2);
    pinMode(HALL_SENSOR, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(HALL_SENSOR), hallSensorISR, FALLING);

#if CAMERA_ENABLED
    if (gpio_install_isr_service(ESP_INTR_FLAG_IRAM) != ESP_OK) {
        ESP_LOGE(mainLogTag, "Failed to install ISR service");
    }
    if (!camera.init(CAM_VSYNC, CAM_HREF, CAM_XCLK, CAM_PCLK, CAM_D0, CAM_D1,
                     CAM_D2, CAM_D3, CAM_D4, CAM_D5, CAM_D6, CAM_D7)) {
        ESP_LOGE(mainLogTag, "Camera initialization failed!");
        return;
    }
    ESP_LOGI(mainLogTag, "Camera: Successful");
#endif

    xTaskCreatePinnedToCore(loopCore0, "lc0", 8192, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(loopCore1, "lc1", 8192, NULL, 1, NULL, 1);
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
        navo.autoMode = AutoMode::ENABLE;
    } else if (message.data() == "stop") {
        ESP_LOGI(mainLogTag, "Machine stopped");
        navo.autoMode = AutoMode::DISABLE;
    } else if (message.data() == "suspend") {
        ESP_LOGI(mainLogTag, "Machine suspended");
        navo.autoMode = AutoMode::SUSPEND;
    } else if (message.data() == "resume") {
        ESP_LOGI(mainLogTag, "Machine resumed");
        navo.autoMode = AutoMode::ACTIVE;
    } else if (message.data() == "remote_forward") {
        navo.autoMode = AutoMode::MANUAL;
        wheels.forward(true);
    } else if (message.data() == "remote_backward") {
        navo.autoMode = AutoMode::MANUAL;
        wheels.backward();
    } else if (message.data() == "remote_left") {
        navo.autoMode = AutoMode::MANUAL;
        wheels.left();
    } else if (message.data() == "remote_right") {
        navo.autoMode = AutoMode::MANUAL;
        wheels.right();
    } else if (message.data() == "remote_stop") {
        navo.autoMode = AutoMode::MANUAL;
        wheels.stop();
        navo.autoMode = AutoMode::DISABLE;
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
        navo.previousDirection = wheels.direction;
        return;
    }

    if (navo.t_loop1 - tInterrupt <= 1.0) return;
    tInterrupt = navo.t_loop1;
    // The robot's going straight
    if (navo.previousDirection == Direction::LEFT or
        navo.previousDirection == Direction::RIGHT) {
        navo.previousDirection = wheels.direction;
        return;
    }

    if (wheels.direction == Direction::FORWARD)
        navo.distanceHall += WHEEL_DISTANCE_MM;
    else if (wheels.direction == Direction::BACKWARD)
        navo.distanceHall -= WHEEL_DISTANCE_MM;
}

void stepLoop0() {
    double now = micros() / 1000000.0;
    navo.dt_loop0 = static_cast<float>(now - navo.t_loop0);
    navo.t_loop0 = now;

    if (!(navo.dt_loop0 > 0)) {
        navo.dt_loop0 = 0;
    }
}

void pollAndSendData() {
    static float sendTime = 0.0;
    if (wsClient.available()) {
        wsClient.poll();
        sendTime += navo.dt_loop0;
        if (sendTime <= 1.0f) return;

        String data = navo.getStr();
        wsClient.send(data);
        navo.distanceHall = 0;
        sendTime = 0.0f;
    } else {
        ESP_LOGW(mainLogTag, "WS client is not available");
    }
}

#if CAMERA_ENABLED
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
#endif

void loopCore0(void *pvParameters) {
    while (true) {
        stepLoop0();
        pollAndSendData();
#if CAMERA_ENABLED
        takeImageAndSendPostRequest();
#endif
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    vTaskDelete(NULL);
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

void doCorrection() {
    static const bool enableSmooth = false;
    static float correctionTime = 0.0;
    if (navo.correction == Correction::NO)
        return;
    else if (navo.correction == Correction::TO_LEFT) {
        wheels.correction(false);
        navo.correction = Correction::IN_PROGRESS;
    } else if (navo.correction == Correction::TO_RIGHT) {
        wheels.correction(true);
        navo.correction = Correction::IN_PROGRESS;
    } else {
        ESP_LOGD(mainLogTag, "correction time: %f", correctionTime);
        correctionTime += navo.dt_loop1;
        if (correctionTime <= 0.2f) return;
        wheels.forward(enableSmooth);
        navo.correction = Correction::NO;
        correctionTime = 0.0f;
    }
}

void stepLoop1() {
    double now = micros() / 1000000.0;
    navo.dt_loop1 = static_cast<float>(now - navo.t_loop1);
    navo.t_loop1 = now;

    if (!(navo.dt_loop1 > 0)) {
        navo.dt_loop1 = 0;
    }
}

void calibrateGyroOnce() {
    static float stopTime = 0.0f;
    stopTime =
        wheels.direction == Direction::STOP ? stopTime + navo.dt_loop1 : 0.0f;
    if (stopTime < 2.0f) return;
    ESP_LOGD(mainLogTag, "Callibrate gyro");
    imu.CalibrateGyro(1);
}

void updateYaw() {
    static int16_t gz;
    static double yaw = 0.0;
    static double tmp = 0.0;
    gz = imu.getRotationZ();

    yaw += static_cast<double>(gz) * navo.dt_loop1 / GYRO_SENSITIVITY;
    tmp = fmod(yaw, 360.0);
    navo.yaw = tmp < 0 ? tmp += 360.0 : tmp;
    ESP_LOGD(mainLogTag, "yaw: %f, tmp: %f", navo.yaw, tmp);
    // calibrateGyroOnce();

    if (navo.autoMode != AutoMode::ACTIVE ||
        navo.correction == Correction::IN_PROGRESS ||
        wheels.direction != Direction::FORWARD ||
        fabs(navo.yawReference - navo.yaw) <= 1.5)
        return;
    ESP_LOGI(mainLogTag, "Start correction, diff: %f",
             navo.yawReference - navo.yaw);
    navo.correction = navo.yawReference < navo.yaw ? Correction::TO_RIGHT
                                                   : Correction::TO_LEFT;
}

void updateVoltageAndCurrent() {
    navo.voltage = ina.getBusVoltage_V();
    navo.current = ina.getCurrent_mA();
}

void autoMode() {
    if (navo.autoMode == AutoMode::ENABLE) {
        wheels.forward(true);
        navo.yawReference = navo.yaw;
        navo.autoMode = AutoMode::ACTIVE;
    } else if (navo.autoMode == AutoMode::DISABLE or
               navo.autoMode == AutoMode::SUSPEND) {
        wheels.stop();
    }
    if (navo.autoMode == AutoMode::ACTIVE) {
        doCorrection();
    }
}

void loopCore1(void *pvParameters) {
    while (true) {
        stepLoop1();
        updateYaw();
        updateDistances();
        autoMode();
        updateVoltageAndCurrent();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    vTaskDelete(NULL);
}

void loop() { vTaskDelay(portMAX_DELAY); }