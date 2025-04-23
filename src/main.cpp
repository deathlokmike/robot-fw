#include <Wire.h>

#include "CameraHandler.h"
#include "MainLoops.h"
#include "Navo.h"
#include "Sensors.h"
#include "WebSocketClient.h"
#include "WiFiManager.h"

Navo navo;

void setup() {
    Serial.begin(USB_SPEED);
    Wire.begin();
    connectToWiFi();
    connectToServer();
    setupHallSensor();
    setupIMU();
    setupPowerSensor();
    setupUltrasonicSensors();

#if CAMERA_ENABLED
    setupCamera();
#endif

    startMainLoops();
}

void loop() { vTaskDelay(portMAX_DELAY); }