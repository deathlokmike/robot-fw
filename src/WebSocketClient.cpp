#include "WebSocketClient.h"

#include <ArduinoWebsockets.h>

#include "Navo.h"
#include "esp_log.h"

websockets::WebsocketsClient wsClient;

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
        navo.autoMode = AutoMode::ENABLE;
    } else if (message.data() == "stop") {
        navo.autoMode = AutoMode::DISABLE;
    } else if (message.data() == "suspend") {
        navo.autoMode = AutoMode::SUSPEND;
    } else if (message.data() == "resume") {
        navo.autoMode = AutoMode::ACTIVE;
    } else if (message.data() == "remote_forward") {
        navo.autoMode = AutoMode::MANUAL;
        navo.wheels.forward(true, false);
    } else if (message.data() == "remote_backward") {
        navo.autoMode = AutoMode::MANUAL;
        navo.wheels.backward();
    } else if (message.data() == "remote_left") {
        navo.autoMode = AutoMode::MANUAL;
        navo.wheels.left();
    } else if (message.data() == "remote_right") {
        navo.autoMode = AutoMode::MANUAL;
        navo.wheels.right();
    } else if (message.data() == "remote_stop") {
        navo.wheels.stop(true);
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

void pollAndSendData() {
    static float sendTime = 0.0;
    if (wsClient.available()) {
        wsClient.poll();
        sendTime += navo.dt_loop0;

        bool sendNow = false;

        if (navo.hallState == HallState::HALL_USED) {
            sendNow = true;
        } else if (sendTime >= 1.0f) {
            sendNow = true;
        }

        if (sendNow) {
            String data = navo.getStr();
            wsClient.send(data);

            if (navo.hallState == HallState::HALL_USED) {
                navo.distanceHall = 0;
                navo.hallState = HallState::HALL_RESET;
            }

            sendTime = 0.0f;
        }

    } else {
        ESP_LOGW(mainLogTag, "WS client is not available");
    }
}