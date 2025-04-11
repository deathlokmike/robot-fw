#pragma once
#define USB_SPEED 115200

#define CAM_PCLK 33
#define CAM_XCLK 32

#define CAM_D7 2
#define CAM_D6 12
#define CAM_D5 13
#define CAM_D4 14
#define CAM_D3 15
#define CAM_D2 16
#define CAM_D1 17
#define CAM_D0 27

#define CAM_VSYNC 34
#define CAM_HREF 35

#define HALL_SENSOR 26

#define FRONT_ECHO 25
#define SIDE_ECHO 39

#define IN1 4
#define IN2 18  // always openned
#define IN3 5
#define IN4 19

#define TRIG 23

#define WHEEL_DISTANCE_MM 179.07
#define GYRO_SENSITIVITY 131.0 

const char *ssid = "...";
const char *password = "...";
const char *websocket_server = "ws://192.168.1.163:8000/ws/cleaner";
const String endpoint = "http://192.168.1.163:8000/api/image";