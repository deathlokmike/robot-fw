#pragma once
#define CAMERA_ENABLED 0

#if CAMERA_ENABLED
void setupCamera();
void takeImageAndSendPostRequest();
#endif
