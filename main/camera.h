#pragma once

#include <esp_camera.h>

esp_err_t init_camera(void);
pixformat_t get_camera_format(void);
int get_camera_quality(void);
esp_err_t update_camera(pixformat_t pixformat, framesize_t framesize, int jpegquality);