#pragma once

#include <esp_camera.h>

esp_err_t init_camera(void);
pixformat_t get_camera_format(void);