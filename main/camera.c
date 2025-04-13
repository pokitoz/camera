#include "camera_local.h"

#include <esp_camera.h>
#include <esp_log.h>

static const char *LOG_CAMERA_TAG = "picture:camera";

static camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SIOD,
    .pin_sccb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    // XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    // YUV422,GRAYSCALE,RGB565,JPEG
    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_HVGA,

    // 0-63, for OV series camera sensors, lower number means higher quality
    .jpeg_quality = 15,
    // When jpeg mode is used, if fb_count more than one, the driver will work in continuous mode.
    .fb_count = 1,
    // CAMERA_FB_IN_DRAM CAMERA_FB_IN_PSRAM
    .fb_location = CAMERA_FB_IN_DRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

esp_err_t init_camera(void)
{
    esp_err_t err = esp_camera_init(&camera_config);

    if (err != ESP_OK)
    {
        ESP_LOGE(LOG_CAMERA_TAG, "on init");
    }
    return err;
}

pixformat_t get_camera_format(void)
{
    return camera_config.pixel_format;
}

int get_camera_quality(void)
{
    return camera_config.jpeg_quality;
}

esp_err_t update_camera(pixformat_t pixformat, framesize_t framesize, int jpegquality)
{
    esp_err_t ret = esp_camera_deinit();

    if (ret != ESP_OK)
    {
        ESP_LOGE(LOG_CAMERA_TAG, "on deinit");
    }
    else
    {
        camera_config.pixel_format = pixformat,
        camera_config.frame_size = framesize,
        camera_config.jpeg_quality = jpegquality,
    
        // Re init with the updated parameters
        ret = init_camera();    
    }

    return ret;
}
