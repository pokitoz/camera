/**
 * This example takes a picture every 5s and print its size on serial monitor.
 */

// =============================== SETUP ======================================

// 1. Board setup (Uncomment):
// #define BOARD_WROVER_KIT
// #define BOARD_ESP32CAM_AITHINKER
// #define BOARD_ESP32S3_WROOM

/**
 * 2. Kconfig setup
 *
 * If you have a Kconfig file, copy the content from
 *  https://github.com/espressif/esp32-camera/blob/master/Kconfig into it.
 * In case you haven't, copy and paste this Kconfig file inside the src directory.
 * This Kconfig file has definitions that allows more control over the camera and
 * how it will be initialized.
 */

/**
 * 3. Enable PSRAM on sdkconfig:
 *
 * CONFIG_ESP32_SPIRAM_SUPPORT=y
 *
 * More info on
 * https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/kconfig.html#config-esp32-spiram-support
 */

// ================================ CODE ======================================

#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "connectwifi.h"


// support IDF 5.x
#ifndef portTICK_RATE_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif

#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_timer.h"


#define BOARD_ESP32CAM_AITHINKER 1

#define CAM_PIN_PWDN 32
#define CAM_PIN_RESET -1 //software reset will be performed
#define CAM_PIN_XCLK 0
#define CAM_PIN_SIOD 26
#define CAM_PIN_SIOC 27

#define CAM_PIN_D7 35
#define CAM_PIN_D6 34
#define CAM_PIN_D5 39
#define CAM_PIN_D4 36
#define CAM_PIN_D3 21
#define CAM_PIN_D2 19
#define CAM_PIN_D1 18
#define CAM_PIN_D0 5
#define CAM_PIN_VSYNC 25
#define CAM_PIN_HREF 23
#define CAM_PIN_PCLK 22

static const char *TAG = "example:take_picture";

#if ESP_CAMERA_SUPPORTED
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

        //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
        .xclk_freq_hz = 20000000,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,

        // YUV422,GRAYSCALE,RGB565,JPEG
        .pixel_format = PIXFORMAT_JPEG,
        // QQVGA-UXGA, For ESP32, do not use sizes above QVGA when not JPEG.
        // The performance of the ESP32-S series has improved a lot, but JPEG mode always gives better frame rates.
        .frame_size = FRAMESIZE_QVGA,

        //0-63, for OV series camera sensors, lower number means higher quality
        .jpeg_quality = 12,
        //When jpeg mode is used, if fb_count more than one, the driver will work in continuous mode.
        .fb_count = 1,
        // CAMERA_FB_IN_DRAM CAMERA_FB_IN_PSRAM
        .fb_location = CAMERA_FB_IN_DRAM,
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
    };

    static esp_err_t init_camera(void)
    {
        //initialize the camera
        esp_err_t err = esp_camera_init(&camera_config);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Camera Init Failed");
            return err;
        }

        return ESP_OK;
    }
#endif



esp_err_t jpg_stream_httpd_handler(httpd_req_t *req){

    #define PART_BOUNDARY "123456789000000000000987654321"
    static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
    static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
    const size_t _STREAM_BOUNDARY_SIZE = strlen(_STREAM_BOUNDARY);
    static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %lu\r\n\r\n";

    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len;
    uint8_t * _jpg_buf;
    char part_buf[64];
    int64_t last_frame = 0;
    size_t hlen = 0;

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if(res != ESP_OK){
        return res;
    }

    last_frame = esp_timer_get_time();
    while(true)
    {
        fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            res = ESP_FAIL;
            break;
        }

        if(fb->format != PIXFORMAT_JPEG){
            bool jpeg_converted = frame2jpg(fb, camera_config.jpeg_quality, &_jpg_buf, &_jpg_buf_len);
            if(!jpeg_converted){
                ESP_LOGE(TAG, "JPEG compression failed");
                esp_camera_fb_return(fb);
                res = ESP_FAIL;
            }
        } else {
            _jpg_buf_len = fb->len;
            _jpg_buf = fb->buf;
        }

        if(res == ESP_OK){
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, _STREAM_BOUNDARY_SIZE);
        }

        if(res == ESP_OK){
            // Only one time is needed.. The rest of the time it will be same size
            if (hlen == 0)
            {
                hlen = snprintf(part_buf, sizeof(part_buf), _STREAM_PART, _jpg_buf_len);
                ESP_LOGI(TAG, "MJPG: %uB", hlen);
            }

            res = httpd_resp_send_chunk(req, part_buf, hlen);
        }
        
        if(res == ESP_OK){
            res = httpd_resp_send_chunk(req, (const char*) _jpg_buf, _jpg_buf_len);
        }
        
        if(fb->format != PIXFORMAT_JPEG){
            free(_jpg_buf);
        }

        esp_camera_fb_return(fb);
        if(res != ESP_OK){
            break;
        }
        
        int64_t fr_end = esp_timer_get_time();
        int64_t timeMs = (fr_end - last_frame) / 1000;
        if (timeMs < 20)
        {
            ESP_LOGI(TAG, "MJPG: %uKB %lldms", _jpg_buf_len, timeMs);
        }

        last_frame = fr_end;

    }

    last_frame = 0;
    return res;
}


esp_err_t setup_server(void)
{
    static const httpd_uri_t uri_get = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = jpg_stream_httpd_handler,
        .user_ctx = NULL
    };

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t stream_httpd  = NULL;
    esp_err_t err;

    err = httpd_start(&stream_httpd , &config);
    if (err == ESP_OK)
    {
        err = httpd_register_uri_handler(stream_httpd , &uri_get);
    }

    return err;
}

void app_main(void)
{
#if !ESP_CAMERA_SUPPORTED
    ESP_LOGE(TAG, "Camera support is not available for this chip");
    return;
#endif

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    ESP_ERROR_CHECK(init_camera());


    connect_wifi();


    ESP_ERROR_CHECK(setup_server());

    /*
    while (1)
    {
        ESP_LOGI(TAG, "Taking picture...");
        camera_fb_t *pic = esp_camera_fb_get();

        // use pic->buf to access the image
        ESP_LOGI(TAG, "Picture taken! Its size was: %zu bytes", pic->len);
        esp_camera_fb_return(pic);

        vTaskDelay(5000 / portTICK_RATE_MS);
    }
        */
}
