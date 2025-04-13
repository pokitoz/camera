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

#include "camera.h"

#include "esp_http_server.h"
#include "esp_timer.h"
#include "publisher.h"


static const char *LOG_TAG = "picture:stream";

esp_err_t jpg_stream_httpd_handler(httpd_req_t *req)
{

#define PART_BOUNDARY "123456789000000000000987654321"
    static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
    static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
    const size_t _STREAM_BOUNDARY_SIZE = strlen(_STREAM_BOUNDARY);
    static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %lu\r\n\r\n";

    const bool convertToJpeg  = (get_camera_format() != PIXFORMAT_JPEG);

    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len;
    uint8_t *_jpg_buf;
    char part_buf[64];
    int64_t last_frame = 0;
    size_t hlen = 0;

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if (res != ESP_OK)
    {
        return res;
    }

    while (true)
    {
        last_frame = esp_timer_get_time();
        fb = esp_camera_fb_get();
        if (!fb)
        {
            ESP_LOGE(LOG_TAG, "Camera capture failed");
            res = ESP_FAIL;
            break;
        }

        if (convertToJpeg)
        {
            bool jpeg_converted = frame2jpg(fb, get_camera_format(), &_jpg_buf, &_jpg_buf_len);
            if (!jpeg_converted)
            {
                ESP_LOGE(LOG_TAG, "JPEG compression failed");
                esp_camera_fb_return(fb);
                res = ESP_FAIL;
            }
        }
        else
        {
            _jpg_buf_len = fb->len;
            _jpg_buf = fb->buf;
        }

        if (res == ESP_OK)
        {
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, _STREAM_BOUNDARY_SIZE);
        }

        if (res == ESP_OK)
        {
            // Only one time is needed.. The rest of the time it will be same size
            if (hlen == 0)
            {
                hlen = snprintf(part_buf, sizeof(part_buf), _STREAM_PART, _jpg_buf_len);
                ESP_LOGI(LOG_TAG, "MJPG: %uB", hlen);
            }

            res = httpd_resp_send_chunk(req, part_buf, hlen);
        }

        if (res == ESP_OK)
        {
            res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
        }

        if (convertToJpeg)
        {
            free(_jpg_buf);
        }

        esp_camera_fb_return(fb);
        if (res != ESP_OK)
        {
            break;
        }

        int64_t timeMs = (esp_timer_get_time() - last_frame) / 1000;
        if (timeMs < 20)
        {
            ESP_LOGI(LOG_TAG, "MJPG: %uKB %lldms", _jpg_buf_len, timeMs);
        }
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
        .user_ctx = NULL};

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t stream_httpd = NULL;
    esp_err_t err;

    err = httpd_start(&stream_httpd, &config);
    if (err == ESP_OK)
    {
        err = httpd_register_uri_handler(stream_httpd, &uri_get);
    }

    return err;
}

void app_main(void)
{
#if !ESP_CAMERA_SUPPORTED
    ESP_LOGE(LOG_TAG, "Camera support is not available for this chip");
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

    start_publish();
}
