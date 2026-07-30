// ============================================================
// main_old_cam.cpp
// Backup of main.cpp for the PREVIOUS camera module.
// Preserved before switching to OV5640 (DCXYX-LZTKQJ-5M-357-V1 FF).
// ============================================================

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_http_server.h"

// PIN DEFINITIONS
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     15
#define SIOD_GPIO_NUM      4
#define SIOC_GPIO_NUM      5
#define Y9_GPIO_NUM       16
#define Y8_GPIO_NUM       17
#define Y7_GPIO_NUM       18
#define Y6_GPIO_NUM       12
#define Y5_GPIO_NUM       10
#define Y4_GPIO_NUM        8
#define Y3_GPIO_NUM        9
#define Y2_GPIO_NUM       11
#define VSYNC_GPIO_NUM     6
#define HREF_GPIO_NUM      7
#define PCLK_GPIO_NUM     13

#define LED_RED_PIN    (gpio_num_t)2
#define LED_GREEN_PIN  (gpio_num_t)3

static const char* TAG = "FatigueSense_Cam";

// HTTP MJPEG Streaming Configuration
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

#include "img_converters.h"

// --- Camera Initialization ---
bool initCamera() {
    camera_config_t config = {};
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer   = LEDC_TIMER_0;
    config.pin_d0       = Y2_GPIO_NUM;
    config.pin_d1       = Y3_GPIO_NUM;
    config.pin_d2       = Y4_GPIO_NUM;
    config.pin_d3       = Y5_GPIO_NUM;
    config.pin_d4       = Y6_GPIO_NUM;
    config.pin_d5       = Y7_GPIO_NUM;
    config.pin_d6       = Y8_GPIO_NUM;
    config.pin_d7       = Y9_GPIO_NUM;
    config.pin_xclk     = XCLK_GPIO_NUM;
    config.pin_pclk     = PCLK_GPIO_NUM;
    config.pin_vsync    = VSYNC_GPIO_NUM;
    config.pin_href     = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn     = PWDN_GPIO_NUM;
    config.pin_reset    = RESET_GPIO_NUM;
    config.xclk_freq_hz = 10000000; // CRITICAL TEST: Dropping to 10MHz to see if the signal survives the FPC connector
    
    // Hardware JPEG Compression for max FPS
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size   = FRAMESIZE_VGA; 
    config.jpeg_quality = 12; 
    
    config.fb_count     = 2; 
    config.fb_location  = CAMERA_FB_IN_DRAM; // CRITICAL: Use internal SRAM to bypass the ESP32-S3 Octal PSRAM DMA bug!
    config.grab_mode    = CAMERA_GRAB_LATEST; 

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed! Code = 0x%x", err);
        return false;
    }
    return true;
}

// --- MJPEG Stream Handler ---
esp_err_t stream_handler(httpd_req_t *req) {
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    char * part_buf[64];

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if(res != ESP_OK){
        return res;
    }

    ESP_LOGI(TAG, "Stream connected!");

    while(true){
        fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            res = ESP_FAIL;
        } 
        
        if(res == ESP_OK){
            size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, fb->len);
            res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        }
        if(res == ESP_OK){
            res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
        }
        if(res == ESP_OK){
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        }
        
        if(fb){
            esp_camera_fb_return(fb);
            fb = NULL;
        }
        
        if(res != ESP_OK){
            ESP_LOGI(TAG, "Stream disconnected!");
            break;
        }
    }
    return res;
}

// --- WiFi Access Point Setup ---
void wifi_init_softap(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {};
    strcpy((char*)wifi_config.ap.ssid, "FatigueSense_Cam");
    wifi_config.ap.ssid_len = strlen("FatigueSense_Cam");
    strcpy((char*)wifi_config.ap.password, "safedriving");
    wifi_config.ap.max_connection = 4;
    wifi_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    // CRITICAL FIX: Lower WiFi TX power to prevent RF interference and brownouts on jumper wires!
    // 8 = ~2dBm (default is 78 = 19.5dBm)
    esp_wifi_set_max_tx_power(8);
    
    ESP_LOGI(TAG, "WiFi AP initialized. SSID: FatigueSense_Cam (Low Power Mode)");
}

// --- HTTP Server Setup ---
esp_err_t led_handler(httpd_req_t *req) {
    char buf[100];
    int state = 0;
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        char param[10];
        if (httpd_query_key_value(buf, "state", param, sizeof(param)) == ESP_OK) {
            state = atoi(param);
            // Toggle red LED pin
            gpio_set_level(LED_RED_PIN, state ? 1 : 0);
        }
    }
    const char* resp = "OK";
    httpd_resp_send(req, resp, strlen(resp));
    return ESP_OK;
}

void start_camera_server() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 81; 
    
    httpd_handle_t stream_httpd = NULL;
    if (httpd_start(&stream_httpd, &config) == ESP_OK) {
        httpd_uri_t stream_uri = {
            .uri       = "/stream",
            .method    = HTTP_GET,
            .handler   = stream_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(stream_httpd, &stream_uri);
        
        httpd_uri_t led_uri = {
            .uri       = "/led",
            .method    = HTTP_GET,
            .handler   = led_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(stream_httpd, &led_uri);

        ESP_LOGI(TAG, "Camera Server started on port 81");
    }
}

extern "C" void app_main() {
    // 500ms boot delay for camera power stabilization
    vTaskDelay(500 / portTICK_PERIOD_MS);

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Setup LEDs
    gpio_reset_pin(LED_RED_PIN);
    gpio_set_direction(LED_RED_PIN, GPIO_MODE_OUTPUT);
    gpio_reset_pin(LED_GREEN_PIN);
    gpio_set_direction(LED_GREEN_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_RED_PIN, 0);
    gpio_set_level(LED_GREEN_PIN, 1); // Green on to indicate ready

    // Start WiFi FIRST, with lower TX power
    wifi_init_softap();
    start_camera_server();

    // Initialize Camera LAST
    if (!initCamera()) {
        ESP_LOGE(TAG, "CRITICAL CAMERA INIT FAILURE!");
        gpio_set_level(LED_RED_PIN, 1);
        gpio_set_level(LED_GREEN_PIN, 0);
        while (true) { vTaskDelay(1000 / portTICK_PERIOD_MS); }
    }

    // The main loop just sleeps, the HTTP server and camera run in background threads
    while (true) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
