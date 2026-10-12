// Minimal browser camera stream. Previous application is commented out below.
#include <cstdio>
#include <atomic>
#include <cstdlib>
#include "esp_timer.h"
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

static const char *TAG = "CameraOnly";
static constexpr gpio_num_t RED_LED = GPIO_NUM_1;
static constexpr gpio_num_t GREEN_LED = GPIO_NUM_2;

static void init_leds() {
    ESP_ERROR_CHECK(gpio_set_level(RED_LED, 0));
    ESP_ERROR_CHECK(gpio_set_level(GREEN_LED, 0));
    gpio_config_t config = {};
    config.pin_bit_mask = (1ULL << RED_LED) | (1ULL << GREEN_LED);
    config.mode = GPIO_MODE_OUTPUT;
    ESP_ERROR_CHECK(gpio_config(&config));
}

static void init_buzzer() {
    // Camera uses timer/channel 0; the existing transistor-driven buzzer is GPIO21.
    ledc_timer_config_t timer = {};
    timer.speed_mode = LEDC_LOW_SPEED_MODE;
    timer.duty_resolution = LEDC_TIMER_10_BIT;
    timer.timer_num = LEDC_TIMER_1;
    timer.freq_hz = 100;
    timer.clk_cfg = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&timer));
    ledc_channel_config_t channel = {};
    channel.gpio_num = 21;
    channel.speed_mode = LEDC_LOW_SPEED_MODE;
    channel.channel = LEDC_CHANNEL_1;
    channel.timer_sel = LEDC_TIMER_1;
    channel.intr_type = LEDC_INTR_DISABLE;
    channel.duty = 0;
    ESP_ERROR_CHECK(ledc_channel_config(&channel));
}

// Only this task writes buzzer duty. App alerts never write LED pins.
static std::atomic<int> buzzer_mode{0}; // 0 off, 1 fatigue ramp, 2 three beeps, 3 browser beep
static void app_buzzer_task(void *) {
    int previous = 0;
    int64_t started = 0;
    while (true) {
        int mode = buzzer_mode.load();
        if (mode != previous) { started = esp_timer_get_time(); previous = mode; }
        int elapsed = static_cast<int>((esp_timer_get_time() - started) / 1000);
        int duty = 0;
        bool finished = false;
        if (mode == 1) duty = elapsed >= 2500 ? 1023 : 40 + 983 * elapsed / 2500;
        if (mode == 2) {
            duty = (elapsed < 200 || (elapsed >= 350 && elapsed < 550) ||
                    (elapsed >= 700 && elapsed < 900)) ? 1023 : 0;
            finished = elapsed >= 1500;
        }
        if (mode == 3) { duty = elapsed < 500 ? 1023 : 0; finished = elapsed >= 500; }
        if (finished) { int expected = mode; buzzer_mode.compare_exchange_strong(expected, 0); }
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1));
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static esp_err_t health_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_send(req, "ok", 2);
}

static esp_err_t app_alert_handler(httpd_req_t *req) {
    char query[128], type[16] = {}, sequence[16] = {}, outputs[16] = {};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "type", type, sizeof(type)) != ESP_OK ||
        httpd_query_key_value(query, "seq", sequence, sizeof(sequence)) != ESP_OK ||
        httpd_query_key_value(query, "outputs", outputs, sizeof(outputs)) != ESP_OK ||
        std::strcmp(outputs, "buzzer") != 0) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "type, seq and outputs=buzzer required");
    }
    if (!sequence[0] || std::strspn(sequence, "0123456789") != std::strlen(sequence))
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid seq");
    unsigned long long parsed = std::strtoull(sequence, nullptr, 10);
    if (parsed == 0 || parsed > UINT32_MAX)
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid seq");
    int mode;
    if (std::strcmp(type, "fatigue") == 0) mode = 1;
    else if (std::strcmp(type, "yawn_fatigue") == 0) mode = 2;
    else if (std::strcmp(type, "clear") == 0 || std::strcmp(type, "yawn") == 0) mode = 0;
    else return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "unknown alert type");
    static uint32_t last_seq = 0;
    uint32_t seq = static_cast<uint32_t>(parsed);
    if ((seq >> 16) == (last_seq >> 16) && seq <= last_seq)
        return httpd_resp_send(req, "stale", 5);
    last_seq = seq;
    // A yawn-only notification must not interrupt an ongoing three-beep alert.
    if (!(std::strcmp(type, "yawn") == 0 && buzzer_mode.load() == 2))
        buzzer_mode.store(mode);
    ESP_LOGI(TAG, "App buzzer-only alert: %s seq=%lu", type, static_cast<unsigned long>(seq));
    return health_handler(req);
}

static esp_err_t output_handler(httpd_req_t *req) {
    const char *label = static_cast<const char *>(req->user_ctx);
    const bool all = std::strcmp(label, "B4: All three") == 0;
    const bool red = all || std::strcmp(label, "B1: Red LED") == 0;
    const bool green = all || std::strcmp(label, "B2: Green LED") == 0;
    const bool buzz = all || std::strcmp(label, "B3: Buzzer") == 0;
    ESP_LOGI(TAG, "%s: ON for 500 ms", label);
    ESP_ERROR_CHECK(gpio_set_level(RED_LED, red));
    ESP_ERROR_CHECK(gpio_set_level(GREEN_LED, green));
    buzzer_mode.store(buzz ? 3 : 0);
    // Only the control server waits; the stream has its own server task.
    // Always turn off locally, even if the browser disconnects during the beep.
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_ERROR_CHECK(gpio_set_level(RED_LED, 0));
    ESP_ERROR_CHECK(gpio_set_level(GREEN_LED, 0));
    buzzer_mode.store(0);
    ESP_LOGI(TAG, "%s: OFF", label);
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_send(req, "Test complete - outputs off", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t init_camera() {
    camera_config_t config = {};
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = 11;
    config.pin_d1 = 9;
    config.pin_d2 = 8;
    config.pin_d3 = 10;
    config.pin_d4 = 12;
    config.pin_d5 = 18;
    config.pin_d6 = 17;
    config.pin_d7 = 16;
    config.pin_xclk = 15;
    config.pin_pclk = 13;
    config.pin_vsync = 6;
    config.pin_href = 7;
    config.pin_sccb_sda = 4;
    config.pin_sccb_scl = 5;
    config.pin_pwdn = -1;
    config.pin_reset = -1;
    config.xclk_freq_hz = 10000000;
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = FRAMESIZE_QVGA; // 320x240 diagnostic stream
    config.jpeg_quality = 20;
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    return esp_camera_init(&config);
}

static esp_err_t stream_handler(httpd_req_t *req) {
    esp_err_t result = httpd_resp_set_type(
        req, "multipart/x-mixed-replace;boundary=frame");
    if (result != ESP_OK) return result;
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    ESP_LOGI(TAG, "Browser stream connected");
    while (true) {
        camera_fb_t *frame = esp_camera_fb_get();
        if (!frame) {
            ESP_LOGE(TAG, "Camera capture failed");
            return ESP_FAIL;
        }
        if (!frame->buf || frame->format != PIXFORMAT_JPEG || frame->len < 4 ||
            frame->buf[0] != 0xff || frame->buf[1] != 0xd8 ||
            frame->buf[frame->len - 2] != 0xff || frame->buf[frame->len - 1] != 0xd9) {
            esp_camera_fb_return(frame);
            ESP_LOGE(TAG, "Invalid JPEG; ending stream");
            return ESP_FAIL;
        }
        char header[96];
        int length = std::snprintf(header, sizeof(header),
            "\r\n--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
            static_cast<unsigned>(frame->len));
        if (length < 0 || static_cast<size_t>(length) >= sizeof(header)) {
            esp_camera_fb_return(frame);
            return ESP_FAIL;
        }
        result = httpd_resp_send_chunk(req, header, length);
        if (result == ESP_OK) {
            result = httpd_resp_send_chunk(req,
                reinterpret_cast<const char *>(frame->buf), frame->len);
        }
        esp_camera_fb_return(frame);
        if (result != ESP_OK) {
            ESP_LOGI(TAG, "Browser stream ended: %s", esp_err_to_name(result));
            return result;
        }
        // Limit load to below 10 FPS; never delay while holding a frame buffer.
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static esp_err_t page_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req,
        "<!doctype html><html><head><meta name='viewport' content='width=device-width'>"
        "<title>Camera</title></head><body><h2>Camera stream</h2>"
        "<img id='camera' style='max-width:100%;height:auto' alt='Camera stream'>"
        "<p>Each button activates its outputs for 0.5 seconds.</p>"
        "<style>button{padding:16px;margin:6px;font-size:18px}</style>"
        "<button type='button' data-path='/red'>B1: Red LED</button>"
        "<button type='button' data-path='/green'>B2: Green LED</button>"
        "<button type='button' data-path='/buzzer'>B3: Buzzer</button>"
        "<button type='button' data-path='/all'>B4: All three</button>"
        "<p id='status' role='status'></p>"
        "<script>"
        "document.getElementById('camera').src='http://'+location.hostname+':81/stream';"
        "const buttons=document.querySelectorAll('button'),status=document.getElementById('status');"
        "buttons.forEach(button=>button.onclick=async()=>{"
        "buttons.forEach(b=>b.disabled=true);status.textContent=button.textContent+' running...';"
        "try{const r=await fetch(button.dataset.path,{method:'POST',signal:AbortSignal.timeout(5000)});"
        "if(!r.ok)throw new Error('HTTP '+r.status);status.textContent=await r.text();}"
        "catch(e){status.textContent='Request failed. Check the ESP32 log.';}"
        "finally{buttons.forEach(b=>b.disabled=false);}});"
        "</script>"
        "</body></html>", HTTPD_RESP_USE_STRLEN);
}

static void start_camera_server() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.stack_size = 8192;
    config.send_wait_timeout = 3;
    config.recv_wait_timeout = 3;
    httpd_handle_t server = nullptr;
    ESP_ERROR_CHECK(httpd_start(&server, &config));
    httpd_uri_t page = {};
    page.uri = "/";
    page.method = HTTP_GET;
    page.handler = page_handler;
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &page));
    static const char *paths[] = {"/red", "/green", "/buzzer", "/all"};
    static const char *labels[] = {"B1: Red LED", "B2: Green LED", "B3: Buzzer", "B4: All three"};
    for (int i = 0; i < 4; ++i) {
        httpd_uri_t output = {};
        output.uri = paths[i];
        output.method = HTTP_POST;
        output.handler = output_handler;
        output.user_ctx = const_cast<char *>(labels[i]);
        ESP_ERROR_CHECK(httpd_register_uri_handler(server, &output));
    }

    httpd_uri_t alert = {};
    alert.uri = "/alert";
    alert.method = HTTP_GET;
    alert.handler = app_alert_handler;
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &alert));
    httpd_uri_t health = {};
    health.uri = "/health";
    health.method = HTTP_GET;
    health.handler = health_handler;
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &health));

    // A continuous stream blocks its HTTP server task, so use a second server.
    config.server_port = 81;
    config.ctrl_port = 32769;
    httpd_handle_t stream_server = nullptr;
    ESP_ERROR_CHECK(httpd_start(&stream_server, &config));
    httpd_uri_t stream = {};
    stream.uri = "/stream";
    stream.method = HTTP_GET;
    stream.handler = stream_handler;
    ESP_ERROR_CHECK(httpd_register_uri_handler(stream_server, &stream));
    ESP_LOGI(TAG, "Open http://192.168.4.1/ in a browser (one stream at a time)");
}

static void wifi_event_handler(void *, esp_event_base_t base,
                               int32_t id, void *data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STACONNECTED) {
        auto *event = static_cast<wifi_event_ap_staconnected_t *>(data);
        ESP_LOGI(TAG, "Phone connected, AID=%d", event->aid);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STADISCONNECTED) {
        auto *event = static_cast<wifi_event_ap_stadisconnected_t *>(data);
        ESP_LOGW(TAG, "Phone disconnected, AID=%d reason=%d",
                 event->aid, event->reason);
    } else if (base == IP_EVENT && id == IP_EVENT_AP_STAIPASSIGNED) {
        auto *event = static_cast<ip_event_ap_staipassigned_t *>(data);
        ESP_LOGI(TAG, "Phone assigned IP: " IPSTR, IP2STR(&event->ip));
    }
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "CAMERA + OUTPUT TEST: browser buttons for red, green, buzzer and all three");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *ap = esp_netif_create_default_wifi_ap();
    if (!ap) {
        ESP_LOGE(TAG, "Failed to create AP network interface");
        return;
    }
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));
    ESP_ERROR_CHECK(esp_event_handler_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register(
        IP_EVENT, IP_EVENT_AP_STAIPASSIGNED, wifi_event_handler, nullptr));

    wifi_config_t config = {};
    std::strcpy(reinterpret_cast<char *>(config.ap.ssid), "FatigueSense_Cam");
    config.ap.ssid_len = std::strlen("FatigueSense_Cam");
    std::strcpy(reinterpret_cast<char *>(config.ap.password), "safedriving");
    config.ap.channel = 1;
    config.ap.max_connection = 4;
    config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &config));
    ESP_ERROR_CHECK(esp_wifi_start());
    // Preserve the previous application's configured transmit power.
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(40));
    ESP_LOGI(TAG, "Hotspot ready: FatigueSense_Cam | IP: 192.168.4.1 | no Internet");
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_err_t camera_result = init_camera();
    if (camera_result != ESP_OK) {
        ESP_LOGE(TAG, "Camera initialization failed: %s; hotspot remains active",
                 esp_err_to_name(camera_result));
        return;
    }
    ESP_LOGI(TAG, "Camera initialized");
    init_leds();
    init_buzzer();
    if (xTaskCreate(app_buzzer_task, "app_buzzer", 2048, nullptr, 5, nullptr) != pdPASS) {
        ESP_LOGE(TAG, "Could not start buzzer task");
        return;
    }
    start_camera_server();
    // Wi-Fi, DHCP and HTTP continue in ESP-IDF's background tasks.
}

// ================= PREVIOUS APPLICATION (COMMENTED OUT) =================
// // ============================================================
// // main.cpp
// // FatigueSense ESP32-S3 — Camera streaming + LED/Buzzer control
// // WiFi: SoftAP mode (SSID: FatigueSense_Cam)
// // Camera stream: port 81  /stream
// // LED/Buzzer UI: port 80  /
// // ============================================================
// 
// #include <stdio.h>
// #include <string.h>
// #include <stdlib.h>
// #include <inttypes.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "driver/gpio.h"
// #include "driver/ledc.h"
// #include "esp_system.h"
// #include "esp_timer.h"
// #include "esp_camera.h"
// #include "esp_log.h"
// #include "esp_wifi.h"
// #include "esp_event.h"
// #include "nvs_flash.h"
// #include "esp_http_server.h"
// #include "img_converters.h"
// 
// // ============================================================
// // PIN DEFINITIONS — Camera (OV5640 / previous module)
// // ============================================================
// #define PWDN_GPIO_NUM     -1
// #define RESET_GPIO_NUM    -1
// #define XCLK_GPIO_NUM     15
// #define SIOD_GPIO_NUM      4
// #define SIOC_GPIO_NUM      5
// #define Y9_GPIO_NUM       16
// #define Y8_GPIO_NUM       17
// #define Y7_GPIO_NUM       18
// #define Y6_GPIO_NUM       12
// #define Y5_GPIO_NUM       10
// #define Y4_GPIO_NUM        8
// #define Y3_GPIO_NUM        9
// #define Y2_GPIO_NUM       11
// #define VSYNC_GPIO_NUM     6
// #define HREF_GPIO_NUM      7
// #define PCLK_GPIO_NUM     13
// 
// // ============================================================
// // PIN DEFINITIONS — LEDs & Buzzer
// // ============================================================
// #define LED1_GPIO       GPIO_NUM_1
// #define LED2_GPIO       GPIO_NUM_2
// #define BUZZER_GPIO     GPIO_NUM_21   // Drives transistor base via 1kΩ resistor; PWM-driven
// 
// // ============================================================
// // Buzzer PWM config
// // NOTE: Camera uses LEDC_CHANNEL_0 / LEDC_TIMER_0 for XCLK.
// //       Buzzer is assigned LEDC_CHANNEL_1 / LEDC_TIMER_1 to avoid conflict.
// // ============================================================
// #define BUZZER_LEDC_TIMER       LEDC_TIMER_1
// #define BUZZER_LEDC_MODE        LEDC_LOW_SPEED_MODE
// #define BUZZER_LEDC_CHANNEL     LEDC_CHANNEL_1
// #define BUZZER_LEDC_DUTY_RES    LEDC_TIMER_10_BIT   // 0–1023, finer steps
// #define BUZZER_LEDC_FREQUENCY   100                 // Slow gate freq — buzzer fully on/off each cycle
// 
// #define BUZZER_MIN_DUTY     40    // ~4% — short quiet pulses, just above oscillator threshold
// #define BUZZER_MAX_DUTY     1023  // 100% — continuous full tone
// #define YAWN_BEEP_DUTY      BUZZER_MAX_DUTY
// #define BUZZER_RAMP_MS      2500  // Time to go from min → max while held
// #define BUZZER_STEP_MS      30    // Smaller steps = smoother perceived ramp
// 
// // ============================================================
// // HTTP Streaming constants
// // ============================================================
// #define PART_BOUNDARY "123456789000000000000987654321"
// static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
// static const char* _STREAM_BOUNDARY     = "\r\n--" PART_BOUNDARY "\r\n";
// static const char* _STREAM_PART        = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";
// 
// static const char *TAG = "FatigueSense";
// 
// // ============================================================
// // Shared state (volatile — written by HTTP handlers, read by tasks)
// // ============================================================
// static volatile bool led1_on    = false;
// static volatile bool led2_on    = false;
// static volatile bool blink_mode = false;
// static volatile bool buzzer_held = false;
// static volatile bool yawn_beep_active = false;
// static volatile int ping_blink_count = 0;
// static volatile bool wifi_station_connected = false;
// static volatile bool stream_connected       = false;
// static volatile char current_alert_type[16] = "clear";
// static volatile bool any_alert_active = false;
// static uint32_t last_alert_seq = 0;
// static uint16_t last_alert_session = 0;
// static int current_duty = 0;
// 
// // ============================================================
// // Hardware initialisation
// // ============================================================
// 
// static void gpio_outputs_init(void) {
//     // BUZZER_GPIO is owned by the LEDC peripheral — do NOT include it here.
//     gpio_config_t io_conf = {
//         .pin_bit_mask = (1ULL << LED1_GPIO) | (1ULL << LED2_GPIO),
//         .mode         = GPIO_MODE_OUTPUT,
//         .pull_up_en   = GPIO_PULLUP_DISABLE,
//         .pull_down_en = GPIO_PULLDOWN_DISABLE,
//         .intr_type    = GPIO_INTR_DISABLE,
//     };
//     gpio_config(&io_conf);
// }
// 
// static void buzzer_pwm_init(void) {
//     ledc_timer_config_t ledc_timer = {
//         .speed_mode      = BUZZER_LEDC_MODE,
//         .duty_resolution = BUZZER_LEDC_DUTY_RES,
//         .timer_num       = BUZZER_LEDC_TIMER,
//         .freq_hz         = BUZZER_LEDC_FREQUENCY,
//         .clk_cfg         = LEDC_AUTO_CLK
//     };
//     ledc_timer_config(&ledc_timer);
// 
//     ledc_channel_config_t ledc_channel = {
//         .gpio_num   = BUZZER_GPIO,
//         .speed_mode = BUZZER_LEDC_MODE,
//         .channel    = BUZZER_LEDC_CHANNEL,
//         .intr_type  = LEDC_INTR_DISABLE,
//         .timer_sel  = BUZZER_LEDC_TIMER,
//         .duty       = 0,
//         .hpoint     = 0
//     };
//     ledc_channel_config(&ledc_channel);
// }
// 
// // ============================================================
// // Camera initialisation
// // ============================================================
// static bool initCamera(void) {
//     camera_config_t config = {};
//     config.ledc_channel = LEDC_CHANNEL_0;   // Camera owns channel 0 / timer 0
//     config.ledc_timer   = LEDC_TIMER_0;
//     config.pin_d0       = Y2_GPIO_NUM;
//     config.pin_d1       = Y3_GPIO_NUM;
//     config.pin_d2       = Y4_GPIO_NUM;
//     config.pin_d3       = Y5_GPIO_NUM;
//     config.pin_d4       = Y6_GPIO_NUM;
//     config.pin_d5       = Y7_GPIO_NUM;
//     config.pin_d6       = Y8_GPIO_NUM;
//     config.pin_d7       = Y9_GPIO_NUM;
//     config.pin_xclk     = XCLK_GPIO_NUM;
//     config.pin_pclk     = PCLK_GPIO_NUM;
//     config.pin_vsync    = VSYNC_GPIO_NUM;
//     config.pin_href     = HREF_GPIO_NUM;
//     config.pin_sscb_sda = SIOD_GPIO_NUM;
//     config.pin_sscb_scl = SIOC_GPIO_NUM;
//     config.pin_pwdn     = PWDN_GPIO_NUM;
//     config.pin_reset    = RESET_GPIO_NUM;
//     config.xclk_freq_hz = 20000000; // 20 MHz XCLK for higher sensor FPS
// 
//     // Hardware JPEG compression for max FPS & clarity on OV2640
//     config.pixel_format = PIXFORMAT_JPEG;
//     config.frame_size   = FRAMESIZE_VGA;
//     config.jpeg_quality = 12; // Quality setting: 12 (balanced high FPS & detail over WiFi)
// 
//     config.fb_count    = 2;
//     config.fb_location = CAMERA_FB_IN_DRAM; // CRITICAL: bypass ESP32-S3 Octal PSRAM DMA bug
//     config.grab_mode   = CAMERA_GRAB_LATEST;
// 
//     esp_err_t err = esp_camera_init(&config);
//     if (err != ESP_OK) {
//         ESP_LOGE(TAG, "Camera init failed! Code = 0x%x", err);
//         return false;
//     }
//     return true;
// }
// 
// // ============================================================
// // FreeRTOS Tasks
// // ============================================================
// 
// // Ramps buzzer volume up while held; drops to 0 instantly on release
// static void buzzer_task(void *arg) {
//     static const int yawn_phase_ms[] = { 200, 150, 200, 150, 200, 600 };
//     bool was_yawn_active = false;
//     int yawn_phase = 0;
//     int yawn_elapsed_ms = 0;
// 
//     while (1) {
//         if (buzzer_held) {
//             was_yawn_active = false;
//             yawn_phase = 0;
//             yawn_elapsed_ms = 0;
//             if (current_duty == 0) {
//                 current_duty = BUZZER_MIN_DUTY;
//             } else if (current_duty < BUZZER_MAX_DUTY) {
//                 current_duty += (BUZZER_MAX_DUTY - BUZZER_MIN_DUTY) * BUZZER_STEP_MS / BUZZER_RAMP_MS;
//                 if (current_duty > BUZZER_MAX_DUTY) current_duty = BUZZER_MAX_DUTY;
//             }
//             ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, current_duty);
//         } else if (yawn_beep_active) {
//             if (!was_yawn_active) {
//                 yawn_phase = 0;
//                 yawn_elapsed_ms = 0;
//                 was_yawn_active = true;
//             }
//             const bool beep_on = (yawn_phase == 0 || yawn_phase == 2 || yawn_phase == 4);
//             current_duty = beep_on ? YAWN_BEEP_DUTY : 0;
//             yawn_elapsed_ms += BUZZER_STEP_MS;
//             if (yawn_elapsed_ms >= yawn_phase_ms[yawn_phase]) {
//                 yawn_elapsed_ms = 0;
//                 if (yawn_phase == 5) {
//                     // MAR fatigue is a one-shot alert: stop after exactly
//                     // three beeps and wait for the next distinct command.
//                     yawn_beep_active = false;
//                     led1_on = false;
//                     any_alert_active = false;
//                     strcpy((char *)current_alert_type, "clear");
//                     yawn_phase = 0;
//                 } else {
//                     yawn_phase++;
//                 }
//             }
//             ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, current_duty);
//         } else {
//             was_yawn_active = false;
//             yawn_phase = 0;
//             yawn_elapsed_ms = 0;
//             current_duty = 0;
//             ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, 0);
//         }
//         ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL);
//         vTaskDelay(pdMS_TO_TICKS(BUZZER_STEP_MS));
//     }
// }
// 
// // Handles LED blink mode; manual on/off is ignored while blinking
// static void led_task(void *arg) {
//     bool phase = false;
//     while (1) {
//         if (ping_blink_count > 0) {
//             gpio_set_level(LED2_GPIO, (ping_blink_count % 2 != 0) ? 1 : 0);
//             ping_blink_count--;
//             vTaskDelay(pdMS_TO_TICKS(100));
//         } else if (blink_mode) {
//             phase = !phase;
//             gpio_set_level(LED1_GPIO, phase ? 1 : 0);
//             gpio_set_level(LED2_GPIO, phase ? 0 : 1);
//             vTaskDelay(pdMS_TO_TICKS(300));
//         } else {
//             bool app_connected = wifi_station_connected || stream_connected;
//             gpio_set_level(LED1_GPIO, led1_on ? 1 : 0);
//             gpio_set_level(LED2_GPIO, (led2_on || (app_connected && !any_alert_active)) ? 1 : 0);
//             vTaskDelay(pdMS_TO_TICKS(50));
//         }
//     }
// }
// 
// // ============================================================
// // WiFi — SoftAP
// // ============================================================
// static void wifi_event_handler(void* arg, esp_event_base_t event_base,
//                                int32_t event_id, void* event_data)
// {
//     if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
//         wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
//         ESP_LOGI(TAG, "Station connected, AID=%d", event->aid);
//         wifi_station_connected = true;
//     } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
//         wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
//         ESP_LOGI(TAG, "Station disconnected, AID=%d", event->aid);
//         wifi_station_connected = false;
//     }
// }
// 
// static void wifi_init_softap(void) {
//     ESP_ERROR_CHECK(esp_netif_init());
//     ESP_ERROR_CHECK(esp_event_loop_create_default());
//     esp_netif_create_default_wifi_ap();
// 
//     wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
//     ESP_ERROR_CHECK(esp_wifi_init(&cfg));
// 
//     ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
//                                                         ESP_EVENT_ANY_ID,
//                                                         &wifi_event_handler,
//                                                         NULL,
//                                                         NULL));
// 
//     wifi_config_t wifi_config = {};
//     strcpy((char*)wifi_config.ap.ssid,     "FatigueSense_Cam");
//     wifi_config.ap.ssid_len     = strlen("FatigueSense_Cam");
//     strcpy((char*)wifi_config.ap.password, "safedriving");
//     wifi_config.ap.max_connection = 4;
//     wifi_config.ap.authmode       = WIFI_AUTH_WPA_WPA2_PSK;
// 
//     ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
//     ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
//     ESP_ERROR_CHECK(esp_wifi_start());
// 
//     // Moderate AP transmit power for reliable phone connections.
//     // ESP-IDF units are quarter-dBm: 40 is approximately 10 dBm.
//     esp_wifi_set_max_tx_power(40);
// 
//     ESP_LOGI(TAG, "WiFi AP started. SSID: FatigueSense_Cam (Low Power Mode)");
// }
// 
// // ============================================================
// // HTTP handlers — Camera stream (port 81)
// // ============================================================
// static esp_err_t stream_handler(httpd_req_t *req) {
//     camera_fb_t *fb = NULL;
//     esp_err_t res = ESP_OK;
//     char part_buf[64];
// 
//     res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
//     if (res != ESP_OK) return res;
// 
//     ESP_LOGI(TAG, "Stream connected!");
//     stream_connected = true;
// 
//     while (true) {
//         fb = esp_camera_fb_get();
//         if (!fb) {
//             ESP_LOGE(TAG, "Camera capture failed");
//             res = ESP_FAIL;
//         }
// 
//         if (res == ESP_OK) {
//             size_t hlen = snprintf(part_buf, sizeof(part_buf), _STREAM_PART, fb->len);
//             res = httpd_resp_send_chunk(req, part_buf, hlen);
//         }
//         if (res == ESP_OK) {
//             res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
//         }
//         if (res == ESP_OK) {
//             res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
//         }
// 
//         if (fb) {
//             esp_camera_fb_return(fb);
//             fb = NULL;
//         }
// 
//         if (res != ESP_OK) {
//             ESP_LOGI(TAG, "Stream disconnected!");
//             stream_connected = false;
//             break;
//         }
//     }
//     stream_connected = false;
//     return res;
// }
// 
// // ============================================================
// // HTTP handlers — LED & Buzzer control (port 80)
// // ============================================================
// static const char *html_page =
//     "<!DOCTYPE html><html><head>"
//     "<meta name='viewport' content='width=device-width,initial-scale=1'>"
//     "<title>FatigueSense Control</title>"
//     "<style>"
//     "body{font-family:sans-serif;text-align:center;padding:20px;background:#111;color:#eee}"
//     "h2{color:#4fc3f7}"
//     "button{font-size:18px;padding:14px 24px;margin:8px;border-radius:8px;border:none;"
//     "background:#333;color:#fff;cursor:pointer;transition:background .2s}"
//     "button.on{background:#2a9d3d}"
//     "button:active{opacity:.7}"
//     "</style></head><body>"
//     "<h2>FatigueSense — LED &amp; Buzzer</h2>"
//     "<div>"
//     "<button id='b1' onclick=\"toggle('led1')\">LED 1</button>"
//     "<button id='b2' onclick=\"toggle('led2')\">LED 2</button>"
//     "<button id='bb' onclick=\"toggle('blink')\">Blink</button>"
//     "</div>"
//     "<div>"
//     "<button ontouchstart=\"buzz(1)\" ontouchend=\"buzz(0)\" "
//     "onmousedown=\"buzz(1)\" onmouseup=\"buzz(0)\" onmouseleave=\"buzz(0)\""
//     ">&#128266; Hold to Buzz</button>"
//     "</div>"
//     "<p><a href='http://192.168.4.1:81/stream' style='color:#4fc3f7'>&#127909; Camera Stream</a></p>"
//     "<script>"
//     "let state={led1:false,led2:false,blink:false};"
//     "function toggle(k){"
//     "  state[k]=!state[k];"
//     "  fetch('/'+k+'/'+(state[k]?'on':'off'));"
//     "  render();"
//     "}"
//     "function buzz(v){fetch('/buzzer/'+(v?'on':'off'));}"
//     "function render(){"
//     "  document.getElementById('b1').className=state.led1?'on':'';"
//     "  document.getElementById('b2').className=state.led2?'on':'';"
//     "  document.getElementById('bb').className=state.blink?'on':'';"
//     "}"
//     "</script></body></html>";
// 
// static esp_err_t root_get_handler(httpd_req_t *req)     { httpd_resp_send(req, html_page, HTTPD_RESP_USE_STRLEN); return ESP_OK; }
// 
// static esp_err_t led1_on_handler(httpd_req_t *req)      { led1_on = true;     ESP_LOGI(TAG, "LED1 ON");     httpd_resp_send(req, "ok", 2); return ESP_OK; }
// static esp_err_t led1_off_handler(httpd_req_t *req)     { led1_on = false;    ESP_LOGI(TAG, "LED1 OFF");    httpd_resp_send(req, "ok", 2); return ESP_OK; }
// static esp_err_t led2_on_handler(httpd_req_t *req)      { led2_on = true;     ESP_LOGI(TAG, "LED2 ON");     httpd_resp_send(req, "ok", 2); return ESP_OK; }
// static esp_err_t led2_off_handler(httpd_req_t *req)     { led2_on = false;    ESP_LOGI(TAG, "LED2 OFF");    httpd_resp_send(req, "ok", 2); return ESP_OK; }
// static esp_err_t blink_on_handler(httpd_req_t *req)     { blink_mode = true;  ESP_LOGI(TAG, "Blink ON");    httpd_resp_send(req, "ok", 2); return ESP_OK; }
// static esp_err_t blink_off_handler(httpd_req_t *req)    { blink_mode = false; ESP_LOGI(TAG, "Blink OFF");   httpd_resp_send(req, "ok", 2); return ESP_OK; }
// static esp_err_t buzzer_on_handler(httpd_req_t *req)    { buzzer_held = true;  ESP_LOGI(TAG, "Buzzer ON");  httpd_resp_send(req, "ok", 2); return ESP_OK; }
// static esp_err_t buzzer_off_handler(httpd_req_t *req)   { buzzer_held = false; ESP_LOGI(TAG, "Buzzer OFF"); httpd_resp_send(req, "ok", 2); return ESP_OK; }
// 
// // ── Android FatigueSense alert endpoint ──────────────────────────────────────
// // Called by Esp32AlertSender in the Android app:
// //   GET /alert?type=fatigue&seq=N      → continuous buzzer + LED1
// //   GET /alert?type=yawn_fatigue&seq=N → three beeps + LED1
// //   GET /alert?type=yawn&seq=N         → LED2 only
// //   GET /alert?type=clear&seq=N        → all alert outputs off
// static esp_err_t alert_handler(httpd_req_t *req) {
//     char buf[128];
//     char type[16] = "";
//     char seq_text[16] = "";
// 
//     // Parse query string for "type" parameter
//     if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
//         httpd_query_key_value(buf, "type", type, sizeof(type));
//         httpd_query_key_value(buf, "seq", seq_text, sizeof(seq_text));
//     }
// 
//     if (type[0] == '\0' || seq_text[0] == '\0') {
//         ESP_LOGW(TAG, "ALERT: missing type or seq");
//         httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "type and seq required");
//         return ESP_OK;
//     }
// 
//     char *end = NULL;
//     unsigned long parsed_seq = strtoul(seq_text, &end, 10);
//     if (end == seq_text || *end != '\0' || parsed_seq > UINT32_MAX) {
//         ESP_LOGW(TAG, "ALERT: invalid seq='%s'", seq_text);
//         httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid seq");
//         return ESP_OK;
//     }
// 
//     uint32_t incoming_seq = (uint32_t)parsed_seq;
//     uint16_t incoming_session = (uint16_t)(incoming_seq >> 16);
//     if (incoming_session != last_alert_session) {
//         last_alert_session = incoming_session;
//         last_alert_seq = 0;
//     }
//     if (incoming_seq > 0 && incoming_seq <= last_alert_seq) {
//         ESP_LOGW(TAG, "ALERT: stale seq=%" PRIu32 " (last=%" PRIu32 ")", incoming_seq, last_alert_seq);
//         httpd_resp_send(req, "stale", 5);
//         return ESP_OK;
//     }
//     last_alert_seq = incoming_seq;
// 
//     if (strcmp(type, (const char *)current_alert_type) == 0) {
//         ESP_LOGI(TAG, "ALERT: heartbeat no-op type='%s' seq=%" PRIu32, type, incoming_seq);
//         httpd_resp_send(req, "ok", 2);
//         return ESP_OK;
//     }
//     if (yawn_beep_active && strcmp(type, "yawn") == 0) {
//         ESP_LOGI(TAG, "ALERT: yawn ignored while yawn_fatigue beeps are active");
//         httpd_resp_send(req, "ok", 2);
//         return ESP_OK;
//     }
// 
//     if (strcmp(type, "fatigue") == 0) {
//         buzzer_held = true;
//         yawn_beep_active = false;
//         led1_on     = true;
//         led2_on     = false;
//         blink_mode  = false;
//         ESP_LOGI(TAG, "ALERT: fatigue — buzzer ON, LED1 ON");
//     } else if (strcmp(type, "yawn_fatigue") == 0) {
//         buzzer_held = false;
//         yawn_beep_active = true;
//         led1_on     = true;
//         led2_on     = false;
//         blink_mode  = false;
//         ESP_LOGI(TAG, "ALERT: yawn_fatigue — three-beep buzzer, LED1 ON, seq=%" PRIu32, incoming_seq);
//     } else if (strcmp(type, "yawn") == 0) {
//         buzzer_held = false;
//         yawn_beep_active = false;
//         led1_on     = false;
//         led2_on     = true;
//         blink_mode  = false;
//         ESP_LOGI(TAG, "ALERT: yawn — LED2 ON");
//     } else if (strcmp(type, "ping") == 0) {
//         ping_blink_count = 6;
//         ESP_LOGI(TAG, "ALERT: ping — blinking Green LED");
//     } else if (strcmp(type, "clear") == 0) {
//         buzzer_held = false;
//         yawn_beep_active = false;
//         led1_on     = false;
//         led2_on     = false;
//         blink_mode  = false;
//         ESP_LOGI(TAG, "ALERT: clear — all off");
//     } else {
//         ESP_LOGW(TAG, "ALERT: unknown type='%s'", type);
//         httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "unknown alert type");
//         return ESP_OK;
//     }
// 
//     strncpy((char *)current_alert_type, type, sizeof(current_alert_type) - 1);
//     current_alert_type[sizeof(current_alert_type) - 1] = '\0';
//     any_alert_active = strcmp(type, "clear") != 0;
//     httpd_resp_send(req, "ok", 2);
//     return ESP_OK;
// }
// 
// static esp_err_t ping_handler(httpd_req_t *req) {
//     ping_blink_count = 6;
//     ESP_LOGI(TAG, "PING received — blinking Green LED");
//     httpd_resp_send(req, "pong", 4);
//     return ESP_OK;
// }
// 
// // ============================================================
// // HTTP Servers
// // ============================================================
// 
// // Port 80 — LED/Buzzer control UI
// static void start_control_server(void) {
//     httpd_handle_t server = NULL;
//     httpd_config_t config = HTTPD_DEFAULT_CONFIG();
//     config.server_port     = 80;
//     config.max_uri_handlers = 14;
// 
//     if (httpd_start(&server, &config) == ESP_OK) {
//         httpd_uri_t uris[] = {
//             { "/",           HTTP_GET, root_get_handler,  NULL },
//             { "/led1/on",    HTTP_GET, led1_on_handler,   NULL },
//             { "/led1/off",   HTTP_GET, led1_off_handler,  NULL },
//             { "/led2/on",    HTTP_GET, led2_on_handler,   NULL },
//             { "/led2/off",   HTTP_GET, led2_off_handler,  NULL },
//             { "/blink/on",   HTTP_GET, blink_on_handler,  NULL },
//             { "/blink/off",  HTTP_GET, blink_off_handler, NULL },
//             { "/buzzer/on",  HTTP_GET, buzzer_on_handler, NULL },
//             { "/buzzer/off", HTTP_GET, buzzer_off_handler, NULL },
//             { "/alert",      HTTP_GET, alert_handler,       NULL },
//             { "/ping",       HTTP_GET, ping_handler,        NULL },
//         };
//         for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
//             httpd_register_uri_handler(server, &uris[i]);
//         }
//         ESP_LOGI(TAG, "Control server started on port 80");
//     }
// }
// 
// // Port 81 — MJPEG camera stream
// static void start_camera_server(void) {
//     httpd_handle_t stream_httpd = NULL;
//     httpd_config_t config = HTTPD_DEFAULT_CONFIG();
//     config.server_port = 81;
//     config.ctrl_port   = 32769; // Must differ from control server's ctrl_port (default 32768)
// 
//     if (httpd_start(&stream_httpd, &config) == ESP_OK) {
//         httpd_uri_t stream_uri = {
//             .uri     = "/stream",
//             .method  = HTTP_GET,
//             .handler = stream_handler,
//             .user_ctx = NULL
//         };
//         httpd_register_uri_handler(stream_httpd, &stream_uri);
//         ESP_LOGI(TAG, "Camera stream server started on port 81 — /stream");
//     }
// }
// 
// // ============================================================
// // app_main
// // ============================================================
// extern "C" void app_main(void) {
//     // 500 ms boot delay for camera power stabilisation
//     vTaskDelay(pdMS_TO_TICKS(500));
// 
//     // --- NVS ---
//     esp_err_t ret = nvs_flash_init();
//     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
//         ESP_ERROR_CHECK(nvs_flash_erase());
//         ret = nvs_flash_init();
//     }
//     ESP_ERROR_CHECK(ret);
// 
//     // --- GPIO: LEDs ---
//     gpio_outputs_init();
// 
//     // --- PWM: Buzzer (LEDC channel 1 / timer 1 — camera owns channel 0 / timer 0) ---
//     buzzer_pwm_init();
// 
//     // --- WiFi AP ---
//     wifi_init_softap();
// 
//     // --- HTTP servers ---
//     start_control_server();  // port 80: LED/buzzer UI
//     start_camera_server();   // port 81: MJPEG stream
// 
//     // --- Camera ---
//     if (!initCamera()) {
//         ESP_LOGE(TAG, "CRITICAL: Camera init failure — halting");
//         // Indicate error via LEDs if desired
//         gpio_set_level(LED1_GPIO, 1);
//         gpio_set_level(LED2_GPIO, 0);
//         while (true) { vTaskDelay(pdMS_TO_TICKS(1000)); }
//     }
// 
//     // --- FreeRTOS tasks ---
//     xTaskCreate(led_task,    "led_task",    2048, NULL, 5, NULL);
//     xTaskCreate(buzzer_task, "buzzer_task", 2048, NULL, 5, NULL);
// 
//     ESP_LOGI(TAG, "Setup complete. AP: FatigueSense_Cam | Control: 192.168.4.1 | Stream: 192.168.4.1:81/stream");
// 
//     // Main loop — all work is done in HTTP callbacks and FreeRTOS tasks
//     while (true) {
//         vTaskDelay(pdMS_TO_TICKS(1000));
//     }
// }
