#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "esp_camera.h"
#include "driver/gpio.h"
#include "camera_pins.h"

// --- Configuration ---
#define WIFI_SSID      "homedev"
#define WIFI_PASS      "54321123aaa"
#define USER_LED_PIN   21

static const char *TAG = "wifi_cam";

// --- Wi-Fi Event Group ---
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

// --- Camera Config ---
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
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_VGA,
    .jpeg_quality = 12,
    .fb_count = 2,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

// --- Web UI HTML ---
static const char* INDEX_HTML = 
"<!DOCTYPE html><html><head><meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
"<title>XIAO ESP32S3 Cam</title>"
"<style>"
"body{font-family:Arial,sans-serif;background:#121212;color:#e0e0e0;margin:0;padding:20px;display:flex;flex-direction:column;align-items:center}"
"h1{color:#00e676} .container{max-width:800px;width:100%;text-align:center}"
"img{width:100%;max-width:640px;border:2px solid #333;border-radius:8px;margin-bottom:20px}"
".controls{display:grid;grid-template-columns:repeat(auto-fit,minmax(120px,1fr));gap:10px;margin-top:20px}"
"button{padding:10px;background:#333;color:#fff;border:1px solid #444;border-radius:4px;cursor:pointer;transition:0.3s}"
"button:hover{background:#00e676;color:#000} .info{margin-top:20px;font-size:0.9em;color:#888}"
"</style></head><body>"
"<div class=\"container\">"
"<h1>XIAO ESP32S3 Live</h1>"
"<img src=\"/stream\" id=\"stream\">"
"<div class=\"controls\">"
"<button onclick=\"setRes(5)\">QVGA (320x240)</button>"
"<button onclick=\"setRes(6)\">VGA (640x480)</button>"
"<button onclick=\"setRes(7)\">SVGA (800x600)</button>"
"<button onclick=\"setRes(8)\">XGA (1024x768)</button>"
"<button onclick=\"setRes(9)\">HD (1280x720)</button>"
"<button onclick=\"setRes(10)\">SXGA (1280x1024)</button>"
"<button onclick=\"setRes(11)\">UXGA (1600x1200)</button>"
"<button onclick=\"setRes(13)\">QXGA (2048x1536)</button>"
"</div>"
"<div class=\"info\">Detected Sensor: <span id=\"sensor\">Loading...</span></div>"
"</div>"
"<script>"
"function setRes(val){fetch('/control?res='+val).then(()=>alert('Resolution Changed'))}"
"fetch('/status').then(r=>r.json()).then(d=>{document.getElementById('sensor').innerText=d.sensor})"
"</script></body></html>";

// --- M-JPEG Streaming Helper ---
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

// --- Handlers ---
esp_err_t index_handler(httpd_req_t *req) {
    return httpd_resp_send(req, INDEX_HTML, strlen(INDEX_HTML));
}

esp_err_t stream_handler(httpd_req_t *req) {
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    char *part_buf[64];
    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if (res != ESP_OK) return res;
    while (true) {
        fb = esp_camera_fb_get();
        if (!fb) { res = ESP_FAIL; } 
        else {
            size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, fb->len);
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
            if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
            if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
            esp_camera_fb_return(fb);
        }
        if (res != ESP_OK) break;
    }
    return res;
}

esp_err_t status_handler(httpd_req_t *req) {
    sensor_t * s = esp_camera_sensor_get();
    char json_response[128];
    const char * model = (s->id.PID == OV3660_PID) ? "OV3660" : (s->id.PID == OV2640_PID ? "OV2640" : "Unknown");
    snprintf(json_response, sizeof(json_response), "{\"sensor\":\"%s\", \"pid\":\"0x%x\"}", model, s->id.PID);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json_response, strlen(json_response));
}

esp_err_t control_handler(httpd_req_t *req) {
    char buf[32];
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        char param[16];
        if (httpd_query_key_value(buf, "res", param, sizeof(param)) == ESP_OK) {
            int res_val = atoi(param);
            sensor_t * s = esp_camera_sensor_get();
            s->set_framesize(s, (framesize_t)res_val);
            ESP_LOGI(TAG, "Changed resolution to %d", res_val);
        }
    }
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, NULL, 0);
}

// --- Start HTTP Server ---
void start_camera_server() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t index_uri = { .uri = "/", .method = HTTP_GET, .handler = index_handler };
        httpd_uri_t stream_uri = { .uri = "/stream", .method = HTTP_GET, .handler = stream_handler };
        httpd_uri_t status_uri = { .uri = "/status", .method = HTTP_GET, .handler = status_handler };
        httpd_uri_t control_uri = { .uri = "/control", .method = HTTP_GET, .handler = control_handler };
        httpd_register_uri_handler(server, &index_uri);
        httpd_register_uri_handler(server, &stream_uri);
        httpd_register_uri_handler(server, &status_uri);
        httpd_register_uri_handler(server, &control_uri);
        ESP_LOGI(TAG, "Web Server started. Open your browser!");
    }
}

// --- Wi-Fi Handler ---
static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init() {
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, NULL));
    wifi_config_t wifi_config = { .sta = { .ssid = WIFI_SSID, .password = WIFI_PASS } };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
}

void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    gpio_reset_pin(USER_LED_PIN);
    gpio_set_direction(USER_LED_PIN, GPIO_MODE_OUTPUT);
    if (esp_camera_init(&camera_config) != ESP_OK) { return; }
    wifi_init();
    start_camera_server();
    while (1) {
        gpio_set_level(USER_LED_PIN, 0);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        gpio_set_level(USER_LED_PIN, 1);
        vTaskDelay(4950 / portTICK_PERIOD_MS);
    }
}
