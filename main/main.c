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
#include "driver/temperature_sensor.h"
#include <wifi_provisioning/manager.h>
#include <wifi_provisioning/scheme_ble.h>

// Forward declarations
static esp_err_t custom_prov_data_handler(uint32_t session_id, const uint8_t *inbuf, ssize_t inlen,
                                          uint8_t **outbuf, ssize_t *outlen, void *priv_data);
static void factory_reset_check();
static void init_temp_sensor();
static void wifi_init();
static void start_camera_server();

// --- Configuration ---
#define USER_LED_PIN   21
static const char *TAG = "wifi_cam";
static temperature_sensor_handle_t temp_sensor = NULL;

// --- Wi-Fi Event Group ---
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static int s_retry_num = 0;
#define MAX_RETRY 5

// --- Camera Config ---
static camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN, .pin_reset = CAM_PIN_RESET, .pin_xclk = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SIOD, .pin_sccb_scl = CAM_PIN_SIOC,
    .pin_d7 = CAM_PIN_D7, .pin_d6 = CAM_PIN_D6, .pin_d5 = CAM_PIN_D5, .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3, .pin_d2 = CAM_PIN_D2, .pin_d1 = CAM_PIN_D1, .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC, .pin_href = CAM_PIN_HREF, .pin_pclk = CAM_PIN_PCLK,
    .xclk_freq_hz = 20000000, .ledc_timer = LEDC_TIMER_0, .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_JPEG, .frame_size = FRAMESIZE_VGA, .jpeg_quality = 12,
    .fb_count = 2, .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
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
"button.active{background:#00e676;color:#000;border-color:#00e676}"
"</style></head><body>"
"<div class=\"container\">"
"<h1>XIAO ESP32S3 Live</h1>"
"<img id=\"stream\">"
"<div class=\"controls\">"
"<button id=\"res5\" onclick=\"setRes(5)\">QVGA (320x240)</button>"
"<button id=\"res6\" onclick=\"setRes(6)\">VGA (640x480)</button>"
"<button id=\"res7\" onclick=\"setRes(7)\">SVGA (800x600)</button>"
"<button id=\"res8\" onclick=\"setRes(8)\">XGA (1024x768)</button>"
"<button id=\"res9\" onclick=\"setRes(9)\">HD (1280x720)</button>"
"<button id=\"res10\" onclick=\"setRes(10)\">SXGA (1280x1024)</button>"
"<button id=\"res11\" onclick=\"setRes(11)\">UXGA (1600x1200)</button>"
"<button id=\"res13\" onclick=\"setRes(13)\">QXGA (2048x1536)</button>"
"</div>"
"<div class=\"info\">Detected Sensor: <span id=\"sensor\">Loading...</span></div>"
"<div class=\"info\">Chip Temp: <span id=\"temp\" style=\"color:#00e676;font-weight:bold\">--</span>°C</div>"
"</div>"
"<script>"
"function setRes(val){"
"localStorage.setItem('xiao_res', val);"
"fetch('/control?res='+val).then(()=>{"
"document.querySelectorAll('button').forEach(b=>b.classList.remove('active'));"
"let btn=document.getElementById('res'+val); if(btn) btn.classList.add('active');"
"})}"
"function updateStatus(){"
"fetch('/status').then(r=>r.json()).then(d=>{"
"document.getElementById('sensor').innerText=d.sensor; document.getElementById('temp').innerText=d.temp;"
"document.querySelectorAll('button').forEach(b=>b.classList.remove('active'));"
"let btn=document.getElementById('res'+d.res); if(btn) btn.classList.add('active');"
"}).catch(e=>console.error('Status fetch failed', e))}"
"document.getElementById('stream').src = window.location.protocol + '//' + window.location.hostname + ':81/stream';"
"let savedRes = localStorage.getItem('xiao_res');"
"if(savedRes) { fetch('/control?res='+savedRes).then(updateStatus); }"
"else { updateStatus(); } setInterval(updateStatus, 3000);"
"</script></body></html>";

// --- HTTP Handlers ---
esp_err_t index_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store, must-revalidate");
    return httpd_resp_send(req, INDEX_HTML, strlen(INDEX_HTML));
}

esp_err_t stream_handler(httpd_req_t *req) {
    if (req->method == HTTP_HEAD) { httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*"); return httpd_resp_send(req, NULL, 0); }
    camera_fb_t *fb = NULL; esp_err_t res = ESP_OK; char part_buf[128];
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    res = httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=123456789000000000000987654321");
    if (res != ESP_OK) return res;
    while (true) {
        fb = esp_camera_fb_get();
        if (!fb) { res = ESP_FAIL; } 
        else {
            size_t hlen = snprintf(part_buf, 128, "\r\n--123456789000000000000987654321\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", fb->len);
            res = httpd_resp_send_chunk(req, part_buf, hlen);
            if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
            esp_camera_fb_return(fb);
        }
        if (res != ESP_OK) break;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return res;
}

esp_err_t status_handler(httpd_req_t *req) {
    sensor_t * s = esp_camera_sensor_get();
    float tsens_out = 0.0;
    temperature_sensor_get_celsius(temp_sensor, &tsens_out);
    char json_response[128];
    const char * model = (s->id.PID == OV3660_PID) ? "OV3660" : (s->id.PID == OV2640_PID ? "OV2640" : "Unknown");
    snprintf(json_response, sizeof(json_response), "{\"sensor\":\"%s\", \"temp\":\"%.1f\", \"res\":%d}", model, tsens_out, s->status.framesize);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json_response, strlen(json_response));
}

esp_err_t control_handler(httpd_req_t *req) {
    char buf[32], param[16];
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        if (httpd_query_key_value(buf, "res", param, sizeof(param)) == ESP_OK) {
            int res_val = atoi(param);
            sensor_t * s = esp_camera_sensor_get();
            s->set_framesize(s, (framesize_t)res_val);
            ESP_LOGI(TAG, "Resolution: %d", res_val);
        }
    }
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, NULL, 0);
}

void start_camera_server() {
    httpd_config_t config_main = HTTPD_DEFAULT_CONFIG(); 
    config_main.server_port = 80; 
    config_main.ctrl_port = 32768; 
    config_main.lru_purge_enable = true;
    httpd_handle_t server = NULL;
    httpd_uri_t index_uri = { .uri = "/", .method = HTTP_GET, .handler = index_handler };
    httpd_uri_t status_uri = { .uri = "/status", .method = HTTP_GET, .handler = status_handler };
    httpd_uri_t control_uri = { .uri = "/control", .method = HTTP_GET, .handler = control_handler };
    if (httpd_start(&server, &config_main) == ESP_OK) {
        httpd_register_uri_handler(server, &index_uri);
        httpd_register_uri_handler(server, &status_uri);
        httpd_register_uri_handler(server, &control_uri);
        ESP_LOGI(TAG, "Server(80) OK");
    }

    httpd_config_t config_stream = HTTPD_DEFAULT_CONFIG();
    config_stream.server_port = 81;
    config_stream.ctrl_port = 32770; // Higher offset to avoid range issues
    config_stream.lru_purge_enable = true;
    httpd_handle_t s_server = NULL;
    httpd_uri_t stream_uri = { .uri = "/stream", .method = HTTP_GET, .handler = stream_handler };
    httpd_uri_t s_head = { .uri = "/stream", .method = HTTP_HEAD, .handler = stream_handler };
    if (httpd_start(&s_server, &config_stream) == ESP_OK) {
        httpd_register_uri_handler(s_server, &stream_uri);
        httpd_register_uri_handler(s_server, &s_head);
        ESP_LOGI(TAG, "Stream(81) OK");
    } else {
        ESP_LOGE(TAG, "Server(81) FAIL. Try different ctrl_port.");
    }
}

// --- Provisioning ---
static esp_err_t custom_prov_data_handler(uint32_t session_id, const uint8_t *inbuf, ssize_t inlen, uint8_t **outbuf, ssize_t *outlen, void *priv_data) {
    esp_netif_ip_info_t ip_info; esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    char *ip_str = "0.0.0.0";
    if (netif && esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) { static char buf[32]; esp_ip4addr_ntoa(&ip_info.ip, buf, 32); ip_str = buf; }
    *outbuf = (uint8_t *)strdup(ip_str); *outlen = strlen(ip_str); return ESP_OK;
}

static void prov_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_PROV_EVENT) {
        switch (event_id) {
            case WIFI_PROV_START:
                wifi_prov_mgr_endpoint_create("xiao-status");
                wifi_prov_mgr_endpoint_register("xiao-status", custom_prov_data_handler, NULL);
                break;
            case WIFI_PROV_CRED_RECV: s_retry_num = 0; break;
            case WIFI_PROV_END: wifi_prov_mgr_deinit(); break;
            default: break;
        }
    }
}

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_id == WIFI_EVENT_STA_START) esp_wifi_connect();
    else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < MAX_RETRY) { esp_wifi_connect(); s_retry_num++; }
        else xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
    } else if (event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init() {
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init()); ESP_ERROR_CHECK(esp_event_loop_create_default()); esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT(); ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_PROV_EVENT, ESP_EVENT_ANY_ID, &prov_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));
    wifi_prov_mgr_config_t p_cfg = { .scheme = wifi_prov_scheme_ble, .scheme_event_handler = WIFI_PROV_SCHEME_BLE_EVENT_HANDLER_FREE_BTDM };
    ESP_ERROR_CHECK(wifi_prov_mgr_init(p_cfg));
    bool provisioned = false; wifi_prov_mgr_is_provisioned(&provisioned);
    if (!provisioned) {
        ESP_LOGI(TAG, "Starting XIAO_CAM_PROV...");
        ESP_ERROR_CHECK(wifi_prov_mgr_start_provisioning(WIFI_PROV_SECURITY_0, NULL, "XIAO_CAM_PROV", NULL));
        xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    } else {
        wifi_prov_mgr_deinit(); ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA)); ESP_ERROR_CHECK(esp_wifi_start());
        EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
        if (bits & WIFI_FAIL_BIT) { ESP_LOGW(TAG, "Resetting NVS..."); nvs_flash_erase(); esp_restart(); }
    }
}

static void factory_reset_check() {
    gpio_config_t io_conf = { .pin_bit_mask = (1ULL << GPIO_NUM_0), .mode = GPIO_MODE_INPUT, .pull_up_en = GPIO_PULLUP_ENABLE }; 
    gpio_config(&io_conf);
    if (gpio_get_level(GPIO_NUM_0) == 0) {
        ESP_LOGI(TAG, "RESET START: Hold BOOT for 3 seconds...");
        int count = 0; 
        while (count < 30) { 
            if (gpio_get_level(GPIO_NUM_0) != 0) { ESP_LOGI(TAG, "Aborted."); return; }
            vTaskDelay(pdMS_TO_TICKS(100)); 
            count++; 
            if (count % 10 == 0) ESP_LOGI(TAG, "Holding... %ds", count/10);
        }
        ESP_LOGW(TAG, "FACTORY RESET TRIGGERED!");
        nvs_flash_erase(); esp_restart();
    }
}

static void init_temp_sensor() {
    temperature_sensor_config_t t_cfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
    ESP_ERROR_CHECK(temperature_sensor_install(&t_cfg, &temp_sensor)); ESP_ERROR_CHECK(temperature_sensor_enable(temp_sensor));
}

void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init()); 
    gpio_reset_pin(USER_LED_PIN); gpio_set_direction(USER_LED_PIN, GPIO_MODE_OUTPUT);
    ESP_LOGI(TAG, "Wait 1s for manual reset trigger..."); vTaskDelay(pdMS_TO_TICKS(1000));
    factory_reset_check();
    init_temp_sensor();
    if (esp_camera_init(&camera_config) != ESP_OK) { ESP_LOGE(TAG, "Cam fail"); return; }
    sensor_t * s = esp_camera_sensor_get(); s->set_framesize(s, FRAMESIZE_VGA);
    wifi_init(); start_camera_server();
    while (1) { gpio_set_level(USER_LED_PIN, 0); vTaskDelay(50); gpio_set_level(USER_LED_PIN, 1); vTaskDelay(4950); }
}
