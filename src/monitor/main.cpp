/**
 * monitor/main.cpp — ESP32-WROOM-32 (TempMonitor v2)
 * 5× MAX31855, hardware SPI VSPI | Nextion 320×480, UART2
 * Noise filter: median(5) + EMA(α=0.3) | Serial JSON → USB
 * WiFi config via SoftAP captive portal (192.168.4.1)
 *
 * GPIO:
 *   SPI VSPI : SCK=18  MISO=19
 *   Sensors  : CS T1=32 T2=33 T3=25 T4=26 T5=27
 *   Nextion  : UART2 TX=17  RX=16  (5 V from VIN)
 *
 * Nextion protocol (matches display_test.py):
 *   ESP32 → Nextion numeric var : "var_name=N\xFF\xFF\xFF"   (NO .val)
 *   ESP32 → Nextion text comp   : "comp.txt=\"...\"\xFF\xFF\xFF"
 *   Nextion → ESP32             : ASCII string + \xFF\xFF\xFF
 *     WIFI_CFG | LOG_CLEAR | REFRESH_250/500/1000
 *     TOGGLE_CF | RESET_MINMAX | HEATER_ON | HEATER_OFF
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "nvs_flash.h"

static const char* TAG = "Monitor";

// ── SPI (VSPI = SPI3_HOST) ────────────────────────────────────
#define SPI_PIN_CLK   GPIO_NUM_18
#define SPI_PIN_MISO  GPIO_NUM_19

static const gpio_num_t CS_PINS[5] = {
    GPIO_NUM_32, GPIO_NUM_33, GPIO_NUM_25, GPIO_NUM_26, GPIO_NUM_27
};

static spi_device_handle_t spi_bus;

// ── Nextion UART2 ─────────────────────────────────────────────
#define NXT_UART       UART_NUM_2
#define NXT_TX         GPIO_NUM_17
#define NXT_RX         GPIO_NUM_16
#define NXT_BAUD_SLOW  9600
#define NXT_BAUD_FAST  115200
#define NXT_RX_BUF     512
#define NXT_TX_BUF     512

// ── Фильтр: медиана(5) + EMA ──────────────────────────────────
#define FILTER_LEN  5
#define EMA_ALPHA   0.3f

static float raw_buf[5][FILTER_LEN];
static int   buf_pos[5];
static float ema_val[5];
static bool  ema_init[5];

static float median5(float a[5]) {
    float s[5];
    memcpy(s, a, sizeof(s));
    for (int i = 1; i < 5; i++) {
        float k = s[i]; int j = i - 1;
        while (j >= 0 && s[j] > k) { s[j+1] = s[j]; j--; }
        s[j+1] = k;
    }
    return s[2];
}

static float filter_update(int ch, float raw) {
    if (!ema_init[ch]) {
        for (int j = 0; j < FILTER_LEN; j++) raw_buf[ch][j] = raw;
        ema_val[ch]  = raw;
        ema_init[ch] = true;
        buf_pos[ch]  = 0;
        return raw;
    }
    raw_buf[ch][buf_pos[ch]] = raw;
    buf_pos[ch] = (buf_pos[ch] + 1) % FILTER_LEN;
    float med = median5(raw_buf[ch]);
    ema_val[ch] = EMA_ALPHA * med + (1.0f - EMA_ALPHA) * ema_val[ch];
    return ema_val[ch];
}

// ── Состояние ─────────────────────────────────────────────────
static bool  unit_fahrenheit = false;
static float t_min[5], t_max[5], t_prev[5];
static int   heater_status = 0;
static int   refresh_ms    = 500;

// Лог ошибок (хранится на ESP32, как в шаблоне)
#define LOG_LINES 10
static char log_entries[LOG_LINES][64];
static int  log_count = 0;

// Запрос на WiFi-подключение из HTTP-обработчика
static struct {
    char ssid[64];
    char pass[64];
    volatile bool pending;
} wifi_req = {};

static inline float to_disp(float c) {
    return (!unit_fahrenheit || isnan(c)) ? c : c * 1.8f + 32.0f;
}

static int append_float(char* buf, int pos, float v) {
    if (isnan(v) || isinf(v)) return pos + sprintf(buf + pos, "null");
    return pos + sprintf(buf + pos, "%.2f", (double)v);
}

// ── SPI / MAX31855 ────────────────────────────────────────────
static void spi_init(void) {
    gpio_config_t io = {};
    io.mode         = GPIO_MODE_OUTPUT;
    io.pull_up_en   = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type    = GPIO_INTR_DISABLE;
    io.pin_bit_mask = 0;
    for (int i = 0; i < 5; i++) io.pin_bit_mask |= (1ULL << CS_PINS[i]);
    ESP_ERROR_CHECK(gpio_config(&io));
    for (int i = 0; i < 5; i++) gpio_set_level(CS_PINS[i], 1);

    spi_bus_config_t bus = {};
    bus.mosi_io_num   = -1;
    bus.miso_io_num   = SPI_PIN_MISO;
    bus.sclk_io_num   = SPI_PIN_CLK;
    bus.quadwp_io_num = -1;
    bus.quadhd_io_num = -1;
    bus.max_transfer_sz = 4;
    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &bus, SPI_DMA_DISABLED));

    spi_device_interface_config_t dev = {};
    dev.mode           = 0;
    dev.clock_speed_hz = 4 * 1000 * 1000;
    dev.spics_io_num   = -1;
    dev.queue_size     = 1;
    ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &dev, &spi_bus));
}

static uint32_t max31855_read_raw(int idx) {
    uint8_t rx[4] = {};
    spi_transaction_t t = {};
    t.length    = 32;
    t.rx_buffer = rx;
    gpio_set_level(CS_PINS[idx], 0);
    ESP_ERROR_CHECK(spi_device_transmit(spi_bus, &t));
    gpio_set_level(CS_PINS[idx], 1);
    return ((uint32_t)rx[0] << 24) | ((uint32_t)rx[1] << 16) |
           ((uint32_t)rx[2] << 8)  |  rx[3];
}

static float max31855_parse(uint32_t raw) {
    if (raw & 0x7) return NAN;
    int16_t t14 = (int16_t)((raw >> 18) & 0x3FFF);
    if (t14 & 0x2000) t14 |= (int16_t)0xC000;
    return t14 * 0.25f;
}

// ── Nextion: отправка ─────────────────────────────────────────
static const uint8_t NXT_TERM[3] = {0xFF, 0xFF, 0xFF};

static void nxt_cmd(const char* cmd) {
    uart_write_bytes(NXT_UART, cmd, strlen(cmd));
    uart_write_bytes(NXT_UART, (const char*)NXT_TERM, 3);
}

// Числовая глобальная переменная: "var_name=N"  (БЕЗ .val — как в display_test.py)
static void nxt_set_var(const char* name, int val) {
    char buf[64];
    snprintf(buf, sizeof(buf), "%s=%d", name, val);
    nxt_cmd(buf);
}

// Текстовый компонент: "comp.txt=\"...\""
static void nxt_set_txt(const char* comp, const char* txt) {
    char buf[128];
    snprintf(buf, sizeof(buf), "%s.txt=\"%s\"", comp, txt);
    nxt_cmd(buf);
}

static void nxt_init(void) {
    // Старт на 9600 (Nextion по умолчанию), затем переключаем на 115200
    uart_config_t cfg = {};
    cfg.baud_rate  = NXT_BAUD_SLOW;
    cfg.data_bits  = UART_DATA_8_BITS;
    cfg.parity     = UART_PARITY_DISABLE;
    cfg.stop_bits  = UART_STOP_BITS_1;
    cfg.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_APB;
    ESP_ERROR_CHECK(uart_driver_install(NXT_UART, NXT_RX_BUF, NXT_TX_BUF, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(NXT_UART, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(NXT_UART, NXT_TX, NXT_RX,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    vTaskDelay(pdMS_TO_TICKS(500));    // ждём загрузку Nextion
    nxt_cmd("bkcmd=0");                // отключить return codes
    nxt_cmd("bauds=115200");           // переключить Nextion на 115200
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_ERROR_CHECK(uart_set_baudrate(NXT_UART, NXT_BAUD_FAST));
    vTaskDelay(pdMS_TO_TICKS(50));
    nxt_cmd("page 1");
}

// ── Nextion: отправка данных датчиков ─────────────────────────
static void nxt_send_sensors(const float temps[5]) {
    char name[20];
    for (int i = 0; i < 5; i++) {
        int n = i + 1;

        snprintf(name, sizeof(name), "t%d_val", n);
        nxt_set_var(name, isnan(temps[i]) ? -1 : (int)roundf(to_disp(temps[i])));

        if (!isinf(t_min[i])) {
            snprintf(name, sizeof(name), "t%d_min", n);
            nxt_set_var(name, (int)roundf(to_disp(t_min[i])));
        }
        if (!isinf(t_max[i])) {
            snprintf(name, sizeof(name), "t%d_max", n);
            nxt_set_var(name, (int)roundf(to_disp(t_max[i])));
        }
        if (!isnan(temps[i]) && !isnan(t_prev[i])) {
            snprintf(name, sizeof(name), "t%d_rate", n);
            nxt_set_var(name, (int)roundf((temps[i] - t_prev[i]) * (1000.0f / refresh_ms)));
        }
    }
    nxt_set_var("heater_status", heater_status);
}

// ── Nextion: системная информация ─────────────────────────────
static void nxt_send_system_info(void) {
    nxt_set_txt("espStatus", "Connected");

    char mac[24] = {};
    uint8_t m[6] = {};
    if (esp_wifi_get_mac(WIFI_IF_STA, m) == ESP_OK) {
        snprintf(mac, sizeof(mac), "MAC:%02X:%02X:%02X:%02X:%02X:%02X",
                 m[0], m[1], m[2], m[3], m[4], m[5]);
        nxt_set_txt("macAddr", mac);
    }
}

// ── Nextion: лог ошибок ───────────────────────────────────────
static void nxt_send_log(void) {
    char comp[16];
    for (int i = 0; i < LOG_LINES; i++) {
        snprintf(comp, sizeof(comp), "log%d", i + 1);
        nxt_set_txt(comp, log_entries[i]);
    }
    nxt_set_var("errCount", log_count);
    nxt_cmd("cov errCount,errCnt.txt,0");
}

static void add_log_entry(const char* entry) {
    for (int i = LOG_LINES - 1; i > 0; i--)
        memcpy(log_entries[i], log_entries[i-1], sizeof(log_entries[0]));
    snprintf(log_entries[0], sizeof(log_entries[0]), "%s", entry);
    log_count++;
    nxt_send_log();
}

static void clear_log(void) {
    for (int i = 0; i < LOG_LINES; i++) log_entries[i][0] = '\0';
    log_count = 0;
    nxt_send_log();
}

// ── WiFi ──────────────────────────────────────────────────────
#define WIFI_AP_SSID   "ThermalHMI_Setup"
#define NVS_NAMESPACE  "wifi_cfg"

static httpd_handle_t http_server = NULL;

static const char* HTML_CONFIG =
    "<!DOCTYPE html><html><head><meta charset='UTF-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>WiFi Setup</title>"
    "<style>body{font-family:sans-serif;max-width:380px;margin:40px auto;padding:0 16px}"
    "input{width:100%;padding:8px;margin:6px 0 14px;box-sizing:border-box}"
    "button{width:100%;padding:12px;background:#0066cc;color:#fff;"
    "border:none;border-radius:4px;font-size:16px;cursor:pointer}</style></head>"
    "<body><h2>ThermalHMI WiFi Setup</h2>"
    "<form method='POST' action='/connect'>"
    "<label>Network (SSID)</label>"
    "<input name='ssid' required placeholder='WiFi name'>"
    "<label>Password</label>"
    "<input name='pass' type='password' placeholder='Password (leave blank if open)'>"
    "<button type='submit'>Connect</button>"
    "</form></body></html>";

static const char* HTML_OK =
    "<!DOCTYPE html><html><head><meta charset='UTF-8'><title>OK</title></head>"
    "<body style='font-family:sans-serif;text-align:center;padding:40px'>"
    "<h2>Connecting...</h2><p>Check the display for status.</p></body></html>";

// URL-decode одного поля из application/x-www-form-urlencoded
static void url_decode_field(const char* body, const char* key,
                             char* out, int out_len) {
    char search[40];
    snprintf(search, sizeof(search), "%s=", key);
    const char* p = strstr(body, search);
    if (!p) { out[0] = '\0'; return; }
    p += strlen(search);
    int i = 0;
    while (*p && *p != '&' && i < out_len - 1) {
        if (*p == '+') { out[i++] = ' '; p++; }
        else if (*p == '%' && p[1] && p[2]) {
            char h[3] = {p[1], p[2], 0};
            out[i++] = (char)strtol(h, NULL, 16);
            p += 3;
        } else {
            out[i++] = *p++;
        }
    }
    out[i] = '\0';
}

static esp_err_t http_get_handler(httpd_req_t* req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, HTML_CONFIG, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t http_post_handler(httpd_req_t* req) {
    char body[256] = {};
    int  got = httpd_req_recv(req, body, sizeof(body) - 1);
    if (got > 0) body[got] = '\0';

    // Сначала ответить, потом переключать WiFi
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, HTML_OK, HTTPD_RESP_USE_STRLEN);

    if (got > 0) {
        url_decode_field(body, "ssid", wifi_req.ssid, sizeof(wifi_req.ssid));
        url_decode_field(body, "pass", wifi_req.pass, sizeof(wifi_req.pass));
        if (strlen(wifi_req.ssid) > 0)
            wifi_req.pending = true;  // main_task выполнит подключение
    }
    return ESP_OK;
}

static void start_http_server(void) {
    if (http_server) return;
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    if (httpd_start(&http_server, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "HTTP server start failed");
        return;
    }
    httpd_uri_t get_uri  = {"/",        HTTP_GET,  http_get_handler,  NULL};
    httpd_uri_t post_uri = {"/connect", HTTP_POST, http_post_handler, NULL};
    httpd_register_uri_handler(http_server, &get_uri);
    httpd_register_uri_handler(http_server, &post_uri);
    ESP_LOGI(TAG, "HTTP server at 192.168.4.1");
}

static void stop_http_server(void) {
    if (!http_server) return;
    httpd_stop(http_server);
    http_server = NULL;
}

static void wifi_event_handler(void*, esp_event_base_t base,
                               int32_t id, void* data) {
    if (base == WIFI_EVENT) {
        if (id == WIFI_EVENT_AP_START) {
            ESP_LOGI(TAG, "AP started: %s", WIFI_AP_SSID);
            start_http_server();
        } else if (id == WIFI_EVENT_STA_DISCONNECTED) {
            ESP_LOGW(TAG, "WiFi disconnected");
            nxt_set_txt("wifiStatus", "Disconnected");
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* ev = (ip_event_got_ip_t*)data;
        char ip_str[20], ssid_str[33] = {};
        snprintf(ip_str, sizeof(ip_str), "IP:" IPSTR, IP2STR(&ev->ip_info.ip));
        wifi_ap_record_t ap;
        if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK)
            strlcpy(ssid_str, (char*)ap.ssid, sizeof(ssid_str));
        ESP_LOGI(TAG, "WiFi connected: %s %s", ssid_str, ip_str);
        nxt_set_txt("wifiStatus", "Connected");
        nxt_set_txt("ssidVal",    ssid_str);
        nxt_set_txt("ipAddr",     ip_str);
        stop_http_server();
    }
}

static void wifi_init(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL, NULL));

    // Попробовать auto-connect из NVS
    char ssid[64] = {}, pass[64] = {};
    bool has_creds = false;
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) == ESP_OK) {
        size_t sl = sizeof(ssid), pl = sizeof(pass);
        has_creds = (nvs_get_str(nvs, "ssid", ssid, &sl) == ESP_OK &&
                     nvs_get_str(nvs, "pass", pass, &pl) == ESP_OK && sl > 1);
        nvs_close(nvs);
    }

    if (has_creds) {
        wifi_config_t sta_cfg = {};
        strlcpy((char*)sta_cfg.sta.ssid,     ssid, sizeof(sta_cfg.sta.ssid));
        strlcpy((char*)sta_cfg.sta.password, pass, sizeof(sta_cfg.sta.password));
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_cfg));
        ESP_ERROR_CHECK(esp_wifi_start());
        ESP_ERROR_CHECK(esp_wifi_connect());
        ESP_LOGI(TAG, "Auto-connecting to '%s'", ssid);
    } else {
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
        ESP_ERROR_CHECK(esp_wifi_start());
    }
}

// Запустить SoftAP для настройки WiFi
static void start_wifi_config(void) {
    ESP_LOGI(TAG, "Starting WiFi config AP");
    nxt_set_txt("espStatus",  "Config...");
    nxt_set_txt("wifiStatus", "Config...");

    stop_http_server();
    esp_wifi_stop();
    vTaskDelay(pdMS_TO_TICKS(300));

    wifi_config_t ap_cfg = {};
    strlcpy((char*)ap_cfg.ap.ssid, WIFI_AP_SSID, sizeof(ap_cfg.ap.ssid));
    ap_cfg.ap.ssid_len       = strlen(WIFI_AP_SSID);
    ap_cfg.ap.authmode       = WIFI_AUTH_OPEN;
    ap_cfg.ap.max_connection = 4;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    // start_http_server вызывается из WIFI_EVENT_AP_START
}

// Подключиться к WiFi (вызывается из main_task, безопасно)
static void do_wifi_connect(const char* ssid, const char* pass) {
    // Сохранить в NVS
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_set_str(nvs, "ssid", ssid);
        nvs_set_str(nvs, "pass", pass);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
    ESP_LOGI(TAG, "Connecting to '%s'", ssid);
    nxt_set_txt("espStatus",  "Connecting...");
    nxt_set_txt("wifiStatus", "Connecting...");

    stop_http_server();
    esp_wifi_stop();
    vTaskDelay(pdMS_TO_TICKS(300));

    wifi_config_t sta_cfg = {};
    strlcpy((char*)sta_cfg.sta.ssid,     ssid, sizeof(sta_cfg.sta.ssid));
    strlcpy((char*)sta_cfg.sta.password, pass, sizeof(sta_cfg.sta.password));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());
}

// ── Nextion: приём команд ─────────────────────────────────────
static char nxt_rx_acc[256];
static int  nxt_rx_acc_len = 0;

static void nxt_handle_command(const char* cmd) {
    ESP_LOGI(TAG, "NXT: '%s'", cmd);
    if      (strcmp(cmd, "WIFI_CFG")     == 0) start_wifi_config();
    else if (strcmp(cmd, "LOG_CLEAR")    == 0) clear_log();
    else if (strcmp(cmd, "REFRESH_250")  == 0) refresh_ms = 250;
    else if (strcmp(cmd, "REFRESH_500")  == 0) refresh_ms = 500;
    else if (strcmp(cmd, "REFRESH_1000") == 0) refresh_ms = 1000;
    else if (strcmp(cmd, "TOGGLE_CF")    == 0) unit_fahrenheit = !unit_fahrenheit;
    else if (strcmp(cmd, "RESET_MINMAX") == 0) {
        for (int i = 0; i < 5; i++) { t_min[i] = INFINITY; t_max[i] = -INFINITY; }
    }
    else if (strcmp(cmd, "HEATER_ON")  == 0) heater_status = 1;
    else if (strcmp(cmd, "HEATER_OFF") == 0) heater_status = 0;
}

static void nxt_process_rx(void) {
    uint8_t tmp[128];
    int len = uart_read_bytes(NXT_UART, tmp, sizeof(tmp), pdMS_TO_TICKS(0));
    for (int i = 0; i < len; i++) {
        if (nxt_rx_acc_len < (int)sizeof(nxt_rx_acc) - 1)
            nxt_rx_acc[nxt_rx_acc_len++] = (char)tmp[i];

        if (nxt_rx_acc_len >= 3 &&
            (uint8_t)nxt_rx_acc[nxt_rx_acc_len-3] == 0xFF &&
            (uint8_t)nxt_rx_acc[nxt_rx_acc_len-2] == 0xFF &&
            (uint8_t)nxt_rx_acc[nxt_rx_acc_len-1] == 0xFF) {

            nxt_rx_acc[nxt_rx_acc_len - 3] = '\0';
            const char* cmd = nxt_rx_acc;
            while (*cmd != '\0' && (uint8_t)*cmd < 0x20) cmd++;
            if (*cmd != '\0') nxt_handle_command(cmd);
            nxt_rx_acc_len = 0;
        }
    }
}

// ── Основная задача ───────────────────────────────────────────
static void main_task(void* /*arg*/) {
    float temps[5];
    static char json_line[256];
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        int64_t now_ms = esp_timer_get_time() / 1000LL;

        // 1. Датчики
        uint8_t err_mask = 0;
        for (int i = 0; i < 5; i++) {
            float raw = max31855_parse(max31855_read_raw(i));
            if (isnan(raw)) {
                err_mask |= (uint8_t)(1 << i);
                temps[i]  = NAN;
            } else {
                temps[i] = filter_update(i, raw);
                if (temps[i] < t_min[i]) t_min[i] = temps[i];
                if (temps[i] > t_max[i]) t_max[i] = temps[i];
            }
        }

        // 2. Nextion
        nxt_process_rx();
        nxt_send_sensors(temps);

        // 3. Обработать отложенный запрос WiFi-подключения (из HTTP POST)
        if (wifi_req.pending) {
            wifi_req.pending = false;
            do_wifi_connect(wifi_req.ssid, wifi_req.pass);
        }

        // 4. rate для следующего цикла
        for (int i = 0; i < 5; i++) t_prev[i] = temps[i];

        // 5. JSON → UART0 (USB)
        int p = 0;
        p += sprintf(json_line + p, "{\"timestamp\":%lld,\"top\":[", (long long)now_ms);
        for (int i = 0; i < 4; i++) {
            if (i > 0) json_line[p++] = ',';
            p = append_float(json_line, p, temps[i]);
        }
        p += sprintf(json_line + p, "],\"bottom\":[");
        p = append_float(json_line, p, temps[4]);
        p += sprintf(json_line + p, "],\"status\":\"%s\"}\n",
                     err_mask ? "SENSOR_ERR" : "OK");
        fputs(json_line, stdout);
        fflush(stdout);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(200));
    }
}

// ── app_main ──────────────────────────────────────────────────
extern "C" void app_main(void) {
    vTaskDelay(pdMS_TO_TICKS(1500));
    ESP_LOGI(TAG, "=== TempMonitor v2 (ESP32-WROOM-32) ===");

    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    for (int i = 0; i < 5; i++) {
        t_min[i]    =  INFINITY;
        t_max[i]    = -INFINITY;
        t_prev[i]   = NAN;
        ema_init[i] = false;
        buf_pos[i]  = 0;
    }
    for (int i = 0; i < LOG_LINES; i++) log_entries[i][0] = '\0';

    spi_init();
    ESP_LOGI(TAG, "Sensors SPI OK");

    nxt_init();
    ESP_LOGI(TAG, "Nextion UART2 OK");

    wifi_init();
    ESP_LOGI(TAG, "WiFi init OK");

    nxt_send_system_info();

    xTaskCreate(main_task, "main_task", 8192, nullptr, 5, nullptr);
    ESP_LOGI(TAG, "Main task @ 5 Hz");
}
