/**
 * Node Top Firmware  —  ESP32-C6
 * 4× MAX31855 (soft-SPI) | LCD 1.47" ST7789 landscape via LGFX_Sprite
 * ESP-NOW RX (node_bottom data) + TX (wake commands) | Serial JSON → USB-CDC → PC
 *
 * Waveshare ESP32-C6-LCD-1.47 pinout (verified):
 *   LCD:     MOSI=6   SCLK=7   CS=14  DC=15  RST=21  BL=22
 *   Sensors: CLK=5    MISO=9   CS1=18 CS2=19 CS3=20  CS4=23
 *   Buttons: BTN0=0   BTN1=1   BTN2=2  BTN3=3  (INPUT_PULLUP)
 *
 * Button actions:
 *   BTN0 — cycle pages (Grid / Stats)
 *   BTN1 — toggle °C / °F
 *   BTN2 — backlight brightness  100% → 50% → 15% → 100%
 *   BTN3 — reset MIN / MAX history
 *   Any  — wake display + broadcast wake command to node_bottom
 *
 * Display sleep:
 *   Auto-sleep after DISPLAY_SLEEP_TIMEOUT_MS of button inactivity.
 *   Any button press wakes this display and broadcasts NODE_CMD_WAKE_DISPLAY.
 *
 * Data flow:
 *   node_bottom → ESP-NOW → [recv_cb] ─┐
 *   own sensors  → soft-SPI ────────────┴→ JSON → USB-CDC → PC
 *
 * JSON output (every 200 ms):
 *   {"timestamp":12345,"top":[T1,T2,T3,T4],"bottom":[T5],"status":"OK"}
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "lgfx_config.h"
#include "espnow_packet.h"

static const char* TAG = "NODE_TOP";

// ── Pin definitions ──────────────────────────────────────────
#define SENS_CLK    GPIO_NUM_5
#define SENS_MISO   GPIO_NUM_9
// GPIO21 = LCD RST → T4 CS moved to GPIO23
static const gpio_num_t CS_PINS[4] = {
    GPIO_NUM_18, GPIO_NUM_19, GPIO_NUM_20, GPIO_NUM_23
};
#define LCD_BL_PIN   GPIO_NUM_22
#define LCD_RST_PIN  GPIO_NUM_21

#define BTN0_PIN     GPIO_NUM_0
#define BTN1_PIN     GPIO_NUM_1
#define BTN2_PIN     GPIO_NUM_2
#define BTN3_PIN     GPIO_NUM_3

// ── Backlight (LEDC PWM on BL_PIN) ──────────────────────────
#define BL_LEDC_TIMER   LEDC_TIMER_0
#define BL_LEDC_CHANNEL LEDC_CHANNEL_0
#define BL_LEDC_FREQ_HZ 5000u

static const uint8_t BL_LEVELS[]  = {255, 128, 38};   // 100% / 50% / 15%
static const int     BL_NUM_STEPS = (int)(sizeof(BL_LEVELS));

// ── Display geometry ─────────────────────────────────────────
#define SCREEN_W  320
#define SCREEN_H  172
#define ZONE_W    (SCREEN_W / 2)
#define ZONE_H    (SCREEN_H / 2)

#define STATS_ROW_H    34
#define STATS_COL0_CX  20
#define STATS_COL1_CX  96
#define STATS_COL2_CX  193
#define STATS_COL3_CX  277
#define STATS_DIV1     39
#define STATS_DIV2     152
#define STATS_DIV3     233

// ── Timeouts ─────────────────────────────────────────────────
#define NODE_TIMEOUT_MS          1000    // node_bottom считается потерянным
#define DISPLAY_SLEEP_TIMEOUT_MS 30000   // 30 с без кнопок → дисплей выкл

static const char* const SENSOR_LABELS[4] = {"T1", "T2", "T3", "T4"};

// Broadcast MAC для отправки wake-команды node_bottom
static const uint8_t BROADCAST_MAC[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// ── Runtime state ────────────────────────────────────────────
static portMUX_TYPE      btn_mux        = portMUX_INITIALIZER_UNLOCKED;
static volatile uint8_t  btn_flags      = 0;
static int64_t           btn_last_us[4] = {};

static uint8_t  display_page    = 0;
static bool     unit_fahrenheit = false;
static int      bl_idx          = 0;

static float    t_min[4];
static float    t_max[4];

static bool     display_sleeping = false;
static int64_t  last_activity_ms = 0;

// ── Node_bottom shared state ─────────────────────────────────
static SemaphoreHandle_t   s_bot_mutex;
static espnow_packet_t     s_bot_pkt;
static int64_t             s_bot_rx_ms = 0;
static bool                s_bot_valid = false;

// ── LovyanGFX objects ────────────────────────────────────────
static LGFX        lcd;
static LGFX_Sprite canvas(&lcd);

// ── Helpers ──────────────────────────────────────────────────
static inline float to_display(float t_c)
{
    if (isnan(t_c)) return NAN;
    return unit_fahrenheit ? (t_c * 1.8f + 32.0f) : t_c;
}

static int append_float(char* buf, int pos, float v)
{
    if (isnan(v) || isinf(v))
        return pos + sprintf(buf + pos, "null");
    return pos + sprintf(buf + pos, "%.2f", (double)v);
}

// ── ESP-NOW receive callback ──────────────────────────────────
static void espnow_recv_cb(const esp_now_recv_info_t* info,
                           const uint8_t* data, int len)
{
    (void)info;
    if (len < (int)sizeof(espnow_packet_t)) return;
    const espnow_packet_t* pkt = (const espnow_packet_t*)data;
    if (pkt->node_id != NODE_ID_BOTTOM) return;

    int64_t now_ms = esp_timer_get_time() / 1000LL;
    if (xSemaphoreTake(s_bot_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        s_bot_pkt    = *pkt;
        s_bot_rx_ms  = now_ms;
        s_bot_valid  = true;
        xSemaphoreGive(s_bot_mutex);
    }
}

// ── Button ISR ───────────────────────────────────────────────
static void IRAM_ATTR btn_isr_handler(void* arg)
{
    const int btn = (int)(intptr_t)arg;
    static const gpio_num_t PINS[] = {BTN0_PIN, BTN1_PIN, BTN2_PIN, BTN3_PIN};

    int64_t now_us = esp_timer_get_time();
    if (now_us - btn_last_us[btn] < 50000LL) return;
    if (gpio_get_level(PINS[btn]) != 0) return;

    btn_last_us[btn] = now_us;
    portENTER_CRITICAL_ISR(&btn_mux);
    btn_flags |= (uint8_t)(1u << btn);
    portEXIT_CRITICAL_ISR(&btn_mux);
}

// ── Soft-SPI MAX31855 ────────────────────────────────────────
static void spi_sens_init(void)
{
    gpio_config_t io = {};
    io.mode         = GPIO_MODE_OUTPUT;
    io.pull_up_en   = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type    = GPIO_INTR_DISABLE;
    io.pin_bit_mask = (1ULL << SENS_CLK);
    for (int i = 0; i < 4; i++) io.pin_bit_mask |= (1ULL << CS_PINS[i]);
    gpio_config(&io);

    io.mode         = GPIO_MODE_INPUT;
    io.pull_up_en   = GPIO_PULLUP_ENABLE;
    io.pin_bit_mask = (1ULL << SENS_MISO);
    gpio_config(&io);

    gpio_set_level(SENS_CLK, 0);
    for (int i = 0; i < 4; i++) gpio_set_level(CS_PINS[i], 1);
}

static uint32_t max31855_read_raw(gpio_num_t cs)
{
    uint32_t data = 0;
    gpio_set_level(cs, 0);
    esp_rom_delay_us(2);
    for (int i = 31; i >= 0; i--) {
        gpio_set_level(SENS_CLK, 0);
        esp_rom_delay_us(1);
        if (gpio_get_level(SENS_MISO)) data |= (1UL << i);
        gpio_set_level(SENS_CLK, 1);
        esp_rom_delay_us(1);
    }
    gpio_set_level(SENS_CLK, 0);
    gpio_set_level(cs, 1);
    return data;
}

static float max31855_parse(uint32_t raw)
{
    if (raw & 0x7) return NAN;
    int16_t t14 = (int16_t)((raw >> 18) & 0x3FFF);
    if (t14 & 0x2000) t14 |= (int16_t)0xC000;
    return t14 * 0.25f;
}

// ── Backlight (LEDC PWM) ─────────────────────────────────────
static void bl_init(void)
{
    ledc_timer_config_t timer = {};
    timer.speed_mode      = LEDC_LOW_SPEED_MODE;
    timer.duty_resolution = LEDC_TIMER_8_BIT;
    timer.timer_num       = BL_LEDC_TIMER;
    timer.freq_hz         = BL_LEDC_FREQ_HZ;
    timer.clk_cfg         = LEDC_AUTO_CLK;
    ledc_timer_config(&timer);

    ledc_channel_config_t ch = {};
    ch.gpio_num   = LCD_BL_PIN;
    ch.speed_mode = LEDC_LOW_SPEED_MODE;
    ch.channel    = BL_LEDC_CHANNEL;
    ch.intr_type  = LEDC_INTR_DISABLE;
    ch.timer_sel  = BL_LEDC_TIMER;
    ch.duty       = BL_LEVELS[0];
    ch.hpoint     = 0;
    ledc_channel_config(&ch);
}

static void bl_set(uint8_t level)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, BL_LEDC_CHANNEL, level);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, BL_LEDC_CHANNEL);
}

// ── Display sleep / wake ─────────────────────────────────────
static void display_sleep_start(void)
{
    display_sleeping = true;
    bl_set(0);
}

static void display_wake(void)
{
    display_sleeping = false;
    bl_set(BL_LEVELS[bl_idx]);
}

// ── Button GPIO init ─────────────────────────────────────────
static void buttons_init(void)
{
    gpio_config_t io = {};
    io.mode         = GPIO_MODE_INPUT;
    io.pull_up_en   = GPIO_PULLUP_ENABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type    = GPIO_INTR_NEGEDGE;
    io.pin_bit_mask = (1ULL << BTN0_PIN) | (1ULL << BTN1_PIN) |
                      (1ULL << BTN2_PIN) | (1ULL << BTN3_PIN);
    gpio_config(&io);

    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add(BTN0_PIN, btn_isr_handler, (void*)0);
    gpio_isr_handler_add(BTN1_PIN, btn_isr_handler, (void*)1);
    gpio_isr_handler_add(BTN2_PIN, btn_isr_handler, (void*)2);
    gpio_isr_handler_add(BTN3_PIN, btn_isr_handler, (void*)3);

    ESP_LOGI(TAG, "Buttons init OK (GPIO %d/%d/%d/%d)",
             BTN0_PIN, BTN1_PIN, BTN2_PIN, BTN3_PIN);
}

// ── Wi-Fi / ESP-NOW ──────────────────────────────────────────
static void wifi_espnow_init(void)
{
    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS corrupt – erasing");
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(40));
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    // Broadcast peer — для отправки wake-команды node_bottom
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, BROADCAST_MAC, 6);
    peer.channel = 0;
    peer.encrypt = false;
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    ESP_LOGI(TAG, "Wi-Fi / ESP-NOW ready (RX + TX wake)");
}

// ── Display — Page 0: 2×2 grid ──────────────────────────────
static void draw_zone(int idx, float temp)
{
    int zx = (idx & 1) ? ZONE_W : 0;
    int zy = (idx & 2) ? ZONE_H : 0;
    int cx = zx + ZONE_W / 2;
    int cy = zy + ZONE_H / 2;

    canvas.fillRect(zx, zy, ZONE_W, ZONE_H, TFT_BLACK);

    canvas.setTextDatum(lgfx::top_left);
    canvas.setTextSize(2);
    canvas.setTextColor(TFT_CYAN, TFT_BLACK);
    canvas.drawString(SENSOR_LABELS[idx], zx + 5, zy + 4);

    canvas.setTextDatum(lgfx::middle_center);
    canvas.setTextSize(3);
    float t_disp = to_display(temp);
    if (isnan(temp)) {
        canvas.setTextColor(TFT_RED, TFT_BLACK);
        canvas.drawString("ERR", cx, cy);
    } else {
        canvas.setTextColor(TFT_GREEN, TFT_BLACK);
        char buf[16];
        snprintf(buf, sizeof(buf), "%.1f%s",
                 (double)t_disp, unit_fahrenheit ? "F" : "C");
        canvas.drawString(buf, cx, cy);
    }
}

// ── Display — Page 1: stats table ───────────────────────────
static void draw_stats_page(const float temps[4])
{
    canvas.fillScreen(TFT_BLACK);

    canvas.drawFastVLine(STATS_DIV1, 0, SCREEN_H, TFT_DARKGREY);
    canvas.drawFastVLine(STATS_DIV2, 0, SCREEN_H, TFT_DARKGREY);
    canvas.drawFastVLine(STATS_DIV3, 0, SCREEN_H, TFT_DARKGREY);
    canvas.drawFastHLine(0, STATS_ROW_H, SCREEN_W, TFT_DARKGREY);

    canvas.setTextDatum(lgfx::middle_center);
    canvas.setTextSize(1);
    canvas.setTextColor(TFT_DARKGREY, TFT_BLACK);
    canvas.drawString(unit_fahrenheit ? "F" : "C",
                      STATS_COL0_CX, STATS_ROW_H / 2);
    canvas.drawString("NOW",  STATS_COL1_CX, STATS_ROW_H / 2);
    canvas.drawString("MIN",  STATS_COL2_CX, STATS_ROW_H / 2);
    canvas.drawString("MAX",  STATS_COL3_CX, STATS_ROW_H / 2);

    for (int i = 0; i < 4; i++) {
        int y_c = (i + 1) * STATS_ROW_H + STATS_ROW_H / 2;

        float t_disp = to_display(temps[i]);
        float mn     = to_display(t_min[i]);
        float mx     = to_display(t_max[i]);

        canvas.setTextSize(2);
        canvas.setTextColor(TFT_CYAN, TFT_BLACK);
        canvas.drawString(SENSOR_LABELS[i], STATS_COL0_CX, y_c);

        if (isnan(t_disp)) {
            canvas.setTextColor(TFT_RED, TFT_BLACK);
            canvas.drawString("ERR", STATS_COL1_CX, y_c);
        } else {
            char buf[10];
            snprintf(buf, sizeof(buf), "%.1f", (double)t_disp);
            canvas.setTextColor(TFT_GREEN, TFT_BLACK);
            canvas.drawString(buf, STATS_COL1_CX, y_c);
        }

        if (isinf(mn)) {
            canvas.setTextColor(TFT_DARKGREY, TFT_BLACK);
            canvas.drawString("---", STATS_COL2_CX, y_c);
        } else {
            char buf[10];
            snprintf(buf, sizeof(buf), "%.1f", (double)mn);
            canvas.setTextColor(TFT_CYAN, TFT_BLACK);
            canvas.drawString(buf, STATS_COL2_CX, y_c);
        }

        if (isinf(mx)) {
            canvas.setTextColor(TFT_DARKGREY, TFT_BLACK);
            canvas.drawString("---", STATS_COL3_CX, y_c);
        } else {
            char buf[10];
            snprintf(buf, sizeof(buf), "%.1f", (double)mx);
            canvas.setTextColor(TFT_ORANGE, TFT_BLACK);
            canvas.drawString(buf, STATS_COL3_CX, y_c);
        }
    }
}

// ── Display — common: push to LCD ───────────────────────────
static void draw_screen(const float temps[4])
{
    if (canvas.getBuffer() == nullptr) return;

    if (display_page == 0) {
        canvas.fillScreen(TFT_BLACK);
        for (int i = 0; i < 4; i++) draw_zone(i, temps[i]);
        canvas.drawFastVLine(ZONE_W, 0,      SCREEN_H, TFT_DARKGREY);
        canvas.drawFastHLine(0,      ZONE_H, SCREEN_W, TFT_DARKGREY);
    } else {
        draw_stats_page(temps);
    }

    canvas.fillCircle(SCREEN_W / 2 - 6, SCREEN_H - 4, 3,
                      display_page == 0 ? TFT_WHITE : TFT_DARKGREY);
    canvas.fillCircle(SCREEN_W / 2 + 6, SCREEN_H - 4, 3,
                      display_page == 1 ? TFT_WHITE : TFT_DARKGREY);

    canvas.pushSprite(0, 0);
}

static void draw_splash(const char* line1, const char* line2)
{
    if (canvas.getBuffer() == nullptr) return;
    canvas.fillScreen(TFT_BLACK);
    canvas.setTextDatum(lgfx::middle_center);
    canvas.setTextSize(2);
    canvas.setTextColor(TFT_CYAN, TFT_BLACK);
    canvas.drawString(line1, SCREEN_W / 2, SCREEN_H / 2 - 14);
    canvas.setTextSize(1);
    canvas.setTextColor(TFT_DARKGREY, TFT_BLACK);
    canvas.drawString(line2, SCREEN_W / 2, SCREEN_H / 2 + 10);
    canvas.pushSprite(0, 0);
}

// ── Main task (5 Hz) ─────────────────────────────────────────
static void main_task(void* /*arg*/)
{
    float temps[4];
    static char line[256];
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        // ── 1. Handle button events ──────────────────────────
        uint8_t flags;
        portENTER_CRITICAL(&btn_mux);
        flags     = btn_flags;
        btn_flags = 0;
        portEXIT_CRITICAL(&btn_mux);

        if (flags) {
            // Любая кнопка: сбросить таймер сна
            last_activity_ms = esp_timer_get_time() / 1000LL;
            if (display_sleeping) {
                display_wake();
                // Сообщить node_bottom о пробуждении
                espnow_cmd_t wake_cmd = {};
                wake_cmd.node_id = NODE_ID_TOP;
                wake_cmd.cmd     = NODE_CMD_WAKE_DISPLAY;
                esp_now_send(BROADCAST_MAC,
                             (const uint8_t*)&wake_cmd, sizeof(wake_cmd));
                ESP_LOGI(TAG, "Display woke, wake cmd sent to bottom");
            }
        }

        if (flags & (1u << 0)) {
            display_page = (display_page + 1) % 2;
            ESP_LOGI(TAG, "Page → %d", display_page);
        }
        if (flags & (1u << 1)) {
            unit_fahrenheit = !unit_fahrenheit;
            ESP_LOGI(TAG, "Unit → %s", unit_fahrenheit ? "°F" : "°C");
        }
        if (flags & (1u << 2)) {
            bl_idx = (bl_idx + 1) % BL_NUM_STEPS;
            if (!display_sleeping) bl_set(BL_LEVELS[bl_idx]);
            ESP_LOGI(TAG, "Backlight → %d%%",
                     (int)(BL_LEVELS[bl_idx] * 100 / 255));
        }
        if (flags & (1u << 3)) {
            for (int i = 0; i < 4; i++) {
                t_min[i] =  INFINITY;
                t_max[i] = -INFINITY;
            }
            ESP_LOGI(TAG, "Min/Max reset");
        }

        // ── 2. Read all 4 sensors ────────────────────────────
        uint8_t err_mask = 0;
        for (int i = 0; i < 4; i++) {
            uint32_t raw = max31855_read_raw(CS_PINS[i]);
            temps[i]     = max31855_parse(raw);
            if (isnan(temps[i])) {
                err_mask |= (uint8_t)(1 << i);
            } else {
                if (temps[i] < t_min[i]) t_min[i] = temps[i];
                if (temps[i] > t_max[i]) t_max[i] = temps[i];
            }
        }

        // ── 3. Update display (пропустить если спит) ─────────
        if (!display_sleeping) {
            draw_screen(temps);
        }

        // ── 4. Snapshot node_bottom data ─────────────────────
        int64_t now_ms = esp_timer_get_time() / 1000LL;
        espnow_packet_t bot_snap;
        int64_t         bot_rx;
        bool            bot_valid_snap;
        xSemaphoreTake(s_bot_mutex, portMAX_DELAY);
        bot_snap       = s_bot_pkt;
        bot_rx         = s_bot_rx_ms;
        bot_valid_snap = s_bot_valid;
        xSemaphoreGive(s_bot_mutex);

        bool bot_alive = bot_valid_snap && (now_ms - bot_rx < NODE_TIMEOUT_MS);

        // ── 5. Output JSON to USB-Serial ─────────────────────
        int p = 0;
        p += sprintf(line + p, "{\"timestamp\":%lld,\"top\":[",
                     (long long)now_ms);
        for (int i = 0; i < 4; i++) {
            if (i > 0) line[p++] = ',';
            p = append_float(line, p, temps[i]);
        }
        p += sprintf(line + p, "],\"bottom\":[");
        p = append_float(line, p, bot_alive ? bot_snap.temps[0] : NAN);

        const char* status;
        bool bot_err = bot_alive && (bot_snap.status != STATUS_OK);
        if (!bot_alive)               status = "BOT_TIMEOUT";
        else if (err_mask || bot_err) status = "SENSOR_ERR";
        else                          status = "OK";

        p += sprintf(line + p, "],\"status\":\"%s\"}\n", status);
        fputs(line, stdout);
        fflush(stdout);

        // ── 6. Auto-sleep check ──────────────────────────────
        if (!display_sleeping &&
            (now_ms - last_activity_ms >= DISPLAY_SLEEP_TIMEOUT_MS)) {
            display_sleep_start();
            ESP_LOGI(TAG, "Display sleep (inactivity)");
        }

        // ── 7. Precise 200 ms period ─────────────────────────
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(200));
    }
}

// ── Entry point ──────────────────────────────────────────────
extern "C" void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(1500));
    ESP_LOGI(TAG, "=== Node Top starting ===");

    for (int i = 0; i < 4; i++) {
        t_min[i] =  INFINITY;
        t_max[i] = -INFINITY;
    }

    s_bot_mutex      = xSemaphoreCreateMutex();
    last_activity_ms = esp_timer_get_time() / 1000LL;

    // ── Step 1: Sensor GPIO ──────────────────────────────────
    spi_sens_init();
    ESP_LOGI(TAG, "Sensor SPI init OK");

    // ── Step 2: Display hardware reset ──────────────────────
    gpio_set_direction(LCD_RST_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level    (LCD_RST_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level    (LCD_RST_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(150));
    ESP_LOGI(TAG, "Display HW reset done");

    // ── Step 3: Backlight PWM ────────────────────────────────
    bl_init();
    ESP_LOGI(TAG, "Backlight LEDC OK");

    // ── Step 4: LovyanGFX init ───────────────────────────────
    lcd.init();
    lcd.setRotation(1);

    canvas.setColorDepth(8);
    if (canvas.createSprite(SCREEN_W, SCREEN_H) == nullptr) {
        ESP_LOGE(TAG, "Sprite alloc FAILED – halting");
        for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "Sprite OK (%d×%d, 8bpp)", SCREEN_W, SCREEN_H);

    // ── Step 5: Boot splash ──────────────────────────────────
    draw_splash("NODE TOP", "Initialising...");

    // ── Step 6: Button GPIO + ISR ────────────────────────────
    buttons_init();

    // ── Step 7: Wi-Fi / ESP-NOW ──────────────────────────────
    wifi_espnow_init();

    // ── Step 8: First real sensor frame ──────────────────────
    float init_temps[4];
    for (int i = 0; i < 4; i++)
        init_temps[i] = max31855_parse(max31855_read_raw(CS_PINS[i]));
    draw_screen(init_temps);

    // ── Step 9: Main loop ────────────────────────────────────
    ESP_LOGI(TAG, "Launching main task");
    xTaskCreate(main_task, "main_task", 8192, nullptr, 5, nullptr);
}
