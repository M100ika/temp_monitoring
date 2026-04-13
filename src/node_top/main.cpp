/**
 * Node Top Firmware  —  ESP32-C6
 * 4× MAX31855 (soft-SPI) | LCD 1.47" ST7789 landscape via LGFX_Sprite | ESP-NOW TX
 *
 * Waveshare ESP32-C6-LCD-1.47 pinout (verified):
 *   LCD:     MOSI=6   SCLK=7   CS=14  DC=15  RST=21  BL=22
 *   Sensors: CLK=5    MISO=9   CS1=18 CS2=19 CS3=20  CS4=23
 *   Buttons: BTN0=0   BTN1=1   BTN2=2  BTN3=3  (INPUT_PULLUP)
 *
 * Button actions:
 *   BTN0 (GPIO 0) — cycle display pages:
 *                   Page 0 = Grid  (2×2 zones, current reading)
 *                   Page 1 = Stats (NOW / MIN / MAX table)
 *   BTN1 (GPIO 1) — toggle temperature unit  °C ↔ °F
 *   BTN2 (GPIO 2) — backlight brightness  100% → 50% → 15% → 100%
 *   BTN3 (GPIO 3) — reset MIN / MAX history for all sensors
 *
 * Display layout — Page 0 (320×172, rotation=1):
 *   ┌──────────────┬──────────────┐
 *   │ T1  xx.x°C   │ T2  xx.x°C   │  each zone 160×86
 *   ├──────────────┼──────────────┤
 *   │ T3  xx.x°C   │ T4  xx.x°C   │
 *   └──────────────┴──────────────┘
 *
 * Display layout — Page 1 (Stats):
 *   ┌────┬──────────┬──────────┬──────────┐
 *   │ C  │   NOW    │   MIN    │   MAX    │  header row
 *   ├────┼──────────┼──────────┼──────────┤
 *   │ T1 │  xx.x    │  xx.x    │  xx.x    │
 *   │ T2 │  xx.x    │  xx.x    │  xx.x    │
 *   │ T3 │  xx.x    │  xx.x    │  xx.x    │
 *   │ T4 │  xx.x    │  xx.x    │  xx.x    │
 *   └────┴──────────┴──────────┴──────────┘
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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

#define BTN0_PIN     GPIO_NUM_0   // cycle page
#define BTN1_PIN     GPIO_NUM_1   // toggle °C / °F
#define BTN2_PIN     GPIO_NUM_2   // backlight brightness
#define BTN3_PIN     GPIO_NUM_3   // reset min/max stats

static const uint8_t BROADCAST_MAC[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// ── Backlight (LEDC PWM on BL_PIN) ──────────────────────────
#define BL_LEDC_TIMER   LEDC_TIMER_0
#define BL_LEDC_CHANNEL LEDC_CHANNEL_0
#define BL_LEDC_FREQ_HZ 5000u

static const uint8_t BL_LEVELS[]   = {255, 128, 38};   // 100% / 50% / 15%
static const int     BL_NUM_STEPS  = (int)(sizeof(BL_LEVELS));

// ── Display geometry ─────────────────────────────────────────
// Logical screen after setRotation(1): 320 wide × 172 tall
#define SCREEN_W  320
#define SCREEN_H  172
// Page 0 — 2×2 grid zones
#define ZONE_W    (SCREEN_W / 2)   // 160
#define ZONE_H    (SCREEN_H / 2)   //  86

// Page 1 — stats table column geometry
#define STATS_ROW_H    34    // header + 4 rows × 34px = 170px ≤ 172
#define STATS_COL0_CX  20    // "Tx" label center-x
#define STATS_COL1_CX  96    // NOW  center-x
#define STATS_COL2_CX  193   // MIN  center-x
#define STATS_COL3_CX  277   // MAX  center-x
#define STATS_DIV1     39    // vertical divider after label col
#define STATS_DIV2     152   // vertical divider after NOW col
#define STATS_DIV3     233   // vertical divider after MIN col

// Sensor label strings — avoids snprintf / format-truncation warnings
static const char* const SENSOR_LABELS[4] = {"T1", "T2", "T3", "T4"};

// ── Runtime state ────────────────────────────────────────────
static portMUX_TYPE      btn_mux       = portMUX_INITIALIZER_UNLOCKED;
static volatile uint8_t  btn_flags     = 0;        // bit N = BTN_N pressed
static int64_t           btn_last_us[4] = {};       // debounce timestamps (µs)

static uint8_t  display_page    = 0;    // 0=GRID, 1=STATS
static bool     unit_fahrenheit = false;
static int      bl_idx          = 0;    // index into BL_LEVELS[]

// Per-sensor extremes (°C); reset by BTN3
static float    t_min[4];   // initialised to +INFINITY
static float    t_max[4];   // initialised to -INFINITY

// ── LovyanGFX objects ────────────────────────────────────────
static LGFX        lcd;
static LGFX_Sprite canvas(&lcd);

// ── Helpers ──────────────────────────────────────────────────
static inline float to_display(float t_c)
{
    if (isnan(t_c)) return NAN;
    return unit_fahrenheit ? (t_c * 1.8f + 32.0f) : t_c;
}

// ── Button ISR ───────────────────────────────────────────────
static void IRAM_ATTR btn_isr_handler(void* arg)
{
    const int btn = (int)(intptr_t)arg;
    static const gpio_num_t PINS[] = {BTN0_PIN, BTN1_PIN, BTN2_PIN, BTN3_PIN};

    // 50 ms hardware debounce — ignore bounces
    int64_t now_us = esp_timer_get_time();
    if (now_us - btn_last_us[btn] < 50000LL) return;
    // Confirm pin is still LOW (not a transient spike)
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
    ch.duty       = BL_LEVELS[0];   // full brightness on boot
    ch.hpoint     = 0;
    ledc_channel_config(&ch);
}

static void bl_set(uint8_t level)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, BL_LEDC_CHANNEL, level);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, BL_LEDC_CHANNEL);
}

// ── Button GPIO init ─────────────────────────────────────────
static void buttons_init(void)
{
    gpio_config_t io = {};
    io.mode         = GPIO_MODE_INPUT;
    io.pull_up_en   = GPIO_PULLUP_ENABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type    = GPIO_INTR_NEGEDGE;   // falling edge = press
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
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));

    ESP_ERROR_CHECK(esp_now_init());

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, BROADCAST_MAC, 6);
    peer.channel = 0;
    peer.encrypt = false;
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    ESP_LOGI(TAG, "Wi-Fi / ESP-NOW ready");
}

// ── Display — Page 0: 2×2 grid ──────────────────────────────
static void draw_zone(int idx, float temp)
{
    int zx = (idx & 1) ? ZONE_W : 0;
    int zy = (idx & 2) ? ZONE_H : 0;
    int cx = zx + ZONE_W / 2;
    int cy = zy + ZONE_H / 2;

    canvas.fillRect(zx, zy, ZONE_W, ZONE_H, TFT_BLACK);

    // "T1"…"T4" label — top-left of zone
    canvas.setTextDatum(lgfx::top_left);
    canvas.setTextSize(2);
    canvas.setTextColor(TFT_CYAN, TFT_BLACK);
    canvas.drawString(SENSOR_LABELS[idx], zx + 5, zy + 4);

    // Temperature value — centered in zone
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

    // Vertical column dividers
    canvas.drawFastVLine(STATS_DIV1, 0, SCREEN_H, TFT_DARKGREY);
    canvas.drawFastVLine(STATS_DIV2, 0, SCREEN_H, TFT_DARKGREY);
    canvas.drawFastVLine(STATS_DIV3, 0, SCREEN_H, TFT_DARKGREY);
    // Horizontal divider under header row
    canvas.drawFastHLine(0, STATS_ROW_H, SCREEN_W, TFT_DARKGREY);

    // Header row
    canvas.setTextDatum(lgfx::middle_center);
    canvas.setTextSize(1);
    canvas.setTextColor(TFT_DARKGREY, TFT_BLACK);
    canvas.drawString(unit_fahrenheit ? "F" : "C",
                      STATS_COL0_CX, STATS_ROW_H / 2);
    canvas.drawString("NOW",  STATS_COL1_CX, STATS_ROW_H / 2);
    canvas.drawString("MIN",  STATS_COL2_CX, STATS_ROW_H / 2);
    canvas.drawString("MAX",  STATS_COL3_CX, STATS_ROW_H / 2);

    // Data rows T1…T4
    for (int i = 0; i < 4; i++) {
        int y_c = (i + 1) * STATS_ROW_H + STATS_ROW_H / 2;

        // Apply unit conversion (sentinel ±INFINITY stays non-finite)
        float t_disp = to_display(temps[i]);
        float mn     = to_display(t_min[i]);
        float mx     = to_display(t_max[i]);

        // Sensor label
        canvas.setTextSize(2);
        canvas.setTextColor(TFT_CYAN, TFT_BLACK);
        canvas.drawString(SENSOR_LABELS[i], STATS_COL0_CX, y_c);

        // NOW column
        if (isnan(t_disp)) {
            canvas.setTextColor(TFT_RED, TFT_BLACK);
            canvas.drawString("ERR", STATS_COL1_CX, y_c);
        } else {
            char buf[10];
            snprintf(buf, sizeof(buf), "%.1f", (double)t_disp);
            canvas.setTextColor(TFT_GREEN, TFT_BLACK);
            canvas.drawString(buf, STATS_COL1_CX, y_c);
        }

        // MIN column — "---" until first valid reading
        if (isinf(mn)) {
            canvas.setTextColor(TFT_DARKGREY, TFT_BLACK);
            canvas.drawString("---", STATS_COL2_CX, y_c);
        } else {
            char buf[10];
            snprintf(buf, sizeof(buf), "%.1f", (double)mn);
            canvas.setTextColor(TFT_CYAN, TFT_BLACK);
            canvas.drawString(buf, STATS_COL2_CX, y_c);
        }

        // MAX column — "---" until first valid reading
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

    // Page indicator: 2 dots at bottom-center
    // Left dot = Page 0 (GRID), right dot = Page 1 (STATS)
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
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        // ── 1. Handle button events ──────────────────────────
        uint8_t flags;
        portENTER_CRITICAL(&btn_mux);
        flags     = btn_flags;
        btn_flags = 0;
        portEXIT_CRITICAL(&btn_mux);

        if (flags & (1u << 0)) {                       // BTN0 — cycle page
            display_page = (display_page + 1) % 2;
            ESP_LOGI(TAG, "Page → %d (%s)",
                     display_page, display_page == 0 ? "GRID" : "STATS");
        }
        if (flags & (1u << 1)) {                       // BTN1 — toggle unit
            unit_fahrenheit = !unit_fahrenheit;
            ESP_LOGI(TAG, "Unit → %s", unit_fahrenheit ? "°F" : "°C");
        }
        if (flags & (1u << 2)) {                       // BTN2 — brightness
            bl_idx = (bl_idx + 1) % BL_NUM_STEPS;
            bl_set(BL_LEVELS[bl_idx]);
            ESP_LOGI(TAG, "Backlight → %d%%",
                     (int)(BL_LEVELS[bl_idx] * 100 / 255));
        }
        if (flags & (1u << 3)) {                       // BTN3 — reset min/max
            for (int i = 0; i < 4; i++) {
                t_min[i] =  INFINITY;
                t_max[i] = -INFINITY;
            }
            ESP_LOGI(TAG, "Min/Max history reset");
        }

        // ── 2. Read all 4 sensors ────────────────────────────
        uint8_t err_mask = 0;
        for (int i = 0; i < 4; i++) {
            uint32_t raw = max31855_read_raw(CS_PINS[i]);
            temps[i]     = max31855_parse(raw);
            if (isnan(temps[i])) {
                err_mask |= (uint8_t)(1 << i);
            } else {
                // Update per-sensor extremes (always in °C)
                if (temps[i] < t_min[i]) t_min[i] = temps[i];
                if (temps[i] > t_max[i]) t_max[i] = temps[i];
            }
        }

        // ── 3. Update display ────────────────────────────────
        draw_screen(temps);

        // ── 4. Send via ESP-NOW (raw °C — gateway converts if needed) ──
        espnow_packet_t pkt = {};
        pkt.node_id      = NODE_ID_TOP;
        pkt.timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
        memcpy(pkt.temps, temps, sizeof(float) * 4);
        pkt.num_sensors  = 4;
        pkt.status       = err_mask ? STATUS_ERR_SENS : STATUS_OK;

        esp_err_t ret = esp_now_send(BROADCAST_MAC,
                                     (const uint8_t*)&pkt, sizeof(pkt));
        if (ret != ESP_OK)
            ESP_LOGW(TAG, "esp_now_send: %s", esp_err_to_name(ret));

        ESP_LOGI(TAG, "T1=%.1f T2=%.1f T3=%.1f T4=%.1f",
                 (double)temps[0], (double)temps[1],
                 (double)temps[2], (double)temps[3]);

        // ── 5. Precise 200 ms period ─────────────────────────
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(200));
    }
}

// ── Entry point ──────────────────────────────────────────────
extern "C" void app_main(void)
{
    // Allow USB-CDC monitor to attach
    vTaskDelay(pdMS_TO_TICKS(1500));
    ESP_LOGI(TAG, "=== Node Top starting ===");

    // Initialise per-sensor extremes to sentinel values
    for (int i = 0; i < 4; i++) {
        t_min[i] =  INFINITY;
        t_max[i] = -INFINITY;
    }

    // ── Step 1: Sensor GPIO ──────────────────────────────────
    spi_sens_init();
    ESP_LOGI(TAG, "Sensor SPI init OK");

    // ── Step 2: Display hardware reset ──────────────────────
    gpio_set_direction(LCD_RST_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level    (LCD_RST_PIN, 0);            // assert reset
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level    (LCD_RST_PIN, 1);            // release reset
    vTaskDelay(pdMS_TO_TICKS(150));                // ST7789 power-on time
    ESP_LOGI(TAG, "Display HW reset done");

    // ── Step 3: Backlight PWM ────────────────────────────────
    bl_init();
    ESP_LOGI(TAG, "Backlight LEDC OK (full brightness)");

    // ── Step 4: LovyanGFX init ───────────────────────────────
    lcd.init();
    lcd.setRotation(1);    // landscape → 320×172

    canvas.setColorDepth(8);   // 8-bit: 320×172×1 = 54 KB (vs 107 KB 16-bit)
    if (canvas.createSprite(SCREEN_W, SCREEN_H) == nullptr) {
        ESP_LOGE(TAG, "Sprite alloc FAILED – halting");
        for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "Sprite OK (%d×%d, 8bpp)", SCREEN_W, SCREEN_H);

    // ── Step 5: Boot splash ──────────────────────────────────
    draw_splash("NODE TOP", "Initialising...");
    ESP_LOGI(TAG, "Splash displayed");

    // ── Step 6: Button GPIO + ISR ────────────────────────────
    buttons_init();

    // ── Step 7: Wi-Fi / ESP-NOW ──────────────────────────────
    ESP_LOGI(TAG, "Starting Wi-Fi...");
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
