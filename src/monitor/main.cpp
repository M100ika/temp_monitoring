/**
 * monitor/main.cpp — ESP32-WROOM-32
 * 5× MAX31855 (soft-SPI) | ST7735S 1.8" 160×128 (landscape) via LovyanGFX
 * Фильтр шума: медиана(5) + EMA(α=0.3) | Serial JSON → PC
 *
 * GPIO:
 *   LCD (VSPI): MOSI=23 CLK=18 CS=5 DC=22 RST=4 BL=21
 *   Датчики:    CLK=14  MISO=35  CS1=25 CS2=26 CS3=27 CS4=32 CS5=33
 *   Кнопки:     BTN0=0  BTN1=13  BTN2=16  BTN3=17  (INPUT_PULLUP)
 *
 * Кнопки:
 *   BTN0 — следующая страница (Grid → Stats → Grid)
 *   BTN1 — переключить °C / °F
 *   BTN2 — яркость 100% → 50% → 15% → 100%
 *   BTN3 — сброс Min / Max
 *   Любая — пробуждение дисплея (первое нажатие только будит)
 *
 * JSON (каждые 200 мс):
 *   {"timestamp":12345,"top":[T1,T2,T3,T4],"bottom":[T5],"status":"OK"}
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "lgfx_config_monitor.h"

static const char* TAG = "Monitor";

// ── Пины датчиков ─────────────────────────────────────────────
#define SENS_CLK   GPIO_NUM_14
#define SENS_MISO  GPIO_NUM_35

static const gpio_num_t CS_PINS[5] = {
    GPIO_NUM_25, GPIO_NUM_26, GPIO_NUM_27, GPIO_NUM_32, GPIO_NUM_33
};

// ── Пины кнопок ───────────────────────────────────────────────
#define BTN0_PIN  GPIO_NUM_0
#define BTN1_PIN  GPIO_NUM_13
#define BTN2_PIN  GPIO_NUM_16
#define BTN3_PIN  GPIO_NUM_17

// ── Экран (landscape) ─────────────────────────────────────────
#define SCREEN_W  160
#define SCREEN_H  128

// ── Подсветка ─────────────────────────────────────────────────
static const uint8_t BL_LEVELS[]  = {255, 128, 38};  // 100% / 50% / 15%
static const int     BL_NUM_STEPS = 3;

// ── Фильтр: медиана(5) + EMA ─────────────────────────────────
#define FILTER_LEN  5
#define EMA_ALPHA   0.3f

static float raw_buf[5][FILTER_LEN];
static int   buf_pos[5];
static float ema_val[5];
static bool  ema_init[5];

static float median5(float arr[5]) {
    float a[5];
    memcpy(a, arr, sizeof(a));
    // Insertion sort для 5 элементов
    for (int i = 1; i < 5; i++) {
        float key = a[i];
        int j = i - 1;
        while (j >= 0 && a[j] > key) { a[j + 1] = a[j]; j--; }
        a[j + 1] = key;
    }
    return a[2];
}

// Возвращает отфильтрованное значение.
// При первом вызове заполняет буфер текущим значением — без прогрева.
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
static LGFX        lcd;
static LGFX_Sprite canvas(&lcd);

static bool    unit_fahrenheit  = false;
static uint8_t display_page    = 0;      // 0 = Grid, 1 = Stats
static int     bl_idx          = 0;
static float   t_min[5], t_max[5];
static bool    display_sleeping = false;
static int64_t last_btn_ms     = 0;
#define SLEEP_TIMEOUT_MS  30000

static portMUX_TYPE     btn_mux      = portMUX_INITIALIZER_UNLOCKED;
static volatile uint8_t btn_flags    = 0;
static int64_t          btn_last_us[4] = {};

static const char* const SENSOR_LABELS[5] = {"T1", "T2", "T3", "T4", "T5"};

static inline float to_disp(float c) {
    return (!unit_fahrenheit || isnan(c)) ? c : c * 1.8f + 32.0f;
}

static int append_float(char* buf, int pos, float v) {
    if (isnan(v) || isinf(v)) return pos + sprintf(buf + pos, "null");
    return pos + sprintf(buf + pos, "%.2f", (double)v);
}

// ── Программный SPI / MAX31855 ────────────────────────────────
static void spi_sens_init(void) {
    gpio_config_t io = {};
    io.mode         = GPIO_MODE_OUTPUT;
    io.pull_up_en   = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type    = GPIO_INTR_DISABLE;
    io.pin_bit_mask = (1ULL << SENS_CLK);
    for (int i = 0; i < 5; i++) io.pin_bit_mask |= (1ULL << CS_PINS[i]);
    gpio_config(&io);

    io.mode         = GPIO_MODE_INPUT;
    io.pull_up_en   = GPIO_PULLUP_ENABLE;
    io.pin_bit_mask = (1ULL << SENS_MISO);
    gpio_config(&io);

    gpio_set_level(SENS_CLK, 0);
    for (int i = 0; i < 5; i++) gpio_set_level(CS_PINS[i], 1);
}

static uint32_t max31855_read_raw(gpio_num_t cs) {
    uint32_t data = 0;
    gpio_set_level(cs, 0);
    esp_rom_delay_us(2);
    for (int i = 31; i >= 0; i--) {
        gpio_set_level(SENS_CLK, 0); esp_rom_delay_us(1);
        if (gpio_get_level(SENS_MISO)) data |= (1UL << i);
        gpio_set_level(SENS_CLK, 1); esp_rom_delay_us(1);
    }
    gpio_set_level(SENS_CLK, 0);
    gpio_set_level(cs, 1);
    return data;
}

static float max31855_parse(uint32_t raw) {
    if (raw & 0x7) return NAN;          // fault bits: OC / SCG / SCV
    int16_t t14 = (int16_t)((raw >> 18) & 0x3FFF);
    if (t14 & 0x2000) t14 |= (int16_t)0xC000;  // знаковое расширение
    return t14 * 0.25f;
}

// ── Кнопки (ISR) ─────────────────────────────────────────────
static void IRAM_ATTR btn_isr_handler(void* arg) {
    const int btn = (int)(intptr_t)arg;
    static const gpio_num_t PINS[] = {BTN0_PIN, BTN1_PIN, BTN2_PIN, BTN3_PIN};

    int64_t now_us = esp_timer_get_time();
    if (now_us - btn_last_us[btn] < 50000LL) return;   // debounce 50 мс
    if (gpio_get_level(PINS[btn]) != 0) return;         // шум на отпускании

    btn_last_us[btn] = now_us;
    portENTER_CRITICAL_ISR(&btn_mux);
    btn_flags |= (uint8_t)(1u << btn);
    portEXIT_CRITICAL_ISR(&btn_mux);
}

static void buttons_init(void) {
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
}

// ── Дисплей — Страница 0: сетка 3+2 (landscape 160×128) ──────
//
//  T1 (x=0..52)  | T2 (x=54..105) | T3 (x=107..159)   y=0..63
//  ───────────────────────────────────────────────────   y=64
//  T4 (x=0..78)  |  T5 (x=80..159)                     y=64..127

struct zone_t { int16_t x, y, w, h; };

static const zone_t GRID_ZONES[5] = {
    {0,   0,  53, 64},  // T1
    {54,  0,  52, 64},  // T2
    {107, 0,  53, 64},  // T3
    {0,   64, 79, 64},  // T4
    {80,  64, 80, 64},  // T5
};

static void draw_grid_zone(int idx, float temp) {
    const zone_t& z = GRID_ZONES[idx];
    int cx = z.x + z.w / 2;
    int cy = z.y + z.h / 2;

    canvas.fillRect(z.x, z.y, z.w, z.h, TFT_BLACK);

    // Метка датчика — левый верхний угол
    canvas.setTextDatum(lgfx::top_left);
    canvas.setTextSize(1);
    canvas.setTextColor(TFT_CYAN, TFT_BLACK);
    canvas.drawString(SENSOR_LABELS[idx], z.x + 3, z.y + 3);

    // Температура — по центру зоны
    canvas.setTextDatum(lgfx::middle_center);
    if (isnan(temp)) {
        canvas.setTextSize(1);
        canvas.setTextColor(TFT_RED, TFT_BLACK);
        canvas.drawString("ERR", cx, cy);
    } else {
        float t = to_disp(temp);
        canvas.setTextSize(2);
        canvas.setTextColor(TFT_GREEN, TFT_BLACK);
        char buf[16];
        snprintf(buf, sizeof(buf), "%.1f%s", (double)t, unit_fahrenheit ? "F" : "C");
        canvas.drawString(buf, cx, cy);
    }
}

static void draw_grid_page(const float temps[5]) {
    canvas.fillScreen(TFT_BLACK);

    // Разделители
    canvas.drawFastHLine(0,   64, SCREEN_W, TFT_DARKGREY);
    canvas.drawFastVLine(53,  0,  64,       TFT_DARKGREY);
    canvas.drawFastVLine(106, 0,  64,       TFT_DARKGREY);
    canvas.drawFastVLine(79,  64, 64,       TFT_DARKGREY);

    for (int i = 0; i < 5; i++) draw_grid_zone(i, temps[i]);
}

// ── Дисплей — Страница 1: таблица Now/Min/Max ─────────────────
//
//  Заголовок: 28 px
//  5 строк по 20 px: (128-28)/5 = 20
//  Колонки: метка(0-29) NOW(30-79) MIN(80-119) MAX(120-159)

#define STAT_HDR_H  28
#define STAT_ROW_H  20

static const int STAT_COL_CX[4] = {15, 55, 100, 140};

static void draw_stats_page(const float temps[5]) {
    canvas.fillScreen(TFT_BLACK);

    // Вертикальные разделители
    canvas.drawFastVLine(30,  0, SCREEN_H, TFT_DARKGREY);
    canvas.drawFastVLine(80,  0, SCREEN_H, TFT_DARKGREY);
    canvas.drawFastVLine(120, 0, SCREEN_H, TFT_DARKGREY);
    // Горизонтальный под заголовком
    canvas.drawFastHLine(0, STAT_HDR_H, SCREEN_W, TFT_DARKGREY);

    // Заголовки колонок
    canvas.setTextDatum(lgfx::middle_center);
    canvas.setTextSize(1);
    canvas.setTextColor(TFT_DARKGREY, TFT_BLACK);
    canvas.drawString(unit_fahrenheit ? "F" : "C", STAT_COL_CX[0], STAT_HDR_H / 2);
    canvas.drawString("NOW", STAT_COL_CX[1], STAT_HDR_H / 2);
    canvas.drawString("MIN", STAT_COL_CX[2], STAT_HDR_H / 2);
    canvas.drawString("MAX", STAT_COL_CX[3], STAT_HDR_H / 2);

    for (int i = 0; i < 5; i++) {
        int yc = STAT_HDR_H + i * STAT_ROW_H + STAT_ROW_H / 2;

        if (i < 4) canvas.drawFastHLine(0, STAT_HDR_H + (i + 1) * STAT_ROW_H, SCREEN_W, TFT_DARKGREY);

        char buf[10];

        // Метка
        canvas.setTextSize(1);
        canvas.setTextColor(TFT_CYAN, TFT_BLACK);
        canvas.drawString(SENSOR_LABELS[i], STAT_COL_CX[0], yc);

        // NOW
        if (isnan(temps[i])) {
            canvas.setTextColor(TFT_RED, TFT_BLACK);
            canvas.drawString("ERR", STAT_COL_CX[1], yc);
        } else {
            snprintf(buf, sizeof(buf), "%.1f", (double)to_disp(temps[i]));
            canvas.setTextColor(TFT_GREEN, TFT_BLACK);
            canvas.drawString(buf, STAT_COL_CX[1], yc);
        }

        // MIN
        if (isinf(t_min[i])) {
            canvas.setTextColor(TFT_DARKGREY, TFT_BLACK);
            canvas.drawString("---", STAT_COL_CX[2], yc);
        } else {
            snprintf(buf, sizeof(buf), "%.1f", (double)to_disp(t_min[i]));
            canvas.setTextColor(TFT_CYAN, TFT_BLACK);
            canvas.drawString(buf, STAT_COL_CX[2], yc);
        }

        // MAX
        if (isinf(t_max[i])) {
            canvas.setTextColor(TFT_DARKGREY, TFT_BLACK);
            canvas.drawString("---", STAT_COL_CX[3], yc);
        } else {
            snprintf(buf, sizeof(buf), "%.1f", (double)to_disp(t_max[i]));
            canvas.setTextColor(TFT_ORANGE, TFT_BLACK);
            canvas.drawString(buf, STAT_COL_CX[3], yc);
        }
    }
}

static void draw_screen(const float temps[5]) {
    if (canvas.getBuffer() == nullptr) return;
    if (display_page == 0) draw_grid_page(temps);
    else                   draw_stats_page(temps);
    canvas.pushSprite(0, 0);
}

static void draw_splash(const char* msg) {
    if (canvas.getBuffer() == nullptr) return;
    canvas.fillScreen(TFT_BLACK);
    canvas.setTextDatum(lgfx::middle_center);
    canvas.setTextSize(1);
    canvas.setTextColor(TFT_CYAN, TFT_BLACK);
    canvas.drawString("TempMonitor v2", SCREEN_W / 2, SCREEN_H / 2 - 8);
    canvas.setTextColor(TFT_DARKGREY, TFT_BLACK);
    canvas.drawString(msg, SCREEN_W / 2, SCREEN_H / 2 + 8);
    canvas.pushSprite(0, 0);
}

// ── Основная задача (5 Гц) ────────────────────────────────────
static void main_task(void* /*arg*/) {
    float temps[5];
    static char line[256];
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        int64_t now_ms = esp_timer_get_time() / 1000LL;

        // ── 1. Обработка кнопок ───────────────────────────────
        uint8_t flags;
        portENTER_CRITICAL(&btn_mux);
        flags     = btn_flags;
        btn_flags = 0;
        portEXIT_CRITICAL(&btn_mux);

        if (flags) {
            if (display_sleeping) {
                // Первое нажатие только будит — действие не выполняется
                display_sleeping = false;
                lcd.setBrightness(BL_LEVELS[bl_idx]);
                last_btn_ms = now_ms;
                flags = 0;
            } else {
                last_btn_ms = now_ms;
            }
        }

        if (flags & (1u << 0)) {
            display_page = (display_page + 1) % 2;
            ESP_LOGI(TAG, "Page -> %d", display_page);
        }
        if (flags & (1u << 1)) {
            unit_fahrenheit = !unit_fahrenheit;
            ESP_LOGI(TAG, "Unit -> %s", unit_fahrenheit ? "F" : "C");
        }
        if (flags & (1u << 2)) {
            bl_idx = (bl_idx + 1) % BL_NUM_STEPS;
            lcd.setBrightness(BL_LEVELS[bl_idx]);
            ESP_LOGI(TAG, "BL -> %d%%", BL_LEVELS[bl_idx] * 100 / 255);
        }
        if (flags & (1u << 3)) {
            for (int i = 0; i < 5; i++) {
                t_min[i] =  INFINITY;
                t_max[i] = -INFINITY;
            }
            ESP_LOGI(TAG, "Min/Max reset");
        }

        // ── 2. Чтение + фильтрация датчиков ──────────────────
        uint8_t err_mask = 0;
        for (int i = 0; i < 5; i++) {
            float raw = max31855_parse(max31855_read_raw(CS_PINS[i]));
            if (isnan(raw)) {
                err_mask |= (uint8_t)(1 << i);
                temps[i]  = NAN;
            } else {
                temps[i] = filter_update(i, raw);
                if (temps[i] < t_min[i]) t_min[i] = temps[i];
                if (temps[i] > t_max[i]) t_max[i] = temps[i];
            }
        }

        // ── 3. Обновление дисплея ─────────────────────────────
        if (!display_sleeping) {
            draw_screen(temps);
            if (now_ms - last_btn_ms >= SLEEP_TIMEOUT_MS) {
                display_sleeping = true;
                lcd.setBrightness(0);
                ESP_LOGI(TAG, "Display sleep");
            }
        }

        // ── 4. Вывод JSON в Serial ────────────────────────────
        int p = 0;
        p += sprintf(line + p, "{\"timestamp\":%lld,\"top\":[", (long long)now_ms);
        for (int i = 0; i < 4; i++) {
            if (i > 0) line[p++] = ',';
            p = append_float(line, p, temps[i]);
        }
        p += sprintf(line + p, "],\"bottom\":[");
        p = append_float(line, p, temps[4]);
        p += sprintf(line + p, "],\"status\":\"%s\"}\n",
                     err_mask ? "SENSOR_ERR" : "OK");
        fputs(line, stdout);
        fflush(stdout);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(200));
    }
}

// ── Точка входа ───────────────────────────────────────────────
extern "C" void app_main(void) {
    vTaskDelay(pdMS_TO_TICKS(1500));
    ESP_LOGI(TAG, "=== TempMonitor v2 (ESP32-WROOM-32) ===");

    for (int i = 0; i < 5; i++) {
        t_min[i]    =  INFINITY;
        t_max[i]    = -INFINITY;
        ema_init[i] = false;
        buf_pos[i]  = 0;
    }
    last_btn_ms = esp_timer_get_time() / 1000LL;

    // Шаг 1: GPIO датчиков
    spi_sens_init();
    ESP_LOGI(TAG, "Sensor SPI OK");

    // Шаг 2: Дисплей
    lcd.init();
    lcd.setRotation(1);                     // landscape 160×128
    lcd.setBrightness(BL_LEVELS[0]);

    canvas.setColorDepth(8);               // 8 bpp — 20 KB спрайт
    if (canvas.createSprite(SCREEN_W, SCREEN_H) == nullptr) {
        ESP_LOGE(TAG, "Sprite alloc FAILED — halting");
        for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "Display OK (%dx%d, 8bpp)", SCREEN_W, SCREEN_H);

    draw_splash("Initialising...");

    // Шаг 3: Кнопки
    buttons_init();
    ESP_LOGI(TAG, "Buttons OK (GPIO %d/%d/%d/%d)",
             BTN0_PIN, BTN1_PIN, BTN2_PIN, BTN3_PIN);

    // Шаг 4: Первый кадр
    float init_temps[5];
    for (int i = 0; i < 5; i++)
        init_temps[i] = max31855_parse(max31855_read_raw(CS_PINS[i]));
    draw_screen(init_temps);

    // Шаг 5: Основная задача
    ESP_LOGI(TAG, "Starting main task @ 5 Hz");
    xTaskCreate(main_task, "main_task", 8192, nullptr, 5, nullptr);
}
