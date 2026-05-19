/**
 * Node Bottom Firmware  —  ESP32-C6
 * 1× MAX31855 (soft-SPI) | LCD 1.47" ST7789 landscape via LGFX_Sprite | ESP-NOW TX
 *
 * Waveshare ESP32-C6-LCD-1.47 pinout (verified):
 *   LCD  : MOSI=6  SCLK=7  CS=14  DC=15  RST=21  BL=22
 *   Sensor soft-SPI : CLK=GPIO5  MISO=GPIO9  CS=GPIO18
 *
 * Нет кнопок. Дисплей засыпает через DISPLAY_SLEEP_TIMEOUT_MS.
 * Пробуждение — только по ESP-NOW команде NODE_CMD_WAKE_DISPLAY от node_top.
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "lgfx_config.h"
#include "espnow_packet.h"

static const char* TAG = "NODE_BTM";

// ── Pin definitions ──────────────────────────────────────────
#define SENS_CLK    GPIO_NUM_5
#define SENS_MISO   GPIO_NUM_9
#define SENS_CS     GPIO_NUM_18
#define LCD_BL_PIN  GPIO_NUM_22
#define LCD_RST_PIN GPIO_NUM_21

static const uint8_t BROADCAST_MAC[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

#define SCREEN_W  320
#define SCREEN_H  172

// ── Timeout ──────────────────────────────────────────────────
#define DISPLAY_SLEEP_TIMEOUT_MS  30000   // 30 с без wake-команды → дисплей выкл

// ── Display sleep state ──────────────────────────────────────
static volatile bool s_wake_requested = false;   // set in recv_cb, cleared in main_task
static bool          display_sleeping  = false;
static int64_t       last_activity_ms  = 0;

// ── LovyanGFX objects ────────────────────────────────────────
static LGFX        lcd;
static LGFX_Sprite canvas(&lcd);

// ── Soft-SPI MAX31855 ────────────────────────────────────────
static void spi_sens_init(void)
{
    gpio_config_t io = {};
    io.mode         = GPIO_MODE_OUTPUT;
    io.pull_up_en   = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type    = GPIO_INTR_DISABLE;
    io.pin_bit_mask = (1ULL << SENS_CLK) | (1ULL << SENS_CS);
    gpio_config(&io);

    io.mode         = GPIO_MODE_INPUT;
    io.pull_up_en   = GPIO_PULLUP_ENABLE;
    io.pin_bit_mask = (1ULL << SENS_MISO);
    gpio_config(&io);

    gpio_set_level(SENS_CLK, 0);
    gpio_set_level(SENS_CS,  1);
}

static uint32_t max31855_read_raw(void)
{
    uint32_t data = 0;
    gpio_set_level(SENS_CS, 0);
    esp_rom_delay_us(2);
    for (int i = 31; i >= 0; i--) {
        gpio_set_level(SENS_CLK, 0);
        esp_rom_delay_us(1);
        if (gpio_get_level(SENS_MISO)) data |= (1UL << i);
        gpio_set_level(SENS_CLK, 1);
        esp_rom_delay_us(1);
    }
    gpio_set_level(SENS_CLK, 0);
    gpio_set_level(SENS_CS,  1);
    return data;
}

static float max31855_parse(uint32_t raw)
{
    if (raw & 0x7) return NAN;
    int16_t t14 = (int16_t)((raw >> 18) & 0x3FFF);
    if (t14 & 0x2000) t14 |= (int16_t)0xC000;
    return t14 * 0.25f;
}

// ── Display sleep / wake ─────────────────────────────────────
static void display_sleep_start(void)
{
    display_sleeping = true;
    gpio_set_level(LCD_BL_PIN, 0);
}

static void display_wake(void)
{
    display_sleeping = false;
    gpio_set_level(LCD_BL_PIN, 1);
}

// ── ESP-NOW receive callback ──────────────────────────────────
// Запускается в контексте Wi-Fi задачи; пишет только volatile флаг.
static void espnow_recv_cb(const esp_now_recv_info_t* info,
                           const uint8_t* data, int len)
{
    (void)info;
    if (len != (int)sizeof(espnow_cmd_t)) return;
    const espnow_cmd_t* cmd = (const espnow_cmd_t*)data;
    if (cmd->node_id == NODE_ID_TOP && cmd->cmd == NODE_CMD_WAKE_DISPLAY)
        s_wake_requested = true;
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
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(40));   // 10 dBm
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, BROADCAST_MAC, 6);
    peer.channel = 0;
    peer.encrypt = false;
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    ESP_LOGI(TAG, "Wi-Fi / ESP-NOW ready");
}

// ── Display ──────────────────────────────────────────────────
static void draw_screen(float temp)
{
    if (canvas.getBuffer() == nullptr) return;

    const int cx = SCREEN_W / 2;

    canvas.fillScreen(TFT_BLACK);

    canvas.setTextDatum(lgfx::top_center);
    canvas.setTextSize(2);
    canvas.setTextColor(0xFD20u, TFT_BLACK);   // orange
    canvas.drawString("BOTTOM NODE", cx, 10);
    canvas.drawFastHLine(0, 35, SCREEN_W, TFT_DARKGREY);

    canvas.setTextDatum(lgfx::top_center);
    canvas.setTextSize(2);
    canvas.setTextColor(TFT_WHITE, TFT_BLACK);
    canvas.drawString("T5:", cx, 45);

    canvas.setTextDatum(lgfx::middle_center);
    canvas.setTextSize(4);
    if (isnan(temp)) {
        canvas.setTextColor(TFT_RED, TFT_BLACK);
        canvas.drawString("ERR", cx, 110);
    } else {
        canvas.setTextColor(TFT_GREEN, TFT_BLACK);
        char buf[16];
        snprintf(buf, sizeof(buf), "%.2f C", (double)temp);
        canvas.drawString(buf, cx, 110);
    }

    canvas.pushSprite(0, 0);
}

static void draw_splash(const char* line1, const char* line2)
{
    if (canvas.getBuffer() == nullptr) return;
    canvas.fillScreen(TFT_BLACK);
    canvas.setTextDatum(lgfx::middle_center);
    canvas.setTextSize(2);
    canvas.setTextColor(0xFD20u, TFT_BLACK);
    canvas.drawString(line1, SCREEN_W / 2, SCREEN_H / 2 - 14);
    canvas.setTextSize(1);
    canvas.setTextColor(TFT_DARKGREY, TFT_BLACK);
    canvas.drawString(line2, SCREEN_W / 2, SCREEN_H / 2 + 10);
    canvas.pushSprite(0, 0);
}

// ── Main task (5 Hz) ─────────────────────────────────────────
static void main_task(void* /*arg*/)
{
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        // ── 1. Handle wake command от node_top ───────────────
        if (s_wake_requested) {
            s_wake_requested = false;
            last_activity_ms = esp_timer_get_time() / 1000LL;
            if (display_sleeping) {
                display_wake();
                ESP_LOGI(TAG, "Display woke by node_top");
            }
        }

        // ── 2. Read sensor ───────────────────────────────────
        uint32_t raw = max31855_read_raw();
        float temp   = max31855_parse(raw);

        // ── 3. Update display (пропустить если спит) ─────────
        if (!display_sleeping) {
            draw_screen(temp);
        }

        // ── 4. Send ESP-NOW packet ───────────────────────────
        espnow_packet_t pkt = {};
        pkt.node_id      = NODE_ID_BOTTOM;
        pkt.timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
        pkt.temps[0]     = temp;
        pkt.num_sensors  = 1;
        pkt.status       = isnan(temp) ? STATUS_ERR_SENS : STATUS_OK;

        esp_err_t ret = esp_now_send(BROADCAST_MAC,
                                     (const uint8_t*)&pkt, sizeof(pkt));
        if (ret != ESP_OK)
            ESP_LOGW(TAG, "esp_now_send: %s", esp_err_to_name(ret));

        ESP_LOGI(TAG, "T5=%.2f°C", (double)temp);

        // ── 5. Auto-sleep check ──────────────────────────────
        int64_t now_ms = esp_timer_get_time() / 1000LL;
        if (!display_sleeping &&
            (now_ms - last_activity_ms >= DISPLAY_SLEEP_TIMEOUT_MS)) {
            display_sleep_start();
            ESP_LOGI(TAG, "Display sleep (inactivity)");
        }

        // ── 6. Precise 200 ms period ─────────────────────────
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(200));
    }
}

// ── Entry point ──────────────────────────────────────────────
extern "C" void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(1500));
    ESP_LOGI(TAG, "=== Node Bottom starting ===");

    last_activity_ms = esp_timer_get_time() / 1000LL;

    // ── Step 1: Sensor GPIO ──────────────────────────────────
    spi_sens_init();
    ESP_LOGI(TAG, "Sensor SPI init OK");

    // ── Step 2: Display power + hardware reset ───────────────
    gpio_set_direction(LCD_BL_PIN,  GPIO_MODE_OUTPUT);
    gpio_set_level    (LCD_BL_PIN,  1);                // backlight on

    gpio_set_direction(LCD_RST_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level    (LCD_RST_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level    (LCD_RST_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(150));
    ESP_LOGI(TAG, "Display HW reset done");

    // ── Step 3: LovyanGFX init ───────────────────────────────
    lcd.init();
    lcd.setRotation(1);

    canvas.setColorDepth(8);
    if (canvas.createSprite(SCREEN_W, SCREEN_H) == nullptr) {
        ESP_LOGE(TAG, "Sprite alloc FAILED – halting");
        for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "Sprite OK (%d×%d, 8bpp)", SCREEN_W, SCREEN_H);

    // ── Step 4: Boot splash ──────────────────────────────────
    draw_splash("BOTTOM NODE", "Initialising...");

    // ── Step 5: Wi-Fi / ESP-NOW ──────────────────────────────
    wifi_espnow_init();

    // ── Step 6: First real sensor frame ──────────────────────
    draw_screen(max31855_parse(max31855_read_raw()));

    // ── Step 7: Main loop ────────────────────────────────────
    ESP_LOGI(TAG, "Launching main task");
    xTaskCreate(main_task, "main_task", 8192, nullptr, 5, nullptr);
}
