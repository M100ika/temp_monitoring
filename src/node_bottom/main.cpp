/**
 * Node Bottom Firmware  —  ESP32-C6
 * 1× MAX31855 (soft-SPI) | LCD 1.47" ST7789 landscape via LGFX_Sprite | ESP-NOW TX
 *
 * Waveshare ESP32-C6-LCD-1.47 pinout (verified):
 *   LCD  : MOSI=6  SCLK=7  CS=14  DC=15  RST=21  BL=22
 *   Sensor soft-SPI : CLK=GPIO8  MISO=GPIO9  CS=GPIO18
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

// ── Wi-Fi / ESP-NOW ──────────────────────────────────────────
static void wifi_espnow_init(void)
{
    // NVS: erase and retry if partition is corrupt or version-mismatched
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

// ── Display (sprite → pushSprite, no flicker) ────────────────
static void draw_screen(float temp)
{
    if (canvas.getBuffer() == nullptr) return;

    const int cx = SCREEN_W / 2;

    canvas.fillScreen(TFT_BLACK);

    // Header
    canvas.setTextDatum(lgfx::top_center);
    canvas.setTextSize(2);
    canvas.setTextColor(0xFD20u, TFT_BLACK);   // orange
    canvas.drawString("BOTTOM NODE", cx, 10);
    canvas.drawFastHLine(0, 35, SCREEN_W, TFT_DARKGREY);

    // "T5:" label
    canvas.setTextDatum(lgfx::top_center);
    canvas.setTextSize(2);
    canvas.setTextColor(TFT_WHITE, TFT_BLACK);
    canvas.drawString("T5:", cx, 45);

    // Large temperature value
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
    canvas.setTextColor(0xFD20u, TFT_BLACK);   // orange
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
        uint32_t raw = max31855_read_raw();
        float temp   = max31855_parse(raw);

        draw_screen(temp);

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

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(200));
    }
}

// ── Entry point ──────────────────────────────────────────────
extern "C" void app_main(void)
{
    // ── Step 0: Let USB-CDC monitor attach ───────────────────
    // ESP32-C6 USB-Serial/JTAG needs ~1 s for the host driver to
    // enumerate and the terminal to connect before any log output.
    vTaskDelay(pdMS_TO_TICKS(1500));
    ESP_LOGI(TAG, "=== Node Bottom starting ===");

    // ── Step 1: Sensor GPIO ──────────────────────────────────
    spi_sens_init();
    ESP_LOGI(TAG, "Sensor SPI init OK");

    // ── Step 2: Display power + hardware reset ───────────────
    // Drive BL and RST directly so the display is fully ready
    // before handing control to LovyanGFX.
    gpio_set_direction(LCD_BL_PIN,  GPIO_MODE_OUTPUT);
    gpio_set_level    (LCD_BL_PIN,  1);                // backlight on

    gpio_set_direction(LCD_RST_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level    (LCD_RST_PIN, 0);                // assert reset
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level    (LCD_RST_PIN, 1);                // release reset
    vTaskDelay(pdMS_TO_TICKS(150));                    // ST7789 power-on time
    ESP_LOGI(TAG, "Display HW reset done");

    // ── Step 3: LovyanGFX init ───────────────────────────────
    lcd.init();
    lcd.setRotation(1);   // landscape → 320×172

    // 8-bit depth: 320×172×1 = 54 KB vs 107 KB for 16-bit.
    // Prevents heap panic on 320 KB RAM chip.
    canvas.setColorDepth(8);
    if (canvas.createSprite(SCREEN_W, SCREEN_H) == nullptr) {
        ESP_LOGE(TAG, "Sprite alloc FAILED – halting");
        for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "Sprite OK (%d×%d, 8bpp)", SCREEN_W, SCREEN_H);

    // ── Step 4: Boot splash ──────────────────────────────────
    draw_splash("BOTTOM NODE", "Initialising...");
    ESP_LOGI(TAG, "Splash displayed");

    // ── Step 5: Wi-Fi / ESP-NOW ──────────────────────────────
    // Done after display: if this hangs, the splash is visible.
    ESP_LOGI(TAG, "Starting Wi-Fi...");
    wifi_espnow_init();

    // ── Step 6: First real sensor frame ──────────────────────
    uint32_t raw = max31855_read_raw();
    draw_screen(max31855_parse(raw));

    // ── Step 7: Main loop ────────────────────────────────────
    ESP_LOGI(TAG, "Launching main task");
    xTaskCreate(main_task, "main_task", 8192, nullptr, 5, nullptr);
}
