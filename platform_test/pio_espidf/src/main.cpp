#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <LovyanGFX.hpp>

static const char* TAG = "LCD_TEST";

// ========== Конфигурация дисплея ESP32-C6-LCD-1.47 ==========
class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ST7789 _panel_instance;
  lgfx::Bus_SPI      _bus_instance;
  lgfx::Light_PWM    _light_instance;

public:
  LGFX() {
    { auto cfg = _bus_instance.config();
      cfg.spi_host   = SPI2_HOST;
      cfg.spi_mode   = 0;
      cfg.freq_write = 80000000;
      cfg.freq_read  = 20000000;
      cfg.spi_3wire  = false;
      cfg.use_lock   = true;
      cfg.dma_channel = SPI_DMA_CH_AUTO;
      cfg.pin_sclk   = 6;
      cfg.pin_mosi   = 7;
      cfg.pin_miso   = -1;
      cfg.pin_dc     = 15;
      _bus_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance);
    }
    { auto cfg = _panel_instance.config();
      cfg.pin_cs     = 14;
      cfg.pin_rst    = 1;
      cfg.pin_busy   = -1;
      cfg.panel_width  = 172;
      cfg.panel_height = 320;
      cfg.offset_x     = 34;
      cfg.offset_y     = 0;
      cfg.invert       = true;
      cfg.readable     = false;
      cfg.rgb_order    = false;
      cfg.dlen_16bit   = false;
      cfg.bus_shared   = false;
      _panel_instance.config(cfg);
    }
    { auto cfg = _light_instance.config();
      cfg.pin_bl      = 22;
      cfg.invert      = false;
      cfg.freq        = 44100;
      cfg.pwm_channel = 7;
      _light_instance.config(cfg);
      _panel_instance.setLight(&_light_instance);
    }
    setPanel(&_panel_instance);
  }
};

static LGFX lcd;

extern "C" void app_main() {
  ESP_LOGI(TAG, "ESP32-C6 LCD Test - PlatformIO + ESP-IDF");

  lcd.init();
  lcd.setRotation(0);
  lcd.fillScreen(TFT_BLACK);

  lcd.setTextColor(TFT_ORANGE, TFT_BLACK);
  lcd.setTextSize(2);
  lcd.setCursor(10, 10);
  lcd.println("Hello World!");

  lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  lcd.setTextSize(1);
  lcd.setCursor(10, 50);
  lcd.println("PlatformIO + ESP-IDF");
  lcd.setCursor(10, 65);
  lcd.println("ESP32-C6-LCD-1.47");
  lcd.setCursor(10, 80);
  lcd.println("LovyanGFX OK");

  lcd.fillRect(10,  110, 40, 40, TFT_RED);
  lcd.fillRect(60,  110, 40, 40, TFT_GREEN);
  lcd.fillRect(110, 110, 40, 40, TFT_BLUE);

  ESP_LOGI(TAG, "Display initialized OK");

  int cnt = 0;
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    lcd.setCursor(10, 200);
    lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
    lcd.printf("Uptime: %d s   ", ++cnt);
    ESP_LOGI(TAG, "Tick: %d", cnt);
  }
}
