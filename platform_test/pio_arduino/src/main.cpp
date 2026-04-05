#include <Arduino.h>
#include <LovyanGFX.hpp>

// ========== Конфигурация дисплея ESP32-C6-LCD-1.47 (Waveshare) ==========
// ST7789, 172x320, SPI
// GPIO: SCK=6, MOSI=7, CS=14, DC=15, RST=1, BL=22

class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ST7789 _panel_instance;
  lgfx::Bus_SPI      _bus_instance;
  lgfx::Light_PWM    _light_instance;

public:
  LGFX() {
    { // SPI Bus config
      auto cfg = _bus_instance.config();
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
    { // Panel config
      auto cfg = _panel_instance.config();
      cfg.pin_cs     = 14;
      cfg.pin_rst    = 1;
      cfg.pin_busy   = -1;
      cfg.panel_width  = 172;
      cfg.panel_height = 320;
      cfg.offset_x     = 34;
      cfg.offset_y     = 0;
      cfg.offset_rotation = 0;
      cfg.dummy_read_pixel = 8;
      cfg.dummy_read_bits  = 1;
      cfg.readable    = false;
      cfg.invert      = true;
      cfg.rgb_order   = false;
      cfg.dlen_16bit  = false;
      cfg.bus_shared  = false;
      _panel_instance.config(cfg);
    }
    { // Backlight config
      auto cfg = _light_instance.config();
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

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32-C6 LCD Test - PlatformIO + Arduino");

  lcd.init();
  lcd.setRotation(0);
  lcd.fillScreen(TFT_BLACK);

  // Hello World
  lcd.setTextColor(TFT_GREEN, TFT_BLACK);
  lcd.setTextSize(2);
  lcd.setCursor(10, 10);
  lcd.println("Hello World!");

  lcd.setTextColor(TFT_CYAN, TFT_BLACK);
  lcd.setTextSize(1);
  lcd.setCursor(10, 50);
  lcd.println("PlatformIO + Arduino");
  lcd.setCursor(10, 65);
  lcd.println("ESP32-C6-LCD-1.47");
  lcd.setCursor(10, 80);
  lcd.println("LovyanGFX OK");

  // Цветные прямоугольники для проверки RGB
  lcd.fillRect(10,  110, 40, 40, TFT_RED);
  lcd.fillRect(60,  110, 40, 40, TFT_GREEN);
  lcd.fillRect(110, 110, 40, 40, TFT_BLUE);

  Serial.println("Display initialized OK");
}

void loop() {
  static uint32_t t = millis();
  if (millis() - t > 1000) {
    t = millis();
    static int cnt = 0;
    lcd.setCursor(10, 200);
    lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
    lcd.printf("Uptime: %d s   ", ++cnt);
    Serial.printf("Tick: %d\n", cnt);
  }
}
