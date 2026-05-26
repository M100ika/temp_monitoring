#pragma once
#include <LovyanGFX.hpp>

// ST7735S 1.8" TFT на ESP32-WROOM-32
// VSPI (SPI2_HOST): MOSI=23, CLK=18, CS=5, DC=22, RST=4, BL=21
//
// Если цвета или границы картинки едут — подберите offset_x / offset_y:
//   Стандартный вариант:   memory_width=132, memory_height=162, offset_x=2, offset_y=1
//   «Зелёная вкладка»:     memory_width=128, memory_height=160, offset_x=0, offset_y=0

class LGFX : public lgfx::LGFX_Device {
    lgfx::Panel_ST7735S _panel;
    lgfx::Bus_SPI       _bus;
    lgfx::Light_PWM     _light;

public:
    LGFX() {
        {
            auto cfg = _bus.config();
            cfg.spi_host   = SPI2_HOST;
            cfg.spi_mode   = 0;
            cfg.freq_write = 27000000;  // ST7735S max ~27 MHz
            cfg.pin_sclk   = 18;
            cfg.pin_mosi   = 23;
            cfg.pin_miso   = -1;
            cfg.pin_dc     = 22;
            _bus.config(cfg);
            _panel.setBus(&_bus);
        }
        {
            auto cfg = _panel.config();
            cfg.pin_cs        = 5;
            cfg.pin_rst       = 4;
            cfg.panel_width   = 128;
            cfg.panel_height  = 160;
            cfg.memory_width  = 132;   // подобрать если картинка обрезается
            cfg.memory_height = 162;
            cfg.offset_x      = 2;     // подобрать если картинка смещена
            cfg.offset_y      = 1;
            cfg.invert        = false;
            cfg.rgb_order     = false;
            _panel.config(cfg);
        }
        {
            auto cfg = _light.config();
            cfg.pin_bl      = 21;
            cfg.invert      = false;
            cfg.freq        = 44100;
            cfg.pwm_channel = 7;
            _light.config(cfg);
            _panel.setLight(&_light);
        }
        setPanel(&_panel);
    }
};
