#pragma once
#include <LovyanGFX.hpp>

class LGFX : public lgfx::LGFX_Device {
    lgfx::Panel_ST7789 _panel;
    lgfx::Bus_SPI      _bus;
public:
    LGFX() {
        {   // Шина SPI
            auto cfg = _bus.config();
            cfg.spi_host    = SPI2_HOST;
            cfg.spi_mode    = 0;
            cfg.freq_write  = 40000000; // 40MHz — стабильно и быстро
            cfg.pin_sclk    = 7;        // Твой рабочий SCK
            cfg.pin_mosi    = 6;        // Твой рабочий MOSI
            cfg.pin_miso    = -1;
            cfg.pin_dc      = 15;
            _bus.config(cfg);
            _panel.setBus(&_bus);
        }
        {   // Панель ST7789
            auto cfg = _panel.config();
            cfg.pin_cs       = 14;
            cfg.pin_rst      = 21;      // RST строго на 21!
            cfg.panel_width  = 172;
            cfg.panel_height = 320;
            cfg.offset_x     = 34;
            cfg.offset_y     = 0;
            cfg.invert       = true;    // Для ST7789 Waveshare обычно true
            cfg.rgb_order    = false;
            _panel.config(cfg);
        }
        setPanel(&_panel);
    }
};