#line 1 "D:\\job\\28\\MimoFace_src\\MimoFace\\tft_setup.h"
// TFT_eSPI setup for LCDWiki ES3C28P (ESP32-S3, 2.8" IPS 240x320, ILI9341V, 4-wire SPI).
//
// TFT_eSPI 2.5.x loads this file automatically from the sketch folder
// (TFT_eSPI.h: __has_include(<tft_setup.h>)), so the library's own
// User_Setup.h / User_Setup_Select.h do not need to be edited.
//
// Pin source: LCDWiki "2.8inch ESP32-S3 Display" (ES3C28P), section
// "ESP32 Pin Parameters": https://www.lcdwiki.com/2.8inch_ESP32-S3_Display
// The same pins, 40 MHz SPI, colour order and inversion run on this board
// in the XiaoZhi firmware (board quandong/esp32-s3-spi-2.8, lcdwiki-es3c28p).

#define USER_SETUP_ID 280
#define USER_SETUP_INFO "ES3C28P_ILI9341_SPI"

#define ILI9341_DRIVER

#define TFT_WIDTH  240
#define TFT_HEIGHT 320

// IPS panel: colours are inverted at the controller, RGB element order.
#define TFT_INVERSION_ON
#define TFT_RGB_ORDER TFT_RGB

// SPI bus (LCDWiki: IO12 SCLK, IO11 MOSI, IO13 MISO, IO10 CS low active,
// IO46 DC high = data / low = command).
#define TFT_MISO 13
#define TFT_MOSI 11
#define TFT_SCLK 12
#define TFT_CS   10
#define TFT_DC   46
// LCD reset is not a GPIO: it is wired to the ESP32-S3 EN (chip reset) line.
#define TFT_RST  -1

// Backlight: IO45, high level = on.
#define TFT_BL   45
#define TFT_BACKLIGHT_ON HIGH

// ESP32-S3 with Arduino-ESP32 3.x: the default FSPI port mapping in
// TFT_eSPI 2.5.x crashes at init; use the HSPI (SPI3) peripheral.
// The pins above are routed through the GPIO matrix, so any SPI host works.
#define USE_HSPI_PORT

#define LOAD_GLCD
#define LOAD_FONT2
#define SMOOTH_FONT

#define SPI_FREQUENCY      40000000
#define SPI_READ_FREQUENCY 16000000
