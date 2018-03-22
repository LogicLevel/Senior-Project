#ifndef DisplayModule_h
#define DisplayModule_h

#include "Arduino.h"
#include "Frame.h"

const char hueUsername[] = "5Wp1uZobMvFdn91W9-8NHqagJYwzBvgHNvxXHEI3";  // Hue username
const int hueHubPort = 80;

#define BT_width 13
#define BT_height 15
static char BT_bits[] = {
  0x80, 0x01, 0xC0, 0x03, 0xC0, 0x0E, 0xC2, 0x1C, 0xCE, 0x0C, 0xDC, 0x07,
  0xF0, 0x03, 0xE0, 0x00, 0xF0, 0x03, 0xDC, 0x07, 0xCE, 0x1C, 0xC2, 0x1C,
  0x80, 0x0F, 0xC0, 0x03, 0xC0, 0x00, };

#define wifi_width 19
#define wifi_height 15
static char wifi_bits[] = {
  0xC0, 0x1F, 0x00, 0xF8, 0xFF, 0x00, 0xFC, 0xFB, 0x01, 0x1F, 0xE0, 0x03,
  0x0F, 0x00, 0x07, 0xC3, 0x1F, 0x06, 0xE0, 0x7F, 0x00, 0xF0, 0x7E, 0x00,
  0x38, 0xE0, 0x00, 0x10, 0x06, 0x00, 0x80, 0x0F, 0x00, 0x80, 0x0F, 0x00,
  0x80, 0x0F, 0x00, 0x80, 0x0F, 0x00, 0x00, 0x07, 0x00, };

#define indicator_horizonal_width 32
#define indicator_horizonal_height 5
static char indicator_horizonal_bits[] = {
  0xFE, 0xFF, 0xFF, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0x03, 0x00, 0x00, 0xC0,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFF, 0xFF, 0x7F, };

#define indicator_horizonal_filled_width 32
#define indicator_horizonal_filled_height 5
static char indicator_horizonal_filled_bits[] = {
  0xFE, 0xFF, 0xFF, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFF, 0xFF, 0x7F, };

#define indicator_vertical_width 6
#define indicator_vertical_height 32
static char indicator_vertical_bits[] = {
  0x0C, 0x1E, 0x1E, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12,
  0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12,
  0x12, 0x12, 0x12, 0x12, 0x12, 0x1E, 0x1E, 0x0C, };

#define indicator_vertial_filled_width 6
#define indicator_vertial_filled_height 32
static char indicator_vertial_filled_bits[] = {
  0x0C, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E,
  0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E,
  0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x0C, };


  class DisplayModule
  {
    public:
      DisplayModule();

      // pointers to fram links
      Frame *activeFrame = NULL;
      Frame *homeFrame = NULL;

      // signal stength is 0 to 3
      byte *wifiActive = 0;
      byte *bluetoothActive = 0;
      byte *batteryStatus = 0;

      // draws the current frame, indicators, and icons
      void drawDisplay();

    private:
      // draws all inicator blocks
      //   the Frame pointers of the active frame module
      //   to determin indicator state
      void drawIndicators();

      // draws the blutooth wifi and battery icons
      void drawIcons();



  };



#endif
