#ifndef DisplayModule_h
#define DisplayModule_h

#include "Arduino.h"
#include "Frame.h"
#include <Wire.h>
#include "SSD1306.h"
#include "images.h"

  class DisplayModule
  {
    public:
      DisplayModule(SSD1306 *disp);

      // pointers to fram links
      Frame *activeFrame = NULL;
      Frame *homeFrame = NULL;

      // signal stength is 0 to 3
      byte *wifiActive = 0;
      byte *blutoothActive = 0;
      byte *batteryStatus = 0;

      // draws the current frame, indicators, and icons
      void drawDisplay();

    private:
      // Initialize the OLED display using Wire library
      SSD1306  *display(0x3c, 23, 22);

      // draws all inicator blocks
      //   the Frame pointers of the active frame module
      //   to determin indicator state
      void drawIndicators();

      // draws the blutooth wifi and battery icons
      void drawIcons();



  };



#endif
