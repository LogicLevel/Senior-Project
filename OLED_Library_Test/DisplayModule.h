#ifndef DisplayModule_h
#define DisplayModule_h

#include "Arduino.h"
#include "Frame.h"
#include <Wire.h>
#include "SSD1306.h"
#include "images.h"
#include "Gesture.h"

  class DisplayModule
  {
    public:
      DisplayModule(SSD1306 *disp, byte *wifi, byte *blutooth, byte *battery, Frame *home, Gesture *g);

      // set up Display with f as the homeframe
      void setup();

      // pointers to fram links
      Frame *activeFrame = NULL;
      Frame *homeFrame = NULL;

      // signal stength is 0 to 3
      byte *wifiActive = 0;
      byte *blutoothActive = 0;
      byte *batteryStatus = 0;

      void updateDisplay();

    private:
      // Initialize the OLED display using Wire library
      SSD1306  *display;
      Gesture *gesture;

      // ****** DISPLAY FUNCTIONS ****** //
      // draws all inicator blocks
      //   the Frame pointers of the active frame module
      //   to determin indicator state
      void drawIndicators();

      // draws the current frame, indicators, and icons
      void drawDisplay();

      // draws the blutooth wifi and battery icons
      void drawIcons();

      // call change frame to change the frame displayed
      void changeFrame(Frame *f);

      void processGesture();

      // draws the name of the frame
      void drawText();

      //void variableTilt();
      byte variableSetActive = 0;

  };



#endif
