#include <SSD1306.h>
#include <Wire.h>  // Only needed for Arduino 1.6.5 and earlier
#include "DisplayModule.h"
#include "Frame.h"


SSD1306 display(0x3c, 23, 22);
byte wifiStat = 0;
byte blutoothStat = 0;
byte batteryStat = 0;

// declare all frame object and link them together here
Frame HomeFrame = Frame("home");

// gesture vector

DisplayModule DM = DisplayModule(&display, &wifiStat, &blutoothStat, &batteryStat, &HomeFrame);

void setup() {
  // put your setup code here, to run once:
  display.init();
  display.flipScreenVertically();
  Serial.begin(115200);
  DM.drawDisplay();
  display.display();
}

void loop() {
  // put your main code here, to run repeatedly:

}

byte hue = 0;

homeFrame.toggle = &hue;
if (hue == 1){

}
