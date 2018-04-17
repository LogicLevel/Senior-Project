#include "Arduino.h"
#include "DisplayModule.h"


DisplayModule::DisplayModule(SSD1306 *disp, byte *wifi, byte *blutooth, byte *battery, Frame *home){
  display = disp;

  wifiActive = wifi;
  blutoothActive = blutooth;
  batteryStatus = battery;
  homeFrame = home;
  activeFrame = home;
}

void DisplayModule::drawDisplay(){
  // draw fram components here!!
  drawIndicators();
  drawIcons();
}

#define horizontal_x 48
#define vertial_y 40
void DisplayModule::drawIndicators(){
  // check if poiter is not NULL
  if (activeFrame) {
    // Horizontal Icons
    if (activeFrame->front != NULL)
      display->drawXbm(horizontal_x, 0, indicator_horizonal_w, indicator_horizonal_h, indicator_horizonal_filled_bits);
    else
      display->drawXbm(horizontal_x, 59, indicator_horizonal_w, indicator_horizonal_h, indicator_horizonal_bits);
    if (activeFrame->back != NULL)
      display->drawXbm(horizontal_x, 0, indicator_horizonal_w, indicator_horizonal_h, indicator_horizonal_filled_bits);
    else
      display->drawXbm(horizontal_x, 59, indicator_horizonal_w, indicator_horizonal_h, indicator_horizonal_bits);

    // Vertical Icons
    if (activeFrame->right != NULL)
      display->drawXbm(122, vertial_y, indicator_vertical_w, indicator_vertical_h, indicator_vertical_filled_bits);
    else
      display->drawXbm(122, vertial_y, indicator_vertical_w, indicator_vertical_h, indicator_vertical_bits);
    if (activeFrame->left != NULL)
      display->drawXbm(0, vertial_y, indicator_vertical_w, indicator_vertical_h, indicator_vertical_filled_bits);
    else
      display->drawXbm(0, vertial_y, indicator_vertical_w, indicator_vertical_h, indicator_vertical_bits);
  }
}

void DisplayModule::drawIcons(){
  if (wifiActive != NULL)
    display->drawXbm(108, 0, wifi_w, wifi_h, wifi_bits);
  if (blutoothActive != NULL)
    display->drawXbm(92, 0, BT_w, BT_h, BT_bits);
}
