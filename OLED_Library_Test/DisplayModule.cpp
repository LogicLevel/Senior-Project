#include "Arduino.h"
#include "DisplayModule.h"


DisplayModule::DisplayModule(SSD1306 *disp, byte *wifi, byte *blutooth, byte *battery, Frame *home, Gesture *g){
  // store all pointer data
  display = disp;
  wifiActive = wifi;
  blutoothActive = blutooth;
  batteryStatus = battery;
  homeFrame = home;
  activeFrame = home;
  gesture = g;
}

void DisplayModule::setup(){
  // set up the SSD1306 library
  display->init();
  display->flipScreenVertically();
  display->setTextAlignment(TEXT_ALIGN_CENTER);
  display->setFont(ArialMT_Plain_24);
  drawDisplay();
}

void DisplayModule::drawDisplay(){
  // draw fram components here!!
  drawIndicators();
  drawIcons();
  drawText();
  display->display();
}

void DisplayModule::updateDisplay(){
  if (gesture->flag)
    processGesture();
}

#define horizontal_x 48
#define vertial_y 24
void DisplayModule::drawIndicators(){
  // check if poiter is not NULL
  Serial.println("drawIndicators");
  if (activeFrame != NULL) {
    Serial.println("frame active");
    // Horizontal Icons
    if (activeFrame->up != NULL)
      display->drawXbm(horizontal_x, 0, indicator_horizonal_w, indicator_horizonal_h, indicator_horizonal_filled_bits);
    else
      display->drawXbm(horizontal_x, 0, indicator_horizonal_w, indicator_horizonal_h, indicator_horizonal_bits);
    if (activeFrame->down != NULL)
      display->drawXbm(horizontal_x, 56, indicator_horizonal_w, indicator_horizonal_h, indicator_horizonal_filled_bits);
    else
      display->drawXbm(horizontal_x, 56, indicator_horizonal_w, indicator_horizonal_h, indicator_horizonal_bits);

    // Vertical Icons
    if (activeFrame->right != NULL)
      display->drawXbm(118, vertial_y, indicator_vertical_w, indicator_vertical_h, indicator_vertial_filled_bits);
    else
      display->drawXbm(118, vertial_y, indicator_vertical_w, indicator_vertical_h, indicator_vertical_bits);
    if (activeFrame->left != NULL)
      display->drawXbm(0, vertial_y, indicator_vertical_w, indicator_vertical_h, indicator_vertial_filled_bits);
    else
      display->drawXbm(0, vertial_y, indicator_vertical_w, indicator_vertical_h, indicator_vertical_bits);
  }
}

void DisplayModule::drawIcons(){
  if (wifiActive != NULL)
    display->drawXbm(108, 0, wifi_w, wifi_h, wifi_bits);
  if (blutoothActive != NULL)
    display->drawXbm(92, 0, BT_w, BT_h, BT_bits);

  // draw battery Icon
  if (*batteryStatus == 0)
    display->drawXbm(0, 0, battery_25_w, battery_25_h, battery_25_bits);
  else if (*batteryStatus == 1)
    display->drawXbm(0, 0, battery_50_w, battery_50_h, battery_50_bits);
  else if (*batteryStatus == 2)
    display->drawXbm(0, 0, battery_75_w, battery_75_h, battery_75_bits);
  else if (*batteryStatus == 3)
    display->drawXbm(0, 0, battery_100_w, battery_100_h, battery_100_bits);
}

# define display_length 4
void DisplayModule::drawText(){
  Serial.print("Drawing Text:  ");
  display->drawString(64, 20, activeFrame->name);
}

void DisplayModule::changeFrame(Frame *f){
  // pointer to new frame that will be the active frame
  activeFrame = f;
  // the display buffer must be cleared before new data is loaded
  display->clear();
  // redraw the display with the new activeFrame
  drawDisplay();
}

void DisplayModule::processGesture(){
  Serial.println("processing gresture");
  // check to see if a new gesture has been registered
  // reset flag
  gesture->flag = 0;
  if (gesture->up) {
    // check if the gesture is valid
    if (activeFrame->up != NULL)
      changeFrame(activeFrame->up);
    else
      // HAPTIC FEADBACK HERE
    gesture->up = 0; // reset flag
  }
  else if (gesture->down) {
    // check if the gesture is valid
    if (activeFrame->down != NULL)
      changeFrame(activeFrame->down);
    else
      // HAPTIC FEADBACK HERE
    gesture->down = 0; // reset flag
  }
  else if (gesture->right) {
    // check if the gesture is valid
    if (activeFrame->right != NULL)
      changeFrame(activeFrame->right);
    else
      // HAPTIC FEADBACK HERE
    gesture->right = 0; // reset flag
  }
  else if (gesture->left) {
    // check if the gesture is valid
    if (activeFrame->left != NULL)
      changeFrame(activeFrame->left);
    else
      // HAPTIC FEADBACK HERE
    gesture->left = 0; // reset flag
  }
  else if (gesture->select) {
    // check if the gesture is valid
    if (activeFrame->select != NULL)
      *activeFrame->select = 1;
    else
      // HAPTIC FEADBACK HERE
    gesture->select = 0; // reset flag
  }
  else if (gesture->home) {
    changeFrame(homeFrame);
    gesture->home = 0; // reset flag
  }
  // reset the flag
}

/*
#define updateInterval 100 // refreshrate for tilt image in ms
long variableTiltCounter = 0;
void DisplayModule::variableTilt(){
  // while update the display until select or cancel is registered
  if ( (gesture.select!=1) && (gesture.cancel!=1) ){

  }
  else if (gesture.select) {
  }
  else if (gesture.cancel) {
  }
}
*/
