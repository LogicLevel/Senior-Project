#include <SSD1306.h>
#include <Wire.h>  // Only needed for Arduino 1.6.5 and earlier
#include "DisplayModule.h"
#include "Frame.h"
#include "Gesture.h"
#include "Haptic.h"


SSD1306 display(0x3c, 23, 22);
byte wifiStat = 0;
byte blutoothStat = 0;
byte batteryStat = 0;

// declare all frame object and link them together here
Frame homeFrame = Frame("home");

Frame hue = Frame("Hue");
Frame light1 = Frame("Light1");
Frame light2 = Frame("Light2");

// Create gesture object
Gesture gesture = Gesture();
Haptic haptic = Haptic(22);

// gesture vector
DisplayModule DM = DisplayModule(&display, &wifiStat, &blutoothStat, &batteryStat, &homeFrame, &gesture, &haptic);

const int b1 = A0;
const int b2 = A1;
const int b3 = A2;
const int b4 = A3;
const int b5 = A4;
const int b6 = A5;

int b1_state = 1;
int b2_state = 1;
int b3_state = 1;
int b4_state = 1;
int b5_state = 1;
int b6_state = 1;


void setup() {
  delay(500);
  Serial.begin(115200);
  // put your setup code here, to run once:
  linkFrames();
  DM.setup();
  haptic.setup();

  // these are used for testing gestures with a button
  // declare inputs and set pull up resistors
  pinMode(b1, INPUT);
  pinMode(b2, INPUT);
  pinMode(b3, INPUT);
  pinMode(b4, INPUT);
  pinMode(b5, INPUT);
  pinMode(b6, INPUT);


  Serial.println("Setup Complete");
}

void loop() {
  DM.updateDisplay();
  gestureTest();
  haptic.compute(); 

  delay(200);
}

void gestureTest(){
  /*
    up      A0
    down    A1
    left    A2
    right   A3
    select  A4
  */

  if ( (b1_state == 1) && (digitalRead(b1)==0) ){
    gesture.up = 1;
    gesture.flag = 1;
    Serial.println("Flag Set: B1");
  }

  if ( (b2_state == 1) && (digitalRead(b2)==0) ){
    gesture.down = 1;
    gesture.flag = 1;
    Serial.println("Flag Set: B2");
  }

  if ( (b3_state == 1) && (digitalRead(b3)==0) ){
    gesture.left = 1;
    gesture.flag = 1;
    Serial.println("Flag Set: B3");
  }

  if ( (b4_state == 1) && (digitalRead(b4)==0) ){
    gesture.right = 1;
    gesture.flag = 1;
    Serial.println("Flag Set: B4");
  }

  if ( (b5_state == 1) && (digitalRead(b5)==0) ){
    gesture.select = 1;
    gesture.flag = 1;
    Serial.println("Flag Set: B5");
  }

  if ( (b6_state == 1) && (digitalRead(b6)==0) ){
    gesture.home = 1;
    gesture.flag = 1;
    Serial.println("Flag Set: B6");
  }

  if (gesture.flag == 1)
    Serial.print("Flag Set");

  // update button states
  b1_state = digitalRead(b1);
  b2_state = digitalRead(b2);
  b3_state = digitalRead(b3);
  b4_state = digitalRead(b4);
  b5_state = digitalRead(b5);
  b6_state = digitalRead(b6);
}

void linkFrames(){
  // link frames here
  homeFrame.right = &hue;
  hue.left = &homeFrame;

  hue.up = &light1;
  light1.down = &hue;

  light1.up = &light2;
  light2.down = &light1;
}

/*
long printInterval = 2000;
long timer
void debugPrint(){

}
*/
