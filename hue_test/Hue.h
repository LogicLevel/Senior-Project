#ifndef Hue_h
#define Hue_h

#include "Arduino.h"
#include <WiFi.h>
const char* ssid = "Coda b/g/n 2.4 GHz";
const char* password =  "westbattledozen4";

const char hueHubIP[] = "192.168.0.150";  // Hue hub IP
const char hueUsername[] = "5Wp1uZobMvFdn91W9-8NHqagJYwzBvgHNvxXHEI3";  // Hue username
const int hueHubPort = 80;

  class Hue
  {
    public:
      Hue()
      // prints out a list of all available SSIDs
      void scanNetworks();

      // lighting control methods
      boolean setBrightness(int light, int brightness);
      boolean turnOnLight(int light);
      boolean turnOffLight(int light);

    private:
      WiFiClient client;

      boolean setHue(int lightNum, String command);
  };

#endif
