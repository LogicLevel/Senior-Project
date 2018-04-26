#include "Arduino.h"
#include "Hue.h"

/*
Morse::Morse(int pin)
{
  pinMode(pin, OUTPUT);
  _pin = pin;
}
*/

int Hue::connect()
{
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  //WiFi.status() != WL_CONNECTED
  int i;
  for (i=0; i<5; i=i+1) {
    if (WiFi.status() != WL_CONNECTED){
      delay(500);
      Serial.println("Connecting to WiFi..");
    } else {
      Serial.println("Connected to the WiFi network");
      return 1;
    }
  }
  Serial.println("Failed to connect to WiFi network");
}

void Hue::scanNetworks() {
  int numberOfNetworks = WiFi.scanNetworks();

  Serial.print("Number of networks found: ");
  Serial.println(numberOfNetworks);

  for (int i = 0; i < numberOfNetworks; i++) {

    Serial.print("Network name: ");
    Serial.println(WiFi.SSID(i));

    Serial.print("Signal strength: ");
    Serial.println(WiFi.RSSI(i));

    Serial.print("MAC address: ");
    Serial.println(WiFi.BSSIDstr(i));

    Serial.println("-----------------------");

  }
}

boolean Hue::setHue(int lightNum,String command){
  if (client.connect(hueHubIP, hueHubPort))
  {
    while (client.connected())
    {
      client.print("PUT /api/");
      client.print(hueUsername);
      client.print("/lights/");
      client.print(lightNum);  // hueLight zero based, add 1
      client.println("/state HTTP/1.1");
      client.println("keep-alive");
      client.print("Host: ");
      client.println(hueHubIP);
      client.print("Content-Length: ");
      client.println(command.length());
      client.println("Content-Type: text/plain;charset=UTF-8");
      client.println();  // blank line before body
      client.println(command);  // Hue command
    }
    client.stop();
    return true;  // command executed
  }
  else
    return false;  // command failed
}

boolean Hue::turnOnLight(int light){
  String command = "{\"on\": true,\"bri\":255}";
  return setHue(light,command);
}

boolean Hue::setBrightness(int light, int brightness){
  // turn the light off if brightness is zero
  if (brightness == 0) {
    return turnOffLight(light);
  } else {
    String command = "{\"on\": true,\"bri\":";
    String bright_str = String(brightness);
    command = String(command + bright_str + "}");
    return setHue(light,command);
  }
}

boolean Hue::turnOffLight(int light){
  String command = "{\"on\": false,\"bri\":0}";
  return setHue(light,command);
}
