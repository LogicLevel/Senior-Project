#include <WiFi.h>

const char* ssid = "Coda b/g/n 2.4 GHz";
const char* password =  "westbattledozen4";

//{"devicetype":"test user","username":"newdeveloper"}
//devicetype = esp32
// generated username 5Wp1uZobMvFdn91W9-8NHqagJYwzBvgHNvxXHEI3
const char hueHubIP[] = "192.168.0.150";  // Hue hub IP
const char hueUsername[] = "5Wp1uZobMvFdn91W9-8NHqagJYwzBvgHNvxXHEI3";  // Hue username
const int hueHubPort = 80;

WiFiClient client;


void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);

  scanNetworks();
  delay(1000);
 
  WiFi.begin(ssid, password);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
 
  Serial.println("Connected to the WiFi network");
}

String command;
void loop() {
  // put your main code here, to run repeatedly:

  if(digitalRead(13)) {
    command = "{\"on\": true,\"bri\":255}";
    setHue(1,command);
    setHue(2,command);
  } else {
    command = "{\"on\": false,\"bri\":0}";
    setHue(1,command);
    setHue(2,command);
  }
  delay(10);
}

void scanNetworks() {
 
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

boolean setHue(int lightNum,String command)
{
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
