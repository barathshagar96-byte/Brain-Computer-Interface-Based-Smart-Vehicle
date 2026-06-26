#include <ESP8266WiFi.h>
#include <WiFiUDP.h>
#include <SoftwareSerial.h>

const char* AP_SSID     = "BCI_CAR";
const char* AP_PASSWORD = "bcicar123";

IPAddress carIP(192, 168, 4, 2);
const int UDP_PORT = 4210;

SoftwareSerial arSerial(13, 15);

WiFiUDP udp;

void setup() {
  Serial.begin(115200);
  arSerial.begin(9600);
  delay(100);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());

  udp.begin(UDP_PORT);
  Serial.println("Person ESP Ready");
}

void loop() {
  if (arSerial.available()) {
    char cmd = (char)arSerial.read();
    if (cmd == 'F' || cmd == 'S' || cmd == 'L' || cmd == 'R') {
      udp.beginPacket(carIP, UDP_PORT);
      udp.write((uint8_t)cmd);
      udp.endPacket();
      Serial.print("UDP TX: "); Serial.println(cmd);
    }
  }
}
