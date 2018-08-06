#include <ESP8266WiFi.h>
#include <Arduino.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include "AccelStepper.h"

#define WIFI_SSID "cAP"
#define WIFI_PASSWORD "password"

WiFiUDP Udp;
WiFiUDP UdpSend;
unsigned int localUdpPort = 4210;
unsigned int remoteUdpPort = 4211;  // local port to listen on
char  incomingPacket[255];  // buffer for incoming packets




IPAddress ip(192, 168, 5, 15); // this 3 lines for a fix IP-address
IPAddress gateway(192, 168, 5, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(192, 168, 5, 1);

AccelStepper stepper1(AccelStepper::DRIVER, 13, 12);

void setup() {

  Serial.begin(9600);

  stepper1.setMaxSpeed(5000.0);
  stepper1.setSpeed(5000.0);
  stepper1.setAcceleration(300.0);
  stepper1.moveTo(1000);


  // connect to wifi.
  WiFi.config(ip,gateway,subnet,dns);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("connecting");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("connected: ");
  Serial.println(WiFi.localIP());

  Udp.begin(localUdpPort);
  UdpSend.begin(remoteUdpPort);
  Serial.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), localUdpPort);
  digitalWrite(16, LOW);
}

  void loop() {
   
    stepper1.run();
 
    int packetSize = Udp.parsePacket();
    if (packetSize)
    {
      // receive incoming UDP packets
     // Serial.printf("Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
      int len = Udp.read(incomingPacket, 255);
      if (len > 0)
      {
        incomingPacket[len] = 0;
      }
      Serial.printf("UDP packet contents: %s\n", incomingPacket);

     StaticJsonBuffer<300> JSONBuffer;   //Memory pool
     JsonObject& parsed = JSONBuffer.parseObject(incomingPacket);

     if (!parsed.success()) {   //Check for errors in parsing
       Serial.println("Parsing failed");
         return;
     }

     int CountOfSteps = parsed["To"];
     int Acceleration = parsed["Acceleration"];
     int Speed = parsed["Speed"];
     int MaxSpeed = parsed["MaxSpeed"];

     Serial.println(CountOfSteps);
     Serial.println(Acceleration);
     Serial.println(Speed);
     Serial.println(MaxSpeed);

     JsonObject& sendData = JSONBuffer.createObject();
     sendData["To"] = CountOfSteps;
     sendData["Acceleration"] = Acceleration;
     sendData["Speed"] = Speed;
     sendData["MaxSpeed"] = MaxSpeed;

      UdpSend.beginPacket(Udp.remoteIP(), remoteUdpPort);
      sendData.printTo(UdpSend);
      UdpSend.println();
      UdpSend.endPacket();
      stepper1.setCurrentPosition(0);  
   
      stepper1.setMaxSpeed(MaxSpeed);
      stepper1.setSpeed(Speed);
      stepper1.setAcceleration(Acceleration);
      stepper1.moveTo(CountOfSteps); 
    }

  }