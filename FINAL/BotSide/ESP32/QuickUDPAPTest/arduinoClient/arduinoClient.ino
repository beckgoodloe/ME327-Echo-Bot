#include <WiFi.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>
#include <OSCData.h>
#include <DNSServer.h>

#define BUILTIN_LED 12
#define PACKET_LED 5

const byte   DNS_PORT  =   53;

WiFiUDP Udp;
DNSServer dnsServer; 

const IPAddress local(11, 11, 11, 11);
const IPAddress dns(11, 11, 11, 11);
const IPAddress gateway(11, 11, 11, 1); /* not used */
const IPAddress netmask(255, 255, 255, 0);

const unsigned int localPort = 8888;

OSCErrorCode error;

unsigned int ledState = LOW;

void setup() 
{

  Serial.begin(115200);
   
  pinMode(BUILTIN_LED, OUTPUT);
  pinMode(PACKET_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, ledState);
  digitalWrite(BUILTIN_LED, HIGH);

  dnsServer.start(DNS_PORT, "*", local); 

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(local, dns, netmask);
  WiFi.softAP("UDP-RECEIVER");
  WiFi.begin();

  Udp.begin(localPort);

} 

void led(OSCMessage &msg) 
{
  ledState = msg.getInt(0);
  digitalWrite(BUILTIN_LED, ledState);
  Serial.print("/led: ");
  Serial.println(ledState);
}

void loop() 
{
  dnsServer.processNextRequest();
  delay(0);
  
  OSCMessage oscMsg;
  int size = Udp.parsePacket();

  if (size > 0) 
  {
    Serial.print((String)"Request from " + Udp.remoteIP().toString() + ", port " + Udp.remotePort());

    // short blink to show some data was received
    digitalWrite(BUILTIN_LED, LOW);
    delay(50);
    digitalWrite(BUILTIN_LED, HIGH);
    
    while (size--) 
    {
      oscMsg.fill(Udp.read());
    }
    if (!oscMsg.hasError()) 
    {
      oscMsg.dispatch("/led", led);
    } 
    else 
    {
      error = oscMsg.getError();
      Serial.print("error: ");
      Serial.println(error);
    }
  }
}
