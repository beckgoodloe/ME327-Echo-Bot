import oscP5.*;
import netP5.*;

OscP5 osc;
NetAddress esp8266;
boolean pressed = true;

void setup()
{
  osc = new OscP5(this, 12000);
  esp8266 = new NetAddress("11.11.11.11", 8888);
  
  size(100, 100); // click in this square to send an OCS message

}

void draw() {
  OscMessage msg = new OscMessage("/led").add((pressed)?1:0);
  osc.send(msg, esp8266);
  pressed = !pressed;
  println("Pressed");
  delay(1000);
}
