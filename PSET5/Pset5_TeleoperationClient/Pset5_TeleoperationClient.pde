/*
 * File: Pset5_TeleoperationClient
 * --------------
 * Beck Goodloe, Erick Blankenberg
 * ME327
 * 5/9/2019
 * PSET 5 Question 2
 *
 * Description:
 *   Creates graphics for leader-follower system.
 */

import processing.serial.*;

Serial myPort;   

float inByte_m = 0;
float inByte_s = 0;
float lastByte_m = 0;
float lastByte_s = 0;
void setup (){
  size(600, 400);        
  println(Serial.list());
  myPort = new Serial(this, Serial.list()[13], 115200); 
  myPort.bufferUntil('\n');
  background(0);
}

void draw () {
  background(0);
  
  // Draws leader
  stroke(255,0,0);    
  strokeWeight(4);       
  circle(lastByte_m, height/2, 40);
    
  // Draws follower
  stroke(0,255,0);
  strokeWeight(4);
  circle(lastByte_s, height/2, 40);
  
  // Draws interconnecting line
  stroke (0,0,255);
  strokeWeight(2);
  if(lastByte_m>lastByte_s){
    line(lastByte_s+20,height/2,lastByte_m-20,height/2);
  }else{
    line(lastByte_m+20,height/2,lastByte_s-20,height/2);
  }
}

void serialEvent (Serial myPort) {
  String inString = myPort.readStringUntil('\n');
    if (inString != null) {
      inString = trim(inString);
      String splitString[] = inString.split(" ");
      // Leader position reported first
      inByte_m = float(splitString[0]);
      // Follower position reported second
      inByte_s = float(splitString[1]);
    }
    // Corrupted values just default to last viable.
    if (Float.isNaN(inByte_s)||Float.isNaN(inByte_m)){
       inByte_s=lastByte_s;
       inByte_m=lastByte_m;
    }
    lastByte_s  = map(inByte_s, -.1, .1, 0, 400);
    lastByte_m = map(inByte_m,-.1,.1,0,400);


}
