import processing.serial.*;
import java.awt.event.KeyEvent;
import java.io.IOException;
Serial myPort;
String data="";
float roll,ultrasonic;
float current_height,top_height,bottom_height,delta_height;
float last_ultrasonic;
float preset_height = 400.00;
void setup() {
  size (1920, 1080);
  smooth(8);
  frameRate(120);
  myPort = new Serial(this, "COM3", 9600); // starts the serial communication
  myPort.bufferUntil('\n');
  top_height = preset_height;
  bottom_height = preset_height;
}

void draw(){
  delta_height = round(calculateDeltaHeight(roll,100)*100)/100.0;
  current_height = preset_height+delta_height;
  if(ultrasonic != 0.0){
    last_ultrasonic = ultrasonic;
  }
  if(current_height > top_height){
    top_height = current_height;
  } else if (current_height < bottom_height) {
    bottom_height = current_height;
  }
  translate(width/2.0,height/4.0);
  background(233);
  textSize(22);
  fill(0,0,0);
  text("Hoek: " + int(roll) +"°", -width*0.4, 0);
  text("Basishoogte: " + preset_height + "cm", -width*0.4, 50);
  text("Hoogteverschil: " + delta_height + "cm", -width*0.4, 100);
  text("Waterhoogte: " + current_height + "cm", -width*0.4, 150);
  text("Laagste punt: " + bottom_height + "cm", -width*0.4, 200);
  text("Hoogste punt: " + top_height + "cm", -width*0.4, 250);
  
  text("Sonar: " + last_ultrasonic + "cm", -width*0.4, 350);
  fill(91);
  rect(-20,20,40,420);
  translate(0,20);
  rotate(radians(-roll));
  translate(0,-20);
  fill(91);
  rect(0,0,400,40);
  circle(0,20,60);
}

void serialEvent (Serial myPort) { 
  // reads the data from the Serial Port up to the character '.' and puts it into the String variable "data".
  data = myPort.readStringUntil('\n');
  // if you got any bytes other than the linefeed:
  if (data != null) {
    data = trim(data);
    // split the string at "/"
    //roll = float(data);
    String items[] = split(data, '|');
    if (items.length > 1) {
      //--- Roll,Pitch in degrees
      roll = float(items[0]);
      ultrasonic = float(items[1]);
      //pitch = float(items[1]);
      //yaw = float(items[2]);
    }
  }
}

float calculateDeltaHeight(float angle, int armLength){
  return armLength*sin(degToRad(angle));
}

float radToDeg(float input){
  return input * (180/PI);
}

float degToRad(float input){
  return input * (PI/180);
}
