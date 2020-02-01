/* GYRO PINOUT
   VCC    |   5v
   GND    |   GND
   SDA    |   A4
   SCL    |   A5
*/

/* ULTRASONIC PINOUT
   VCC    |   5v
   GND    |   GND
   TRIG   |   D11
   ECHO   |   D12
*/

#include <NewPing.h>
#include <Wire.h>

#define trig_pin 9
#define echo_pin 10
#define max_distance 500

int calibration_button_pin =12;
bool calibration_button_pressed =false;
int calibration_button_state = 0;
bool startup_completed = false;

int measure120_button_pin=11;
bool measure120_button_pressed =false;
int measure120_button_state=0;


//Gyro Variables
float elapsedTime, time, timePrev, elapsedTime250ms;       //Variables for time control
float elapsedTime500ms;
float Gyr_rawX, Gyr_rawY, Gyr_rawZ;     //Here we store the raw data read
float Gyro_angle_x, Gyro_angle_y;         //Here we store the angle value obtained with Gyro data
float Gyro_raw_error_x, Gyro_raw_error_y;

float timePrev250ms = 0;
float timePrev500ms = 0;
float Total_height_to_average_250ms;
int Number_of_readings_250ms;
float Average_height;



//Acc Variables
float Acc_rawX, Acc_rawY, Acc_rawZ;    //Here we store the raw data read
float Acc_angle_x, Acc_angle_y;          //Here we store the angle value obtained with Acc data
float Acc_angle_error_x, Acc_angle_error_y;

int acc_error = 0;
int gyro_error = 0;

float Total_angle_x, Total_angle_y;

NewPing sonar(trig_pin,echo_pin,max_distance);

void setup() {
  Wire.begin();                           //begin the wire comunication

  Wire.beginTransmission(0x68);           //begin, Send the slave adress (in this case 68)
  Wire.write(0x6B);                       //make the reset (place a 0 into the 6B register)
  Wire.write(0x00);
  Wire.endTransmission(true);             //end the transmission
  //Gyro config
  Wire.beginTransmission(0x68);           //begin, Send the slave adress (in this case 68)
  Wire.write(0x1B);                       //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                       //Set the register bits as 00010000 (1000dps full scale)
  Wire.endTransmission(true);             //End the transmission with the gyro
  //Acc config
  Wire.beginTransmission(0x68);           //Start communication with the address found during search.
  Wire.write(0x1C);                       //We want to write to the ACCEL_CONFIG register
  Wire.write(0x10);                       //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);

  Serial.begin(9600);                     //Remember to set this same baud rate to the serial monitor
  time = millis();                        //Start counting time in milliseconds

  pinMode(calibration_button_pin,INPUT_PULLUP);
  pinMode(measure120_button_pin,INPUT_PULLUP);

  while (!startup_completed) {
    testForCalibrationButtonPress();
  }
}



void loop() {
  timePrev = time;
  time = millis();
  elapsedTime = (time - timePrev) / 1000;       // Elapsed time in ms devided by 1000 in order to obtain seconds
  elapsedTime250ms = (time - timePrev250ms);
  elapsedTime500ms = (time - timePrev500ms);
  

  testForCalibrationButtonPress();
  testForMeasure120ButtonPress();
  getAngle();
  sendDataToSerial();

}

float calculateDeltaHeight(float angle, int armLength) {
  return armLength * sin(degToRad(angle));
}

float radToDeg(float input) {
  return input * (180 / PI);
}

float degToRad(float input) {
  return input * (PI / 180);
}

void testForCalibrationButtonPress() {
  calibration_button_state = !digitalRead(calibration_button_pin);
  //Serial.println(calibration_button_state);
  if (calibration_button_state == HIGH) {
    doCalibrationSequence();
    //Serial.println("calibration button pressed");
  }
}

void testForMeasure120ButtonPress() {
  measure120_button_state = !digitalRead(measure120_button_pin);
  Serial.println(measure120_button_state);
  if (measure120_button_pin == HIGH) {
    measure120();
    //Serial.println("measure120 button pressed");
  }
}

float measureUltrasonicDistance(){
  return sonar.ping_cm();
}

void measure120(){
  int numberOfReadings = 0;
  int armLength = 100;
  float delta_height = calculateDeltaHeight(Total_angle_y, armLength);
  float obtained_data[120];
  if (elapsedTime500ms > 500){
    if (numberOfReadings < 120){
      obtained_data[numberOfReadings] = delta_height;
      numberOfReadings++;
      timePrev500ms = time;
    }else{
      Serial.println("Verzamelde data:\n");
      for(int i=0; i<120;i++){
        Serial.println(obtained_data[i]);
      }
    }
  }
}

void sendDataToSerial(){
  Serial.print(Total_angle_y);
  Serial.print("|");
  Serial.println(measureUltrasonicDistance());
}

void doCalibrationSequence() {
  for (int a = 0; a < 200; a++) {
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);                       //Ask for the 0x3B register- correspond to AcX
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 6, true);

    Acc_rawX = (Wire.read() << 8 | Wire.read()) / 4096.0 ; //each value needs two registres
    Acc_rawY = (Wire.read() << 8 | Wire.read()) / 4096.0 ;
    Acc_rawZ = (Wire.read() << 8 | Wire.read()) / 4096.0 ;

    /*---X---*/
    Acc_angle_error_x = Acc_angle_error_x + (radToDeg(atan((Acc_rawY) / sqrt(pow((Acc_rawX), 2) + pow((Acc_rawZ), 2)))));
    /*---Y---*/
    Acc_angle_error_y = Acc_angle_error_y + (radToDeg(atan(-1 * (Acc_rawX) / sqrt(pow((Acc_rawY), 2) + pow((Acc_rawZ), 2)))));

    if (a == 199)
    {
      Acc_angle_error_x = Acc_angle_error_x / 200;
      Acc_angle_error_y = Acc_angle_error_y / 200;
      acc_error = 1;
    }
  }
  for (int i = 0; i < 200; i++) {
    Wire.beginTransmission(0x68);            //begin, Send the slave adress (in this case 68)
    Wire.write(0x43);                        //First adress of the Gyro data
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 4, true);         //We ask for just 4 registers

    Gyr_rawX = Wire.read() << 8 | Wire.read(); //Once again we shif and sum
    Gyr_rawY = Wire.read() << 8 | Wire.read();

    /*---X---*/
    Gyro_raw_error_x = Gyro_raw_error_x + (Gyr_rawX / 32.8);
    /*---Y---*/
    Gyro_raw_error_y = Gyro_raw_error_y + (Gyr_rawY / 32.8);
    if (i == 199)
    {
      Gyro_raw_error_x = Gyro_raw_error_x / 200;
      Gyro_raw_error_y = Gyro_raw_error_y / 200;
      gyro_error = 1;
    }
  }
  if(!startup_completed){
    startup_completed = true;
  }
}

void getAngle(){
  //////////////////////////////////////Gyro read/////////////////////////////////////

  Wire.beginTransmission(0x68);            //begin, Send the slave adress (in this case 68)
  Wire.write(0x43);                        //First adress of the Gyro data
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 4, true);         //We ask for just 4 registers

  Gyr_rawX = Wire.read() << 8 | Wire.read(); //Once again we shif and sum
  Gyr_rawY = Wire.read() << 8 | Wire.read();
  /*To get the gyroscope data in degrees/second the raw value by 32.8 this value is specified by
    the datasheet of the accelerometer for a range of 1000degrees/second. Subtract the error
    values set during calibration to get the correct angular momentum*/
  /*---X---*/
  Gyr_rawX = (Gyr_rawX / 32.8) - Gyro_raw_error_x;
  /*---Y---*/
  Gyr_rawY = (Gyr_rawY / 32.8) - Gyro_raw_error_y;

  /*Multiply the elapsed time with the degrees/second that was just retrieved to get the angle*/
  /*---X---*/
  Gyro_angle_x = Gyr_rawX * elapsedTime;
  /*---Y---*/
  Gyro_angle_y = Gyr_rawY * elapsedTime;

  //////////////////////////////////////Acc read/////////////////////////////////////

  Wire.beginTransmission(0x68);     //begin, Send the slave adress (in this case 68)
  Wire.write(0x3B);                 //Ask for the 0x3B register- correspond to AcX
  Wire.endTransmission(false);      //keep the transmission and next
  Wire.requestFrom(0x68, 6, true);  //We ask for next 6 registers starting withj the 3B
  /*We have asked for the 0x3B register. The IMU will send a brust of register.
    The amount of register to read is specify in the requestFrom function.
    In this case we request 6 registers. Each value of acceleration is made out of
    two 8bits registers, low values and high values. For that we request the 6 of them
    and just make then sum of each pair. For that we shift to the left the high values
    register (<<) and make an or (|) operation to add the low values.
    If we read the datasheet, for a range of+-8g, we have to divide the raw values by 4096*/
  Acc_rawX = (Wire.read() << 8 | Wire.read()) / 4096.0 ; //each value needs two registres
  Acc_rawY = (Wire.read() << 8 | Wire.read()) / 4096.0 ;
  Acc_rawZ = (Wire.read() << 8 | Wire.read()) / 4096.0 ;

  /*To get the accelerometer angles Euler's formula is used with the acceleration values. After
    that the error values set during the calibration*/
  /*---X---*/
  Acc_angle_x = radToDeg(atan((Acc_rawY) / sqrt(pow((Acc_rawX), 2) + pow((Acc_rawZ), 2)))) - Acc_angle_error_x;
  /*---Y---*/
  Acc_angle_y = radToDeg(atan(-1 * (Acc_rawX) / sqrt(pow((Acc_rawY), 2) + pow((Acc_rawZ), 2)))) - Acc_angle_error_y;


  //////////////////////////////////////Total angle and filter/////////////////////////////////////

  /*To get an accurate reading of the angles, both the accelerometer and gyroscope values are used.
    This is done because the gyroscope will drift over time, and the accelerometer alone doesn't
    provide very accurate readings*/
  /*---X axis angle---*/
  Total_angle_x = 0.98 * (Total_angle_x + Gyro_angle_x) + 0.02 * Acc_angle_x;
  /*---Y axis angle---*/
  Total_angle_y = 0.98 * (Total_angle_y + Gyro_angle_y) + 0.02 * Acc_angle_y;
}