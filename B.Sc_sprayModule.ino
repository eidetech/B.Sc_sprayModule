/*
  ---------Sprøytemudul----------
  
  Using the BNO080 IMU

  Euler Angles
  Based on: Example1-RotationVector
  By: Nathan Seidle
  SparkFun Electronics
  Date: December 21st, 2017
  SparkFun code, firmware, and software is released under the MIT License.
  Please see LICENSE.md for further details.

  
  This code output Euler angles: roll, pitch and yaw.
  Then convert it into pwm to controll a motor
  The yaw (compass heading) is tilt-compensated, which is nice.
  https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  https://github.com/sparkfun/SparkFun_MPU-9250-DMP_Arduino_Library/issues/5#issuecomment-306509440

  It takes about 1ms at 400kHz I2C to read a record from the sensor, but we are polling the sensor continually
  between updates from the sensor. Use the interrupt pin on the BNO080 breakout to avoid polling.

  Time of flight sensor measure distance in mm.

*/

#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h" 
#include <VL53L0X.h>
#include <Servo.h>

BNO080 myIMU;
VL53L0X sensor;
Servo actuator;
Servo ESC;

#define RED_BUTTON 2



bool regulate = false;
int lastButtonState;
int currentButtonState;

long prevT = 0;
float eprev = 0;
float eintegral = 0;

bool spray = false;
int counter = 0;
float roll = 0;

void setup()
{
  Serial.begin(115200);

  pinMode(RED_BUTTON, INPUT_PULLUP);

  Wire.begin();
  myIMU.begin();

  
  if (myIMU.begin() == false)
  {
    Serial.println(F("BNO080 not detected at default I2C address"));
    while (1);
    }

  //Init I2C and IMU
  Wire.setClock(400000); //Increase I2C data rate to 400kHz
  myIMU.enableRotationVector(50); //Send data update every 50ms
  
  //Serial.println(F("Rotation vector enabled"));
  //Serial.println(F("Output in form roll, pitch, yaw"));


  //Time of Flight sensor
  sensor.init();
  sensor.setTimeout(500);



  actuator.attach(5);
  TCCR0B = (TCCR0B & 0b11111000) | 0x02;

  ESC.attach(11);
  ESC.write(90);
  delay(500);
}


void loop()
{
    lastButtonState    = currentButtonState;
    currentButtonState = digitalRead(RED_BUTTON);

  // If the button is pressed, toggle the on/off the control loop
  if(lastButtonState == HIGH && currentButtonState == LOW) {
    regulate = !regulate;
    }

  // If the regulate bool is true, then start PID
  if (regulate)
  {
    //Look for reports from the IMU
    if (myIMU.dataAvailable() == true)
    {   
    actuate();
    
    //Get data from IMU
    float roll = (myIMU.getRoll()) * 180.0 / PI; // Convert roll to degrees
    //float pitch = (myIMU.getPitch()) * 180.0 / PI; // Convert pitch to degrees
    //float yaw = (myIMU.getYaw()) * 180.0 / PI; // Convert yaw / heading to degrees
    
    
    
    //float roll = myIMU.getLinAccelY();
    //float y = myIMU.getLinAccelY();
    //float z = myIMU.getLinAccelZ();


    
    //Get data from TOF
    int distance = sensor.readRangeSingleMillimeters();
    



    if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

    //-----PID control-----

    // set target position
    float target = -2.03;
    //int target = 250*sin(prevT/1e6);

    //PID constants (commented out due to potentiometer implementation)
       float kp = 20;
       float kd = 0;
       float ki = 0;

    //distanc from wall
    float pos = -roll;
    
    // error
    float e = pos-target;

    // time difference
    long currT = micros();
    float deltaT = ((float) (currT - prevT))/( 1.0e6 );
    prevT = currT;
    
    //derivative
    float dedt = (e-eprev)/(deltaT);

    // integral
    eintegral = eintegral + e*deltaT;

    //control signal
    float u = kp*e + kd*dedt + ki*eintegral;

    int pwm = (int)fabs(-u+89);

    //if(pwm < 89 )
    //{
    //  pwr = pwr-2;
    //}

//    if(pwr > 91 )
//    {
//      pwr = 91;
//    }

//    if(pwr < 85 )
//    {
//      pwr = 85;
//    }

//    if(e<0.05 || e>-0.05){
//      pwr = 97;
//    }
    

    //SEND PWM
   
    
    ESC.write(pwm);

    // store previous error
    eprev = e;

    //Serial.print(kp,4);
    //Serial.print(", ");
    Serial.print("IMU deg: ");
    Serial.print(pos*10+50);
    Serial.print(", ");
    //Serial.print(millis());
    Serial.print("Error: ");
    Serial.print(e);
    Serial.print(", ");
    Serial.print("U: ");
    Serial.print(u);
    Serial.print(", ");
    Serial.print("PWM: ");
    Serial.println(pwm);
    
    }
    
    
  }
  else{
      disengage();
      //Look for reports from the IMU
    if (myIMU.dataAvailable() == true)
    {   
    
    //Get data from IMU
    float roll = (myIMU.getRoll()) * 180.0 / PI; // Convert roll to degrees
    //float pitch = (myIMU.getPitch()) * 180.0 / PI; // Convert pitch to degrees
    //float yaw = (myIMU.getYaw()) * 180.0 / PI; // Convert yaw / heading to degrees
    
    
    
    //float roll = myIMU.getLinAccelY();
    //float y = myIMU.getLinAccelY();
    //float z = myIMU.getLinAccelZ();


    
    //Get data from TOF
    int distance = sensor.readRangeSingleMillimeters();
    



    if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

    //-----PID control-----

    // set target position
    float target = -2.03;
    //int target = 250*sin(prevT/1e6);

    //PID constants (commented out due to potentiometer implementation)
       float kp = 10;
       float kd = 0;
       float ki = 0;

    //distanc from wall
    float pos = -roll;
    
    // error
    float e = pos-target;

    // time difference
    long currT = micros();
    float deltaT = ((float) (currT - prevT))/( 1.0e6 );
    prevT = currT;
    
    //derivative
    float dedt = (e-eprev)/(deltaT);

    // integral
    eintegral = eintegral + e*deltaT;

    //control signal
    float u = kp*e + kd*dedt + ki*eintegral;

    int pwm = (int)fabs(-u+90);

    //if(pwm < 93 )
    //{
    //  pwr = pwr-4;
    //}

//    if(pwr > 91 )
//    {
//      pwr = 91;
//    }

//    if(pwr < 85 )
//    {
//      pwr = 85;
//    }

//    if(e<0.05 || e>-0.05){
//      pwr = 97;
//    }
    

    //SEND PWM
   
    
    ESC.write(pwm);

    // store previous error
    eprev = e;

    //Serial.print(kp,4);
    //Serial.print(", ");
    Serial.print("IMU deg: ");
    Serial.print(pos*10+50);
    Serial.print(", ");
    //Serial.print(millis());
//    Serial.print("Error: ");
//    Serial.print(e);
//    Serial.print(", ");
//    Serial.print("U: ");
//    Serial.print(u);
//    Serial.print(", ");
    Serial.print("PWM: ");
    Serial.println(pwm);
    
    }
  }
}

void actuate()
{
  actuator.write(50);
   
    
}

void disengage()
{
  actuator.write(90);
  ESC.write(90);
  
  
}
