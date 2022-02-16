/*
  Sprøyterobot IMU
  
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

*/

#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h" 
#include <VL53L0X.h>
#include <Servo.h>

BNO080 myIMU;
VL53L0X sensor;
Servo actuator;

int pos = 0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;

bool spray = false;
int counter = 0;


void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println("Spray Module position");

  Wire.begin();

  if (myIMU.begin() == false)
  {
    Serial.println(F("BNO080 not detected at default I2C address"));
    while (1)
      ;
  }

  Wire.setClock(400000); //Increase I2C data rate to 400kHz

  myIMU.enableRotationVector(1); //Send data update every 50ms

  Serial.println(F("Rotation vector enabled"));
  Serial.println(F("Output in form roll, pitch, yaw"));


  //Time of Flight sensor
  sensor.init();
  sensor.setTimeout(500);

  analogWrite(11, 0);
  delay(1000);

  actuator.attach(5);
  TCCR0B = (TCCR0B & 0b11111000) | 0x02;
}

void loop()
{
  //Look for reports from the IMU
  if (myIMU.dataAvailable() == true)
  {
    counter++;
    Serial.print("Counter: ");
    Serial.println(counter);
    if(spray && counter <= 50)
    {
      actuate();
    }else
    {
      disengage();
      if(counter >= 100)
      {
        counter = 0;
      }
    }
    float roll = (myIMU.getRoll()) * 180.0 / PI; // Convert roll to degrees
    float pitch = (myIMU.getPitch()) * 180.0 / PI; // Convert pitch to degrees
    float yaw = (myIMU.getYaw()) * 180.0 / PI; // Convert yaw / heading to degrees

    int distance = sensor.readRangeSingleMillimeters();

//    Serial.print("Roll: ");
//    Serial.print(roll, 1);
//    Serial.print(F(","));
//    Serial.print("Pich: ");
//    Serial.print(pitch, 1);
//    Serial.print(F(","));
//    Serial.print("Yaw: ");
//    Serial.print(yaw, 1);

    //Serial.print("   Distanse in mm: ");
    //Serial.print(distance);
     if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

    //Serial.println();
    
  
  
  // set target position
  int target = 300;
  //int target = 250*sin(prevT/1e6);

  //PID constants
  float kp = 1;
  float kd = 0;
  float ki = 0;

  //distanc from wall
  pos = distance;
  
  // error
  int e = pos-target;

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
  
  // motor power
  float pwr = fabs(u);
  if( pwr > 255 ){
    pwr = 255;
  }
  
  //SEND PWM
  analogWrite(11, pwr);

  // store previous error
  eprev = e;

  //Serial.print(target);
  //Serial.print(" ");
  //Serial.print(pos);
  //Serial.println();
  Serial.print(pwr);
  Serial.println();
  }
  
}

void setProp(int pwmval, int escPin)
{
  analogWrite(escPin, pwmval);
}

void actuate()
{
  actuator.write(50);
}

void disengage()
{
  actuator.write(90);
}
