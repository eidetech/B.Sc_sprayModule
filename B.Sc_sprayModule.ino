/*
  --------- Spray module ----------
  
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
// ######################################### Libraries ########################################
#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h" 
//#include <VL53L0X.h>
#include <Servo.h>

// CAN libraries
#include <SPI.h>
#include <mcp2515.h>

// ############################### Sensor and actutator objects ###############################
BNO080 myIMU;
//VL53L0X sensor;
Servo actuator;
Servo ESC;

// CAN message and object
struct can_frame canMsg;
MCP2515 mcp2515(10);


// ###################################### Global variables ###################################

//---PID---
float kp = 5;         // 5
float ki = 0;         // 0
float kd = 0;         // 0

//---RPM---
int idleRPM = 60;     // 60
int sprayRPM = 90;    // 120
float targetPos = -2;

bool regulate = false;
bool printNumber = true;

int regulationMode = 1;
int currentState = 2;

int lastButtonState;
int currentButtonState;

float e;

long prevT = 0;
float eprev = 0;
float eintegral = 0;


bool spray = false;
int counter = 0;
float pos;
float roll;

float u = 0;

int pwm = 0;


const long sprayIntervalOn = 40000;
const long sprayIntervalOff = 30000;
unsigned long previousMillisStart = 0;
unsigned long previousMillisStop =0;
struct can_frame IMU;
uint16_t pitchCAN;

// ####################################### Setup ##############################################
void setup()
{
    // Serial setup
  delay(1000);
  Serial.begin(115200);

    // Define pin modes of pins
   
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(4, OUTPUT);

  // CAN bus setup
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS);
  mcp2515.setNormalMode();
  
  // IMU setup
  Wire.begin();
  
    if (myIMU.begin() == false)
    {
      Serial.println(F("BNO080 not detected at default I2C address"));
      while (1);
    }

   //Init I2C and IMU
  Wire.setClock(400000); //Increase I2C data rate to 400kHz
  myIMU.enableRotationVector(10); //Send data update every 50ms
  
  
  // //Time of Flight sensor
  // sensor.init();
  // sensor.setTimeout(500);

    // Servo actuator setup
    actuator.attach(5);
    TCCR0B = (TCCR0B & 0b11111000) | 0x02;

    // Drone motor ESC setup
    // ### Write 0 to ESC for it to initialize, and then write ~80 to spin the motor slowly. Remember to use 1000, 2000 work area when attaching the servo object!
    ESC.attach(3,1000,2000);
    ESC.write(0);
    delay(5000); // delay to allow the ESC to recognize the stopped signal.
}

// ####################################### Loop ##############################################
void loop()
{
  long startTid = millis();
  
// ################################### CAN Receive ###########################################
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    if(canMsg.can_id == 0x40)
    {
      currentState = canMsg.data[0];
    }
  }
 
  //Look for reports from the IMU
  if (myIMU.dataAvailable() == true)
  {   
    float pitch = (myIMU.getPitch()) * 180.0 / PI; // Convert pitch to degrees
    float pitchConvert = (myIMU.getPitch()) * 10000.0;
    pitchCAN = (int)pitchConvert+10000;
    IMU.can_id  = 0x41;
    IMU.can_dlc = 2;
    IMU.data[0] = pitchCAN;
    IMU.data[1] = pitchCAN >> 8;
    mcp2515.sendMessage(&IMU);

    
    //Get data from TOF
    //int distance = sensor.readRangeSingleMillimeters();

    //if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

    // ################################### PID Regulator #####################################

   
    // Position variable
    pos = pitch;
    
   
    // Error
    e = pos-targetPos;
    

    // Time difference
    long currT = micros();
    float dt = ((float) (currT - prevT))/( 1.0e6 );
    prevT = currT;

    // Derivative
    float dedt = (e-eprev)/(dt);
    
    
    // Integral
    eintegral = eintegral + e*dt;
    
    
    // Control signal
    u = kp*e + kd*dedt + ki*eintegral;
    
    pwm = (int)-u;
      
 
    
    // store previous error
    eprev = e;
     
    }


// ################################### Pulse regulator #####################################
//    if (startTid - previousMillisStart >= sprayIntervalOff) 
//    {
//      stopSignal = 0;
//    }
//    if (startTid - previousMillisStart >= sprayIntervalOn) 
//    {
//      previousMillisStart = startTid;
//      stopSignal = 1;
//    }
        
        

// If the regulate switch is pressed, write PWM to ESC 
        if(currentState == 1)
        {
            ESC.write(sprayRPM+pwm);
            digitalWrite(4, HIGH);
            actuate();
            //Serial.print("ACTIVE");
            if(u < 0)
            {
               ESC.write(sprayRPM);
            }
            
        }
        else if(currentState == 0)
        {
            ESC.write(idleRPM+pwm);
            disengage();
            digitalWrite(4, LOW);
            //Serial.print("IDLE");
        }
        else
        {
          ESC.write(0);
          disengage();
        }


    
    // ############################### Serial input for setting PID gain values live ###############################
    int input = Serial.read();

    if(input== 112)  //p = 112 ASCII
    {
        if (Serial.available()>0)
        {
          kp=Serial.parseFloat();        
        }
    }

    if(input== 105)  //i = 105 ASCII
    {
        if (Serial.available()>0)
        {
          ki=Serial.parseFloat();
        }
    }

    if(input== 100)  //d = 100 ASCII
    {
      if (Serial.available()>0)
      {
        kd=Serial.parseFloat();
      }
    }

// ############################################### Serial logging ##############################################

   //Serial.print(pos);
   //Serial.print(", ");
   //Serial.println(pitchCAN);
   //   Serial.print(", ");
   //   Serial.println(uprev*10,4);
  
  long stoppTid = millis();
  long tid = stoppTid-startTid;
   //Serial.println(tid);
}


void actuate()
{
  actuator.write(85);   //85
}

void disengage()
{
  actuator.write(50);   //50
}
