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
#include <VL53L0X.h>
#include <Servo.h>

// CAN libraries
#include <SPI.h>
#include <mcp2515.h>

// ############################### Sensor and actutator objects ###############################
BNO080 myIMU;
VL53L0X sensor;
Servo actuator;
Servo ESC;
Servo BLDC;

// CAN message and object
struct can_frame canMsg;
MCP2515 mcp2515(10);

// ########################################## Defines #########################################
#define RED_BUTTON 2
// #################3##################### Global variables ###################################
float kp = 5; // 30
float ki = 0;
float kd = 0;  //1
int idle = 60;
int sprayRPM = 130;
float targetPos = 1;

float kpVel = 0; // 
float kiVel = 0;
float kdVel = 0;  //

bool regulate = false;
bool printNumber = true;

int regulationMode = 1;
int currentState = 0;

int lastButtonState;
int currentButtonState;

float stopSignal = 0;

float targetVel = 0;
float ePos;
float eVel;
long prevT = 0;
float eprevPos = 0;
float eprevVel = 0;
float eintegralPos = 0;
float eintegralVel = 0;


bool spray = false;
int counter = 0;
float pos;
float vel;
float roll;
float uPos = 0;
float u = 0;
float uprev = 0;

int pwm = 0;


const long sprayIntervalOn = 40000;
const long sprayIntervalOff = 30000;
unsigned long previousMillisStart = 0;
unsigned long previousMillisStop =0;

// ####################################### Setup ##############################################
void setup()
{
    // Serial setup
  delay(1000);
  Serial.begin(115200);

    // Define pin modes of pins
    pinMode(RED_BUTTON, INPUT_PULLUP);
    pinMode(LED_BUILTIN, OUTPUT);

  // CAN bus setup
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
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
  
  //myIMU.enableGyro(10); //Send data update every 50ms
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

    // BLDC motor setup (with VESC) for weight compensation
    BLDC.attach(6);
    BLDC.write(0);
}

// ####################################### Loop ##############################################
void loop()
{
  long startTid = millis();
  
// ################################### CAN Receive ###########################################
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    if(canMsg.can_id == 0x1E)
    {
      currentState = canMsg.data[0];
      if(currentState == 1)
      {
                digitalWrite(LED_BUILTIN, HIGH);
        actuate();
                //Serial.println("Actuating...");
      }
      else if(currentState == 0)
      {
                digitalWrite(LED_BUILTIN, LOW);
        disengage();
                //Serial.println("Not actuating...");
      }
      else
      {
        disengage();
      }
    }
  }
  // ############## Button input for activating/deactivating PID regulator #################
  lastButtonState    = currentButtonState;
  currentButtonState = digitalRead(RED_BUTTON);

  // If the button is pressed, toggle the on/off the control loop
  if(lastButtonState == HIGH && currentButtonState == LOW) {
    regulate = !regulate;
  }

    
  //Look for reports from the IMU
  if (myIMU.dataAvailable() == true)
  {   
//    float ax, ay, az, gx, gy, gz, qx, qy, qz, qw; //  mx, my, mz, (qx, qy, qz, qw = i,j,k, real)
//    byte linAccuracy = 0;
//    byte gyroAccuracy = 0;
//    byte magAccuracy = 0;
//    float quatRadianAccuracy = 0;
//    byte quatAccuracy = 0;

    // get IMU data in one go for each sensor type
    //myIMU.getLinAccel(ax, ay, az, linAccuracy);
    //myIMU.getGyro(gx, gy, gz, gyroAccuracy);
    //myIMU.getQuat(qx, qy, qz, qw, quatRadianAccuracy, quatAccuracy);
    float roll = (myIMU.getRoll()) * 180.0 / PI; // Convert roll to degrees
    //myIMU.getMag(mx, my, mz, magAccuracy);
    //Get data from TOF
    //int distance = sensor.readRangeSingleMillimeters();

    //if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

    // ################################### PID Regulator #####################################

        // Set target for PID
    //int target = 250*sin(prevT/1e6);
    
        targetVel = 0;

    // Position variable
    pos = roll;
    //pos = qy*180/PI;
   
    // Error
    ePos = pos-targetPos;
    

    // Time difference
    long currT = micros();
    float dt = ((float) (currT - prevT))/( 1.0e6 );
    prevT = currT;

    // Derivative
    float dedt = (ePos-eprevPos)/(dt);
    
    
    // Integral
    eintegralPos = eintegralPos + ePos*dt;
    
    
    // Control signal
    u = kp*ePos + kd*dedt + ki*eintegralPos;
    
    
    //Velosity sensor
    //vel = gx*180/PI;
    
    //Error inner loop
    //eVel = vel-uPos;
    
    //KI vel
    //eintegralVel = eintegralVel + eVel*dt;

    //KD vel
    float dedtVel = (eVel-eprevVel)/(dt);

    //u = kpVel*eVel + kdVel*dedtVel + kiVel*eintegralVel;
    
    pwm = (int)u;

        // Constrain PWM signal to 0-180
//        if(pwm > 90)
//        {
//            pwm = 90;
//        }
//        if(pwm < 70)
//        {
//            pwm = 70;
//        }
       
        
//        // If the regulate switch is pressed, write PWM to ESC 
//        if(currentState == 1 || regulate == true)
//        {
//            BLDC.write(pwm);
//            //Serial.print("ACTIVE");
//        }else
//        {
//            BLDC.write(90);
//            //Serial.print("IDLE");
//        }
      
 
    
    // store previous error
    eprevPos = ePos;
    eprevVel = eVel;  
     
    }
    
    if (startTid - previousMillisStart >= sprayIntervalOff && stopSignal != 2) 
    {
      stopSignal = 0;
    }
    if (startTid - previousMillisStart >= sprayIntervalOn && stopSignal != 2) 
    {
      previousMillisStart = startTid;
      stopSignal = 1;
    }
        
        // If the regulate switch is pressed, write PWM to ESC 
        if(currentState == 1 || regulate == true || stopSignal == 1)
    {
            ESC.write(sprayRPM+pwm);
            //BLDC.write(pwm);
            actuate();
            //Serial.print("ACTIVE");
            if(u < 0)
            {
               ESC.write(sprayRPM);
            }
            
        }
        else
        {
            ESC.write(idle+pwm);
            //BLDC.write(pwm);
            disengage();
            //Serial.print("IDLE");
        }

         if(stopSignal == 2)
          {
            ESC.write(0);
            BLDC.write(78);
          }

    
    // ############################### Serial input for setting PID gain values live ###############################
    int input = Serial.read();

    if(input== 112)  //p = 112 ASCII
        {if (Serial.available()>0)
        {kp=Serial.parseFloat();
        
        }
    }

    if(input== 105)  //i = 105 ASCII
        {if (Serial.available()>0)
        {ki=Serial.parseFloat();
        }
    }

    if(input== 100)  //d = 100 ASCII
    {
      if (Serial.available()>0)
      {kd=Serial.parseFloat();
      }
    }


    if(input== 115)  //s = 115 ASCII
    {
      if (Serial.available()>0)
      {stopSignal=Serial.parseFloat();
      }
    }
    
    if(input== 118)  //v = 118 ASCII
    {
      if (Serial.available()>0)
      {kpVel=Serial.parseFloat();
      }
    }

    if(input== 119)  //u = 119 ASCII
    {
      if (Serial.available()>0)
      {kdVel=Serial.parseFloat();
      }
    }
    // ############################################### Serial printing ##############################################
//      Serial.print("Kp: ");
//      Serial.print(kp,4);
//      Serial.print(", ");
//      Serial.print("Ki: ");
//      Serial.print(ki,4);
//      Serial.print(", ");
//      Serial.print("Kd: ");
//      Serial.print(kd,4);
//      Serial.print(", ");
//      Serial.print("KpVel: ");
//      Serial.print(kpVel,4);
//      Serial.print(", ");
//      Serial.print("KdVel: ");
//      Serial.print(kdVel,4);
//      Serial.print(", ");
//
//      Serial.print("IMU pos: ");
//      Serial.print(pos);
//      Serial.print("IMU vel: ");
//      Serial.print(vel);
//      Serial.print(", ");
//      Serial.print("U: ");
//      Serial.print(u,6);
//      Serial.print(", ");
//      //Serial.print(millis());
//      Serial.print("PWM: ");
//      Serial.println(pwm);
      //Serial.print(", ");
// ############################################### Serial logging ##############################################

   //Serial.print(millis());
   //Serial.print(", ");
   Serial.println(pos,4);
   //   Serial.print(", ");
   //   Serial.println(uprev*10,4);
  
  long stoppTid = millis();
  long tid = stoppTid-startTid;
   //Serial.println(tid);
}


void actuate()
{
  actuator.write(50);
}

void disengage()
{
  actuator.write(90);
}
