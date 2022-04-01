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
float kp = 6;
float ki = 0;
float kd = 1;

bool regulate = false;
bool printNumber = true;

int lastButtonState;
int currentButtonState;

float target = 0;
float e;
long prevT = 0;
float eprev = 0;
float eintegral = 0;

bool spray = false;
int counter = 0;
float roll = 0;
// ####################################### Setup ##############################################
void setup()
{
    // Serial setup
	Serial.begin(115200);

    // Define pin modes of pins
	pinMode(RED_BUTTON, INPUT_PULLUP);

	// CAN bus setup
	mcp2515.reset();
	mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
	mcp2515.setNormalMode();
  
    // IMU setup
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

	//Time of Flight sensor
	sensor.init();
	sensor.setTimeout(500);

    // Servo actuator setup
	actuator.attach(5);
	TCCR0B = (TCCR0B & 0b11111000) | 0x02;

    // Drone motor ESC setup
    // ### Write 0 to ESC for it to initialize, and then write ~80 to spin the motor slowly. Remember to use 1000, 2000 work area when attaching the servo object!
    ESC.attach(6,1000,2000);
	ESC.write(0);
	delay(5000); // delay to allow the ESC to recognize the stopped signal.

    // BLDC motor setup (with VESC) for weight compensation
    BLCD.attach(3);
    BLDC.write(0);
}

// ####################################### Loop ##############################################
void loop()
{
// ################################### CAN Receive ###########################################
	if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
		if(canMsg.can_id == 0x1E)
		{
			int currentState = canMsg.data[0];
			if(currentState == 1)
			{
				actuate();
                Serial.println("Actuating...");
			}
			else if(currentState == 0)
			{
				disengage();
                Serial.println("Not actuating...");
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
		//Get data from IMU
		float roll = (myIMU.getRoll()) * 180.0 / PI; // Convert roll to degrees

        // Other angles:
		//float pitch = (myIMU.getPitch()) * 180.0 / PI; // Convert pitch to degrees
		//float yaw = (myIMU.getYaw()) * 180.0 / PI; // Convert yaw / heading to degrees

		//float roll = myIMU.getLinAccelY();
		//float y = myIMU.getLinAccelY();
		//float z = myIMU.getLinAccelZ();

		//Get data from TOF
		int distance = sensor.readRangeSingleMillimeters();

		if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

		// ################################### PID Regulator #####################################

        // Set target for PID
		//int target = 250*sin(prevT/1e6);
        target = -3.6;

		// Position variable
		float pos = -roll;

		// Error
		e = pos-target;

		// Time difference
		long currT = micros();
		float dt = ((float) (currT - prevT))/( 1.0e6 );
		prevT = currT;

		// Derivative
		float dedt = (e-eprev)/(dt);

		// Integral
		eintegral = eintegral + e*dt;

		// Control signal
		float u = kp*e + kd*dedt + ki*eintegral;

		int pwm = (int)-u+89;

        // Constrain PWM signal to 0-180
        if(pwm > 180)
        {
            pwm = 180;
        }else if(pwm < 0)
        {
            pwm = 0;
        }

        // If the regulate switch is pressed, write PWM to ESC 
        if(regulate)
        {
            ESC.write(pwm);
            Serial.print("ACTIVE");
        }else
        {
            ESC.write(90);
            Serial.print("IDLE");
        }
			
 
		
		// store previous error
		eprev = e;    

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

		// ############################################### Serial printing ##############################################
			Serial.print("Kp: ");
			Serial.print(kp,4);
			Serial.print(", ");
			Serial.print("Ki: ");
			Serial.print(ki,4);
			Serial.print(", ");
			Serial.print("Kd: ");
			Serial.print(kd,4);
			Serial.print(", ");

			Serial.print("IMU deg: ");
			Serial.print(pos);
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


void actuate()
{
  actuator.write(50);
}

void disengage()
{
  actuator.write(90);
}
