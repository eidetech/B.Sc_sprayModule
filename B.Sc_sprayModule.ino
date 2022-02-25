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

#define KP_POT A0
#define KI_POT A1
#define KD_POT A2
#define RED_BUTTON 2

int kp_analog = 0;
int ki_analog = 0;
int kd_analog = 0;

bool regulate = false;
int lastButtonState;
int currentButtonState;

float pos = 0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;

bool spray = false;
int counter = 0;


void setup()
{
	Serial.begin(115200);

      //Serial.print("Angle");
      //Serial.print(",");
      //Serial.print("Distance");
      //Serial.print(",");
      //Serial.print("Error");
      //Serial.print(",");
      //Serial.print("U");
      //Serial.print(",");
      //Serial.print("P");
      //Serial.print(",");
      //Serial.print("I");
      //Serial.print(",");
      //Serial.print("D");
      //Serial.print(",");
      //Serial.println("Time");
      

	pinMode(RED_BUTTON, INPUT_PULLUP);

	Wire.begin();

	if (myIMU.begin() == false)
	{
		Serial.println(F("BNO080 not detected at default I2C address"));
		while (1);
  	}

	//Init I2C and IMU
	Wire.setClock(400000); //Increase I2C data rate to 400kHz
	myIMU.enableRotationVector(1); //Send data update every 50ms

	//Serial.println(F("Rotation vector enabled"));
	//Serial.println(F("Output in form roll, pitch, yaw"));


	//Time of Flight sensor
	sensor.init();
	sensor.setTimeout(500);



	actuator.attach(5);
	TCCR0B = (TCCR0B & 0b11111000) | 0x02;

	ESC.attach(11,1000,2000);
	ESC.write(0);
	delay(5000); // delay to allow the ESC to recognize the stopped signal.
}


void loop()
{
	// Read analog potentiometer values
    kp_analog = analogRead(KP_POT);
    ki_analog = analogRead(KI_POT);
    kd_analog = analogRead(KD_POT);

	// Map the analog potentiometer values to matching PID values
    float kp = map(kp_analog, 0, 1023, 0, 1000);
    float ki = map(ki_analog, 0, 1023, 0, 1000);
    float kd = map(kd_analog, 0, 1023, 0, 1000);

	kp = kp/100;
	ki = ki/100000;
	kd = kd/10000;
float roll = 0;
      //Look for reports from the IMU
    if (myIMU.dataAvailable() == true)
    {   
    //Get data from IMU
     roll = (myIMU.getRoll()) * 180.0 / PI; // Convert roll to degrees
    float pitch = (myIMU.getPitch()) * 180.0 / PI; // Convert pitch to degrees
    float yaw = (myIMU.getYaw()) * 180.0 / PI; // Convert yaw / heading to degrees
    }

    //Get data from TOF
    int distance = sensor.readRangeSingleMillimeters();
    if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
    
    // set target position
    float target = -4;
      Serial.print(distance);
      Serial.print(",");
      Serial.print(-roll);
      Serial.print(",");
      //Serial.println(target);
      Serial.println(millis());

//	Serial.print("Kp: ");
//	Serial.print(kp, 4);
//	Serial.print(", ");
//	Serial.print("Ki: ");
//	Serial.print(ki, 4);
//	Serial.print(", ");
//	Serial.print("Kd: ");
//	Serial.print(kd, 4);
//	Serial.print(" - Control Mode: ");
	if (regulate)
	{
//		Serial.print("PID - Position: ");
	}else
	{
		//Serial.println(" IDLE");
		//ESC.write(80);
	}

	lastButtonState    = currentButtonState;
    currentButtonState = digitalRead(RED_BUTTON);

	// If the button is pressed, toggle the on/off the control loop
	if(lastButtonState == HIGH && currentButtonState == LOW) {
    regulate = !regulate;
    }

	// If the regulate bool is true, then start PID
	if (regulate)
	{
		
		actuate();

		//-----PID control-----


		//int target = 250*sin(prevT/1e6);

		//PID constants (commented out due to potentiometer implementation)
		//   float kp = 1;
		//   float kd = 0;
		//   float ki = 0;
    
		//distanc from wall
		pos = -roll;
		
		// error
		float e = pos-target;

   if(e < 0)
   {
    e = 0;
   }

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
    pwr = 80+pwr;
		if( pwr > 180 ){
			pwr = 180;
		}
		
		//SEND PWM
		
		//ESC.write(pwr);

		// store previous error
		eprev = e;

		  //Serial.print(roll);
		  //Serial.print(",");
		  //Serial.print(pos);
      //Serial.print(" ");
      //Serial.print(target);
      //Serial.print(",");
      //Serial.print(" ");
  	  //Serial.println(e);
      //Serial.print(",");
      //Serial.print(pwr);
      //Serial.print(",");
      //Serial.print(kp,4);
      //Serial.print(",");
      //Serial.print(ki,4);
      //Serial.print(",");
      //Serial.print(kd,4);
      //Serial.print(",");
      //Serial.println(millis());
		
  }
  else{
      disengage();
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
