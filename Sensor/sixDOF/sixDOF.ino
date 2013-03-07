#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>

#include <Wire.h>

float angles[3]; // yaw pitch roll
float theta;
float motor;

boolean motordir = true;

const int analogOutPin = 9;

// Set the FreeSixIMU object
FreeSixIMU sixDOF = FreeSixIMU();

void setup() { 
  Serial.begin(9600);
  Wire.begin();
  
  delay(5);
  sixDOF.init(); //begin the IMU
  delay(5);
}

void loop() { 
  
  sixDOF.getEuler(angles);
  
//  Serial.print(angles[0]);
//  Serial.print(" | ");  
//  Serial.print(angles[1]); //pitch in theta
//  Serial.print(" | ");
//  Serial.println(angles[2]);
  
//  delay(100); 
  
  theta = angles[1];
  
  motor = map(theta, -50, 50, 0, 250);
  
  if (motor < 0)
    {
      motordir = false;
    }
    
  if (motor > 0)
    {
      motordir = true;
    }
  
     if (motor > 255)
    {
      motor = 255;
    }
    
    motor = abs(motor);
  
  Serial.print(motor);
  Serial.println(motordir);
  
  analogWrite(analogOutPin, motor);
 
  
}

