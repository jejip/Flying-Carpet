#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>

#include <Wire.h>

float angles[3];

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
  
  sixDOF.getAngles(angles);

  Serial.println(angles[1]); //only display theta
  
  delay(100); 
}

