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
  
 sixDOF.getEuler(angles);

  Serial.print(angles[0]* (3.14159/180)); //yaw
  Serial.print(angles[1]* (3.14159/180)); //pitch
  Serial.println(angles[2]* (3.14159/180)); //roll
  
  delay(100); 
}

