/*********************************************************
*  Aansturing van de MD03 motor controller               *
*  motor is in analog mode                               *
*  Sensor wordt via I2C gelezen                          *
*                                                        *
*  Fakir inc. 2013         Tim                           *
*********************************************************/


/************************************ TODO: zet de PWM frequentie hoger dan 20 khz *******************************/



#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>

#include <Wire.h>

int pwmPin = 3;
float angles[3];       //[0]-->Yaw, [1]-->pitch & [2]-->roll

FreeSixIMU sensor = FreeSixIMU();        // aanmaken van FreeSixIMU object

void setup() {
  pinMode(pwmPin, OUTPUT);               // activeer pinmode voor de pwm 
  pinMode(11, OUTPUT);
  
  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);        // zorgt ervoor dat de frequency van de PMW op FAST PMW gezet word dit is de maximale frequency haalbaar: 16 MHz / 64 / 256 = 976.5625Hz 
  TCCR2B = _BV(CS22);
  
  TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM01) | _BV(WGM00); 
  TCCR0B = _BV(CS00);
  
  OCR2A = 180;
  OCR2B = 50;
  
  Serial.begin(9600);                    // activeer serial communicatie
  Wire.begin();                          // initialiseer I2C communicatiemet sensor
  
  delay(5);
  sensor.init();                         // begin the IMU
  delay(5);
  
  delay(100);
  motorTest();
  delay(100);
}

void loop(){
  Serial.println(getSensorValue());
  sendPWM(getSensorValue());
}                              // einde van de main loop


// functie om de waarde van de sensor uit te lezen
double getSensorValue(){
  sensor.getEuler(angles);
  double value = angles[2];
}

void sendPWM(double sensorValue){
    analogWrite(pwmPin, sensorValue);
}

// functie omde motor mee te testen
void motorTest(){
  for(int i = 0; i<255; i++){             // accelereer van 0 tot max snelheid
    analogWrite(pwmPin, i);
    delay(2);
  } 
  for(int i = 255; i >= 0; i--) {         // decelereer van max snelheid naar 0 
    analogWrite(pwmPin, i);
    delay(2);
  }
}
