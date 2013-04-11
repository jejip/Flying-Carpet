#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>

#include <Wire.h>
#include <EEPROM.h>
#include <PID_v1.h>

#define ADDRESS1            0x58                    // Address of MD03 for motor 1
#define ADDRESS2            0x5B                    // Address of MD03 for motor 2
#define SOFTREG             0x07                    // Byte to read software
#define CMDBYTE             0x00                    // Command byte
#define SPEEDBYTE           0x02                    // Byte to write to speed register
#define TEMPREG             0x04                    // Byte to read temprature
#define CURRENTREG          0x05                    // Byte to read motor current

int kill = 0; //for the killswitch

//lichtbol variabelen
int lightvalue;
int bolpin = 10; //pin waar de bol aan zit
long lastmillisbol = 0; //voor de delay

// voor hoeksnelheid
int oldt = 0;
int angledelta;
int angleold;
int anglespeed = 0;
int delta = 100; //time for calculating speed

// *** PID ***

//PID variables
double Input, dInput, Output;
// PID initialising variables
double Kp = 930;
double Ki = 5;
double Kd = 20;
double Setpoint = 0;

double startKp = 150; // Kp value for start position
double start2Kp = 350; //Kp for between start and normal

//Specify the links and initial tuning parameters
PID myPID(&Input, &dInput, &Output, &Setpoint,Kp,Ki,Kd, DIRECT);

//unsigned long serialTime; //this will help us know when to talk with processing


/// *** Sensor ***

float angles[3];
float angle, yaw, pitch, roll;


FreeSixIMU sensor = FreeSixIMU();        // create FreeSixIMU object

byte motorDir_left = 1; //richting van de motor, naar voren of achteren
byte motorDir_right = 1;

// *** Steering parameters***

byte b_left, b_right;
int steerPin = A0;
int steerValue;
double left, right;


void setup(){
//  Serial.begin(115200);                    // activeer serial communicatie

  pinMode(2, INPUT); //killswitch on pin 2 is input
  pinMode(10, OUTPUT); //bol lampen
  
  lightvalue = EEPROM.read(0); //lees de oude lightvalue
  licht(6); //zet de lamp op rood
  
  Wire.begin();                         // initialiseer I2C communicatie
  delay(5);
  sensor.init();                         // begin the IMU
  delay(5);
  
  sendData(CMDBYTE, motorDir_left, motorDir_right, SPEEDBYTE, 0, 0); //setup motor

  myPID.SetMode(AUTOMATIC); //turn the PID on
}

void loop(){
    
  //beginning loop, set the calibration LED on
 digitalWrite(13, HIGH);
//  calibrationled();
  
  //Angles
  sensor.getEuler(angles); //krijg de euler hoek door
  //Bereken meteen de radialen uit de hoek
  roll = angles[2]* (3.14159/180)+0.04;
  pitch = angles[1]* (-3.14159/180)+0.04;
  yaw = angles[0]* (3.14159/180);
  
  //switch angles for turning
    angle = cos(yaw)*roll + sin(yaw)*pitch;

  constrain(angle, -.366, .366); //high pass filter voor de hoek 21 graden, want mechanisch kan die geen grotere hoek maken
  
  
  //Adaptive PID tuning voor opstaan
  if(angle > 18 * (3.14159/180) || angle < -18 * (3.14159/180) ) //andere P waarde voor opstaan
  {
    myPID.SetTunings(startKp, Ki, Kd);
  }
  else if(angle > 12 * (3.14159/180) || angle < -12 * (3.14159/180) ) //andere P waarde voor opstaan 2 
  {
    myPID.SetTunings(start2Kp, Ki, Kd);
  }
  else
  {
    myPID.SetTunings(Kp, Ki, Kd);
  }
  

  //bepaal de motorrichting
  if(angle > 0){
    angle = angle * -1;
    motorDir_left = 1;
    motorDir_right = 1;
  } 
  else{
    motorDir_left = 2;
    motorDir_right = 2;
  }
  
  //hoeksnelheid bepalen voor Kd
  if (millis()+delta > oldt)
  {
  angledelta = (angle - angleold);
  anglespeed = (angledelta / delta);
  //reset variables
  oldt = millis();
  angleold = angle;
  }
  
 // output berekenen met PID
  Input = (angle);
  dInput = (anglespeed);
  myPID.Compute();
  
    // lees stuur waarden en pas ze toe op de output.
  steerValue = analogRead(steerPin);

//sturen op de plek als de hoek van de sensor is kleiener dan 2 graden draaien de wielen in tegengestelde righting.
//  if(angle * (180/4.14159) > -2){
//    if(steerValue < 500){
//       left  = Output + (510 - steerValue) / 25;
//       right = Output + (510 - steerValue) / 25;
//       int dir_right = (motorDir_left - 3) *-1;
//       motorDir_right = (byte)dir_right;
//     }
//
//     else if(steerValue > 523){
//        left = Output + (steerValue - 513) / 25;
//        right = Output + (steerValue - 513) / 25; 
//        int dir_left = (motorDir_right - 3) *-1;
//        motorDir_left = (byte)dir_left;      
//     }
// 
//     else {
//       left = Output;
//       right = Output;
//     }
//  }
  
  // sturen met snelheid

     if(steerValue < 500){
       right  = Output + (510 - steerValue) / 15;
       left = Output; //- 10(510 - steerValue) / 25;
     }

     else if(steerValue > 523){
        right = Output; //- 10(steerValue - 513) / 25;
        left = Output + (steerValue - 513) / 15;       
     }

     else {
       left = Output;
       right = Output;
     }
 
  
     //stuur data naar de motor
  b_left = (byte)left;
  b_right = (byte)right;
  
      //send-receive with processing if it's time
//  if(millis()>serialTime)
//  {
//    SerialReceive();
//    SerialSend();
//    serialTime+=20;
//  }
  
      //killswitch, voordat hij waardes naar de motor stuurt
kill = digitalRead(7);
if(kill == LOW){ //ga uit 
  Output = 0;
  b_left = 0; //zorg dat de motoren uit staan
  b_right = 0;
  myPID.SetMode(MANUAL); //zet de PID uit
  licht(8); // zet de lamp op groen
  }
  else{
    myPID.SetMode(AUTOMATIC); //zet anders de PID aan
    licht(4);
  }
  
  
  sendData(CMDBYTE, motorDir_left, motorDir_right, SPEEDBYTE, b_left, b_right); //stuur data naar de motor


} //end loop
