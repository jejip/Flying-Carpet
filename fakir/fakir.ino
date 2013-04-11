#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>

#include <Wire.h>
#include <PID_v1.h>

#define ADDRESS1            0x58                    // Address of MD03 for motor 1
#define ADDRESS2            0x5B                    // Address of MD03 for motor 2
#define SOFTREG             0x07                    // Byte to read software
#define CMDBYTE             0x00                    // Command byte
#define SPEEDBYTE           0x02                    // Byte to write to speed register
#define TEMPREG             0x04                    // Byte to read temprature
#define CURRENTREG          0x05                    // Byte to read motor current

int kill = 0; //for the killswitch
int ledval = 1; //led value 

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
double Ki = 20;
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

byte motorDir = 1; //richting van de motor, naar voren of achteren


// *** Steering parameters***

byte b_left, b_right;
int steerPin = A0;
int steerValue;
double left, right;


void setup(){
  
  Serial.begin(115200);                    // activeer serial communicatie
  Wire.begin();                         // initialiseer I2C communicatie
  delay(5);
  sensor.init();                         // begin the IMU
  delay(5);
  
  sendData(CMDBYTE, motorDir, SPEEDBYTE, 0, 0); //setup motor

  myPID.SetMode(AUTOMATIC); //turn the PID on

  pinMode(2, INPUT); //killswitch on pin 2 is input
  pinMode(10, OUTPUT); //led
  pinMode(11, OUTPUT); //led
  
}

void loop(){
    
  //beginning loop, set the calibration LED on
  digitalWrite(13, HIGH);
  calibrationled();
  
  //Angles
  sensor.getEuler(angles);
  roll = angles[2]* (3.14159/180)+0.054;
  pitch = angles[1]* (-3.14159/180);
  yaw = angles[0]* (3.14159/180);
  
  //angles for turning
    angle = cos(yaw)*roll + sin(yaw)*pitch;


  //angle += 0.054; //3.15 graden; //offset voor de sensorplaatsing
  constrain(angle, -.366, .366); //high pass filter voor de hoek 21 graden
  
  
  //Adaptive PID tuning voor opstaan
  if(angle > 18 || angle < -18) //andere P waarde voor opstaan
  {
    myPID.SetTunings(startKp, Ki, Kd);
  }
  else if(angle > 12 || angle < -12) //andere P waarde voor opstaan
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
    motorDir = 1;
  } 
  else{
    motorDir = 2;
  }
  
  //hoeksnelheid bepalen voor D
  if (millis()+delta > oldt)
  {
  angledelta = (angle - angleold);
  anglespeed = (angledelta / delta);
  //reset variables
  oldt = millis();
  angleold = angle;
  }
  
 // output berekenen met PID, input is de hoek in radialen.
  Input = (angle);// * (3.14159/180));
  dInput = (anglespeed);// * (3.14159/180));
  myPID.Compute();
  
    // Bereken stuur waarden en pas ze toe op de output.
  steerValue = analogRead(steerPin);

     if(steerValue < 500){
       left  = Output + (510 - steerValue) / 15;
       right = Output; //- 10(510 - steerValue) / 25;
     }

     else if(steerValue > 523){
        left = Output; //- 10(steerValue - 513) / 25;
        right = Output + (steerValue - 513) / 15;       
     }

     else {
       left = Output;
       right = Output;
     }
  
    stuurled();  //LEDjes voor richting aangeven

     //stuur data naar de motor
  b_left = (byte)left;
  b_right = (byte)right;
  
//  b_left = (byte)Output; // + (byte)steerOffset_left;
//  b_right = (byte)Output; // + (byte)steerOffset_right;
  
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
  }
  else{
    myPID.SetMode(AUTOMATIC); //zet anders de PID aan
  }
  
  
  sendData(CMDBYTE, motorDir, SPEEDBYTE, b_left, b_right); //stuur data naar de motor


} //end loop
