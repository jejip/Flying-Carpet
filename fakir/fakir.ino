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

int buttonState = 0; //for the killswitch

// voor hoeksnelheid
int oldt = 0;
int angledelta;
int angleold;
int anglespeed = 0;
int delta = 100; //time for calculating speed

int totalspeed = 0;

// *** PID ***

//PID variables
double Input, dInput, Output;
// PID initialising variables
double Kp = 930;
double Ki = 0;
double Kd = 20;
double Setpoint = 0;
double startKp = 150; // Kp value for start position
double start2Kp = 350; //Kp for between start and normal

//Specify the links and initial tuning parameters
PID myPID(&Input, &dInput, &Output, &Setpoint,Kp,Ki,Kd, DIRECT);

unsigned long serialTime; //this will help us know when to talk with processing


/// *** Sensor ***

float angles[3];
float angle;

FreeSixIMU sensor = FreeSixIMU();        // create FreeSixIMU object

byte motorDir = 1; //richting van de motor, naar voren of achteren

byte b_left, b_right;

// *** Steering parameters***
int steerPin = A0;
int steerValue;
//int steerOffset_left, steerOffset_right;

double left, right;

void setup(){
  
  Serial.begin(115200);                    // activeer serial communicatie
  Wire.begin();                         // initialiseer I2C communicatie
  delay(5);
  sensor.init();                         // begin the IMU
  delay(5);
  
  sendData(CMDBYTE, motorDir, SPEEDBYTE, 0, 0); //setup motor

  myPID.SetMode(AUTOMATIC); //turn the PID on

  pinMode(2, INPUT); //killswitch on pin 2
  
}

void loop(){
    
  //beginning loop, set the calibration LED
  digitalWrite(13, HIGH);
  
  sensor.getEuler(angles);
  angle = angles[2];
  
  //dInput heoksnelheid

  angle += 3.15; //offset voor de sensorplaatsing
  //Filters voor de hoek
  constrain(angle, -21, 21); //high pass
  
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
  
 // output berekenen met PID, input is de hoekin radialen.
  Input = (angle * (3.14159/180));
  dInput = (anglespeed * (3.14159/180));
  myPID.Compute();
  
  
    // Bereken stuur waarden en pas ze toe op de output.
  steerValue = analogRead(steerPin);
     if(steerValue < 300){
       left  = Output + 10;
       right = Output;
//       steerOffset_left = 100; //(int)(300 - steerValue)/100;
//       steerOffset_right =-100; // -steerOffset_left;
     }
     else if(steerValue > 723){
        left = Output;
        right = Output + 10;       
//       steerOffset_right = 100; //(int)(steerValue - 723)/100;
//       steerOffset_left = -100; //-steerOffset_right;
     }
     else {
       left = Output;
       right = Output;
//       steerOffset_left = 0;
//       steerOffset_right = 0;
     }
  
       //LEDjes voor richting aangeven
     if (left > Output)
     {
       digitalWrite(10, HIGH);
     }
     else
     {
       digitalWrite(10, LOW);
     }
     if (right > Output)
     {
       digitalWrite(10, HIGH);
     }
     else
     {
       digitalWrite(10, LOW);
     }
     //stuur data naar de motor
  b_left = (byte)left;
  b_right = (byte)right;
  
//  b_left = (byte)Output; // + (byte)steerOffset_left;
//  b_right = (byte)Output; // + (byte)steerOffset_right;
  
      //send-receive with processing if it's time
  if(millis()>serialTime)
  {
    SerialReceive();
    SerialSend();
    serialTime+=20;
  }
  
      //killswitch, voordat hij waardes naar de motor stuurt
buttonState = digitalRead(8);

if(buttonState == LOW){ //als de switch naar zwart staat, ga uit
  Output = 0;
  myPID.SetMode(MANUAL); //zet de PID uit
  }
  else{
    myPID.SetMode(AUTOMATIC); //zet anders de PID aan
  }
  
  //stuur data naar de motor
  sendData(CMDBYTE, motorDir, SPEEDBYTE, b_left, b_right); 


} //end loop


  

