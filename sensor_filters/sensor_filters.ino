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


//PID variabelen
double Input, Output;
 // PID waarden initialiseren.
double Kp = 24069.31;
double Ki = 0;
double Kd = 2004.31;
double Setpoint = 0;

//PID: Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint,Kp,Ki,Kd, DIRECT);

float angles[3];
float angle;
double offset;

FreeSixIMU sensor = FreeSixIMU();        // aanmaken van FreeSixIMU object

//I2C
byte motorDir = 1;

void setup(){
  Serial.begin(9600);                    // activeer serial communicatie
  Wire.begin();                         // initialiseer I2C communicatiemet sensor 
  delay(5);
  sensor.init();                         // begin the IMU
  delay(5);
  
  Serial.print("reset");
  sendData(CMDBYTE, motorDir, SPEEDBYTE, 0);
  
  // calibreren van de sensor neemt de gemiddelde sensor waarde van 3 seconden, 
  // vervolgens meet hij de hoke nog een keer, als de error groter is dan 1 graden begint de calibratie opnieuw
  double offset = 0;
  double check = 1000;
  while (abs(offset - check) > 1){
    for(int i = 0; i < 30; i++){
    sensor.getEuler(angles);
    offset += angles[1];
    delay(10);
    }
    offset = offset / 30;
    sensor.getEuler(angles);
    check = angles[1];
  }  
  sensor.getEuler(angles);
  Input = angles[1] - offset;
  
  //turn the PID on
  myPID.SetMode(AUTOMATIC);  
}

void loop(){
  sensor.getEuler(angles);
  delay(10);
  angle = angles[1]-offset;
  
  if(angle <= 2 && angle >= -2){
    angle = 0;
  }
  else if(angle > 0){
    angle = angle * -1;
    motorDir = 1;
  } 
  else{
    motorDir = 2;
  }
 // output berekenen met PID, input is de hoekin radialen.
  Input = (angle * 3.14159/180);
  Serial.print(Input);
  Serial.println(" voor PID");
  myPID.Compute();
  
  byte b = (byte)Output;
  Serial.print(b);
  Serial.println(" na PID"); 
  Serial.println();
  sendData(CMDBYTE, motorDir, SPEEDBYTE, (b)); 
  
//  for(int i = 0; i < 10000; i++){
//      Serial.println(i % 100);
//      sendData(CMDBYTE, motorDir, SPEEDBYTE, (byte)(i % 100)); 
//  }
  

  //serial read voor PID waardes
  //if (Serial.available() > 0 {
    //read incoming byte
    
  
  //kijk of de wire.endtransmission een fout geeft
//  int return_value = Wire.endTransmission ();
//Serial.print ("end returns:");
//Serial.println (return_value);

  // sending I2C data

}


void sendData(byte dirReg, byte dirVal, byte speedReg, byte speedVal){         // Function for sending data to MD03
  Wire.beginTransmission(ADDRESS1);         // Send data to MD03
    Wire.write(dirReg);
    Wire.write(dirVal);
    Wire.write(speedReg);
    Wire.write(speedVal);
  byte b = Wire.endTransmission(true);
  Serial.print("Wire return: ");
  Serial.println(b);
  
  Wire.beginTransmission(ADDRESS2);
    Wire.write(dirReg);
    Wire.write(dirVal);
    Wire.write(speedReg);
    Wire.write(speedVal);
  b = Wire.endTransmission();
  Serial.println(b);  
}

void sendPlotData(String seriesName, float data){
    Serial.print("{");
    Serial.print(seriesName);
    Serial.print(",T,");
    Serial.print(data);
    Serial.println("}");
} 


  

