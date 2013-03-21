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
double Kp = 50;
double Ki = 0;
double Kd = 50;
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
//  double offset = 0;
//  double check = 1000;
//  while (abs(offset - check) > 1){
//    for(int i = 0; i < 30; i++){
//    sensor.getEuler(angles);
//    offset += angles[1];
//    delay(10);
//    }
//    offset = offset / 30;
//    sensor.getEuler(angles);
//    check = angles[1];
//  }  
//  sensor.getEuler(angles);
//  Input = angles[1] - offset;
//  
  //turn the PID on
  myPID.SetMode(AUTOMATIC);  
}

void loop(){
  sensor.getEuler(angles);
  angle = angles[1];
  
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
  Serial.println();
  myPID.Compute();
  Serial.print(Output);
  Serial.println(" na PID");  
  
  // sending I2C data
  if(Output > 100){
    Output = 100;
  }
  sendData(CMDBYTE, motorDir, SPEEDBYTE, Output);
  Serial.print( angle );
  Serial.print(" ");
  Serial.println(Output);
  delay(10);
  
  //kijk of de wire.endtransmission een fout geeft
//  int return_value = Wire.endTransmission ();
//Serial.print ("end returns:");
//Serial.println (return_value);
  
}


void sendData(byte dirReg, byte dirVal, byte speedReg, byte speedVal){         // Function for sending data to MD03
  Wire.beginTransmission(ADDRESS1);         // Send data to MD03
    Wire.write(dirReg);
    Wire.write(dirVal);
    Wire.write(speedReg);
    Wire.write(speedVal);
  Wire.endTransmission();
  Wire.beginTransmission(ADDRESS2);
    Wire.write(dirReg);
    Wire.write(dirVal);
    Wire.write(speedReg);
    Wire.write(speedVal);
  Wire.endTransmission();
}

void sendPlotData(String seriesName, float data){
    Serial.print("{");
    Serial.print(seriesName);
    Serial.print(",T,");
    Serial.print(data);
    Serial.println("}");
} 


  

