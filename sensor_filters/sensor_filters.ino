#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>

#include <Wire.h>
#include <PID_v1.h>


#define ADDRESS1            0x58                    // Address of MD03 for motor 1
#define ADDRESS2            0x59                    // Address of MD03 for motor 2
#define SOFTREG             0x07                    // Byte to read software
#define CMDBYTE             0x00                    // Command byte
#define SPEEDBYTE           0x02                    // Byte to write to speed register
#define TEMPREG             0x04                    // Byte to read temprature
#define CURRENTREG          0x05                    // Byte to read motor current


//PID variabelen
double Input, Output;
 // PID waarden initialiseren.
double Kp = 2311.3817521;
double Ki = 0;
double Kd = 253.03382238;
double Setpoint = 0.01;

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
  
  Serial.println("reset");
  int software = getData(SOFTREG);
  Serial.print("MDO3 softaware versie: ");
  Serial.println(software);
  
  
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
  angle = angles[1] - offset;
  sendPlotData("hoek van sensor", angle);
  
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
//  // output berekenen met PID, input is de hoekin radialen.
  Input = (angle * 3.14159/180);
  myPID.Compute();  
  
  // sending I2C data
  sendData(CMDBYTE, motorDir, SPEEDBYTE, Output);

  // sending data to plot  
  sendPlotData("Output", Output);
  sendPlotData("Motor Direction", motorDir);
}

byte getData(byte reg){                   // function for getting data from MD03
  Wire.beginTransmission(ADDRESS1);
    Wire.write(reg);
  Wire.endTransmission();
  
  Wire.requestFrom(ADDRESS1, 1);         // Requests byte from MD03
  while(Wire.available() < 1);          // Waits for byte to become availble
  byte data = Wire.read();

  return(data);
}

void sendData(byte dirReg, byte dirVal, byte speedReg, byte speedVal){         // Function for sending data to MD03
  Wire.beginTransmission(ADDRESS1);         // Send data to MD03
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


  

