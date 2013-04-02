/*******************************************************
*             MD03 example for Arduino                 *
*   MD03 is in I2C mode LCD03 controlled by serial     *
*                                                      *
*             By James Henderson 2012                  *
*******************************************************/

#include <Wire.h>
#include <SoftwareSerial.h>

#define ADDRESS             0x58                    // Address of MD03
#define SOFTREG             0x07                    // Byte to read software
#define CMDBYTE             0x00                    // Command byte
#define SPEEDBYTE           0x02                    // Byte to write to speed register
#define TEMPREG             0x04                    // Byte to read temprature
#define CURRENTREG          0x05                    // Byte to read motor current

//#define LCD_RX              0x02                    // Pin for rx
//#define LCD_TX              0x03                    // Pin for tx
//#define LCD03_HIDE_CUR      0x04
//#define LCD03_CLEAR         0x0C
//#define LCD03_SET_CUR       0x02


byte direct = 1;                                          // Stores what direction the motor should run in

int sensorPinDrive = A0;
int sensorPinDirection = A1;
boolean drive = false;



void setup(){
  Serial.begin(9600);                                    // Begin serial for Serial port
  
  Wire.begin();
  delay(100);
  
  int software = getData(SOFTREG);                       // Gets software version and prints it to LCD03
  Serial.print("MD03 Example  V:");
  Serial.print(software);


}

void loop(){
  if(analogRead(sensorPinDrive) > 1000)
    drive = true;
  
  if(drive){
    sendData(CMDBYTE, direct);
    sendData(SPEEDBYTE, 50);    
    int temp = getData(TEMPREG);
    int current = getData(CURRENTREG);  // Gets motor current
    Serial.print("temprature: ");
    Serial.print(temp);
    Serial.print(" Motor current: ");
    Serial.println(current);
  }
  else{
    sendData(CMDBYTE, direct);
    sendData(SPEEDBYTE, 0);    
  }
  setDirection();
} // end of main loop

byte getData(byte reg){                   // function for getting data from MD03
  Wire.beginTransmission(ADDRESS);
    Wire.write(reg);
  Wire.endTransmission();
  
  Wire.requestFrom(ADDRESS, 1);         // Requests byte from MD03
  while(Wire.available() < 1);          // Waits for byte to become availble
  byte data = Wire.read();

  return(data);
}

void sendData(byte reg, byte val){         // Function for sending data to MD03
  Wire.beginTransmission(ADDRESS);         // Send data to MD03
    Wire.write(reg);
    Wire.write(val);
  Wire.endTransmission();
}

void setDirection(){
  int sensor;
  sensor = analogRead(sensorPinDirection);
  if(sensor > 1000){
    direct = 2;
  }
  else{
    direct = 1;
  }  
}


