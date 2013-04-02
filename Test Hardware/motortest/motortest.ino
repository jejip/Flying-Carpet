#include <Wire.h>

#define ADDRESS1            0x58                    // Address of MD03 for motor 1
#define ADDRESS2            0x5B                    // Address of MD03 for motor 2
#define SOFTREG             0x07                    // Byte to read software
#define CMDBYTE             0x00                    // Command byte
#define SPEEDBYTE           0x02                    // Byte to write to speed register
#define TEMPREG             0x04                    // Byte to read temprature
#define CURRENTREG          0x05                    // Byte to read motor current

byte motorDir = 1;

void setup(){
  Serial.begin(9600);                    // activeer serial communicatie
  Wire.begin();                         // initialiseer I2C communicatiemet sensor 
}

void loop(){
for (int i = 0; i < 255; i += 10){
  sendData(CMDBYTE, motorDir, SPEEDBYTE, (byte)i);
delay(20);
}

for (int i =255; i > 0; i -= 10){
    sendData(CMDBYTE, motorDir, SPEEDBYTE, (byte)i);
delay(20);
}
}

void sendData(byte dirReg, byte dirVal, byte speedReg, byte speedVal){         // Function for sending data to MD03
  Wire.beginTransmission(ADDRESS1);         // Send data to MD03
    Wire.write(dirReg);
    Wire.write(dirVal);
    Wire.write(speedReg);
    Wire.write(speedVal);
  Wire.endTransmission(true);

  
  Wire.beginTransmission(ADDRESS2);
    Wire.write(dirReg);
    Wire.write(dirVal);
    Wire.write(speedReg);
    Wire.write(speedVal);
  Wire.endTransmission();
}
