// *** motor functions ***

void sendData(byte dirReg, byte dirVal, byte speedReg, byte speedVal_left, byte speedVal_right){         // Function for sending data to MD03
  Wire.beginTransmission(ADDRESS1);         // Send data to MD03
    Wire.write(dirReg);
    Wire.write(dirVal);
    Wire.write(speedReg);
    Wire.write(speedVal_left);
  Wire.endTransmission();
//  Serial.print("Wire return: ");
//  Serial.println(b);
  
  Wire.beginTransmission(ADDRESS2);
    Wire.write(dirReg);
    Wire.write(dirVal);
    Wire.write(speedReg);
    Wire.write(speedVal_right);
  Wire.endTransmission();
//  Serial.println(b);  
}  


