/*
 * EEPROM Write
 *
 * Stores values read from analog input 0 into the EEPROM.
 * These values will stay in the EEPROM when the board is
 * turned off and may be retrieved later by another sketch.
 */

#include <EEPROM.h>

int lightvalue = 18;
int led = 7;
int incoming = 0;
int var = 10;

void setup()
{
  Serial.begin(9600);  
  
  pinMode(led, OUTPUT);     
}

void loop()
{

  //lightvalue = EEPROM.read(0);

  //EEPROM.write(0, lightvalue); //write to EEPROM address 0
  if (Serial.available() > 0) {
      incoming = Serial.read();
      incoming -= 48;
      var = incoming;
      
      Serial.print("I received: ");
      Serial.println(incoming);
  
  
  while (lightvalue != var)
  {
//        if (lightvalue = 4)
        //lightvalue = 0; 
        
    
      digitalWrite(led, HIGH);
      delay(20);               
      digitalWrite(led, LOW);    
      
      lightvalue += 1;
      Serial.println(lightvalue);
      //EEPROM.write(0, lightvalue);
      
      delay(20);

  }
  
//  else 
//    EEPROM.write(0, lightvalue);


 
}
