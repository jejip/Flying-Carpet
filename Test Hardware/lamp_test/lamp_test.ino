/*
Glazen bol licht
 */

#include <EEPROM.h>

int lightvalue = 1;
int led = 7;
int incoming = 0;
int var = 5;

void setup()
{
  Serial.begin(9600);  
  
  lightvalue = EEPROM.read(0); //lees de oude lightvalue
  
  pinMode(led, OUTPUT);     
}

void loop()
{
  if (Serial.available() > 0) {
      incoming = Serial.read();
      incoming -= 48;
      var = incoming;
      
      Serial.print("I received: ");
      Serial.println(var);
  }
  
  if (lightvalue == var)
  {
    Serial.print(lightvalue);
    Serial.println("joepie");
    delay(20);
  }
  else
  {    
      digitalWrite(led, HIGH);
      delay(20);               
      digitalWrite(led, LOW);    
      
      lightvalue++;
      EEPROM.write(0, lightvalue); //write to EEPROM address 0
      
      delay(20);
  }
 
if (lightvalue == 18)
      lightvalue = 0;
 
}
