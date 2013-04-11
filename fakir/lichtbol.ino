// functie voor het instellen van het licht in de glazen bol
void licht(int bolvar){
 if (lightvalue == 17) //loop lightvalue voor hoeveel modus er zijn
      lightvalue = 0;
  
  if (lightvalue != bolvar)
  {   
      if (millis() - lastmillisbol > 20) //delay 20
      {
        lastmillisbol = millis();
        digitalWrite(bolpin, HIGH); //relais aan
      }
      
      if (millis() - lastmillisbol > 20) //delay 20
      {
        lastmillisbol = millis();
        
        digitalWrite(bolpin, LOW); //relais uit
      
        lightvalue++; 
        EEPROM.write(0, lightvalue); //write to EEPROM address 0
      }

  }
}
