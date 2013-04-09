/*
 * EEPROM Write
 *
 * Stores values read from analog input 0 into the EEPROM.
 * These values will stay in the EEPROM when the board is
 * turned off and may be retrieved later by another sketch.
 */

#include <EEPROM.h>

int lightvalue = 18;

void setup()
{
}

void loop()
{

  lightvalue = EEPROM.read(0);

  EEPROM.write(0, lightvalue); //write to EEPROM address 0

  
  delay(100);
}
