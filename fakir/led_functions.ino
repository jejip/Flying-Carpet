void stuurled()
{
  if (left > Output)
       {
         digitalWrite(10, 255);
       }
       else
       {
         digitalWrite(10, 50);
       }
       if (right > Output)
       {
         digitalWrite(11, 255);
       }
       else
       {
         digitalWrite(11, 50);
       }    
}

void calibrationled()
{
   analogWrite(10, 50);
   analogWrite(11, 50);
}
