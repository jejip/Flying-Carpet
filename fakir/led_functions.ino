void stuurled()
{
  if (left > Output)
       {
         analogWrite(10, HIGH);
       }
       else
       {
         analogWrite(10, LOW);
       }
       if (right > Output)
       {
         analogWrite(11, HIGH);
       }
       else
       {
         analogWrite(11, LOW);
       }    
}

void calibrationled()
{
   analogWrite(10, ledval);
   analogWrite(11, ledval);
}
