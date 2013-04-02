
#define GRAVITY 101 //this equivalent to 1G in the raw data coming from the accelerometer 
#define Accel_Scale(x) x*(GRAVITY/9.81)//Scaling the raw data of the accel to actual acceleration in meters for seconds square

#define Gyro_Gain 2.5 //2.5 Gyro gain
#define Gyro_Scaled(x) x*((Gyro_Gain*PI)/360)//Return the scaled ADC raw data of the gyro in radians for second
#define G_Dt(x) x*.02 //DT .02 = 20 miliseconds, value used in derivations and integrations

#define ToRad(x) (x*PI)/180.0
#define ToDeg(x) (x*180.0)/PI

#define PRINT_ANALOGS 1 // If 1 will print the analog raw data
#define PRINT_KALMAN 1 //Print kalman
#define PRINT_GPS 1

#define PROCESS_GPS 1

//Sensor: GYROX, GYROY, GYROZ, ACCELX, ACCELY, ACCELZ
float SENSOR_SIGN[]={-1,-1,-1,-1,1,-1}; //{1,1,-1,1,-1,1}Used to change the polarity of the sensors{-1,1,-1,-1,-1,1}

int long timer=0; //general porpuse timer 
int long timer24=0; //Second timer used to print values 
int AN[8]; //array that store the 6 ADC filtered data
int AN_OFFSET[8]; //Array that stores the Offset of the gyrosa
int EX[8]; //General porpuse array to send information

unsigned long Now = millis();
unsigned long lastread = Now;

float dt; // = .016384;
 
//GPS 

//GPS stuff
union long_union {
	int32_t dword;
	uint8_t  byte[4];
} longUnion;

union int_union {
	int16_t word;
	uint8_t  byte[2];
} intUnion;

/*Flight GPS variables*/
int gpsFix=1; //This variable store the status of the GPS
float lat=0; // store the Latitude from the gps
float lon=0;// Store guess what?
float alt_MSL=0; //This is the alt.
float ground_speed=0;// This is the velocity your "plane" is traveling in meters for second, 1Meters/Second= 3.6Km/H = 1.944 knots
float ground_course=90;//This is the runaway direction of you "plane" in degrees
float climb_rate=0; //This is the velocity you plane will impact the ground (in case of being negative) in meters for seconds
char data_update_event=0; 

//uBlox Checksum
byte ck_a=0;
byte ck_b=0;
long iTOW=0; //GPS Millisecond Time of Week
long alt=0; //Height above Ellipsoid 
float speed_3d=0; //Speed (3-D)  (not used)


volatile uint8_t MuxSel=0;
volatile uint8_t analog_reference = DEFAULT;
volatile int16_t analog_buffer[8];


#include <math.h>
#include "kalman.h"


KALDATA xkaldata;
KALDATA ykaldata;

void test(float value[9],int pos)
{
  Serial.print(convert_to_dec(value[pos]));
}

void setup()
{
  Serial.begin(38400);
  pinMode(2,OUTPUT); //Serial Mux
  digitalWrite(2,HIGH); //Serial Mux
  pinMode(5,OUTPUT); //Red LED
  pinMode(6,OUTPUT); // BLue LED
  pinMode(7,OUTPUT); // Yellow LED
  Analog_Reference(EXTERNAL);//Using external analog reference
  Analog_Init();
  
  
  for(int c=0; c<75; c++)
  {
    read_adc_raw();
    
    digitalWrite(7,LOW);
    digitalWrite(6,HIGH);
    digitalWrite(5,LOW);
    delay(50);
    digitalWrite(7,HIGH);
    digitalWrite(6,LOW);
    digitalWrite(5,HIGH);
    delay(50);
  }
  digitalWrite(5,LOW);
  digitalWrite(7,LOW);
  
  for(int y=0; y<=7; y++)
  {
    AN_OFFSET[y]=AN[y];
    Serial.println((int)AN_OFFSET[y]);
  }
    AN_OFFSET[5]=AN[5]-GRAVITY;
    
    kalmanInitState(&xkaldata);
    kalmanInitState(&ykaldata);
}

void loop()
{   
    Now = millis();
    dt = (Now - lastread) * .001; //compute the time delta since last iteration, in sec.
   
    read_adc_raw(); //ADC Stuff

    state_update(Gyro_Scaled(read_adc(1)),&xkaldata, dt);
    state_update(Gyro_Scaled(read_adc(0)),&ykaldata, dt);
    
    kalman_update( (atan2(-read_adc(5),read_adc(3))-(3.14159/2)) ,&xkaldata);
    kalman_update( (atan2(-read_adc(5),read_adc(4))-(3.14159/2)) ,&ykaldata);
    
  
    if((millis()-timer24)>=100)
    {
      timer24=millis();
      #if PROCESS_GPS == 1
      decode_gps();
      #endif
      printdata(); //Send info via serial
    }
    
    lastread = Now;
}

