/* AHMERS
 * Active Healt Monitoring and Emergency Response System
 * 
 * Arduino Nano
 * 
 * 
 * Instructions:
 *
 *********OLED Display*************
 *  Res > D9
 *  D/C > D8
 *  CS  > D10
 *  CLK > D13
 *  DIN > D11
 *  
 *********Temperature Sensor*******
 *  OBJ > A1
 *  SUR > A0
 *  
 *********Heart Beat Sensor********
 *  SDA > A4
 *  SCL > A5
 *  
 *********Bluetooth Module*********
 *  TX  > A2
 *  RX  > A3
 *  
 *********Emergency Switch*********
 *  5V  > 5V
 *  GND > A7
 *  
 */


#include <Wire.h>
#include "MAX30105.h"
#include <Adafruit_Sensor.h>
#include "heartRate.h"
#include <math.h>
#include <SPI.h>
#include <string.h>
#include "ssd1331.h"
#include <SoftwareSerial.h>

MAX30105 particleSensor;

int flag = 1;
int updatetime = 1;
int count = 1;

//**********Heart Beat Sensor**************

const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;

int hb;
float surtemp, objtemp;

unsigned long Interval = 0;


//**********Termperature Sensor****************

#define SUR_TEMP_PIN A0 // Analog input pin connect to temperature sensor SUR pin
#define OBJ_TEMP_PIN A1 // Analog input pin connect to temperature sensor OBJ pin
float temp_calibration=0;       //this parameter was used to calibrate the temperature
//float objt_calibration=0.000; //this parameter was used to calibrate the object temperature
float temperature_range=10;    //we make a map of temperature-voltage according to sensor datasheet. 10 is the temperature step when sensor and 
                               //object distance is 9CM.
float offset_vol=0.014;        //this parameter was used to set the mid level voltage,when put the sensor in normal environment after 10 min,
                               //the sensor output 0.For example,the surrounding temperature is 29℃，but the result is 27℃ via the sensor,
                               //you should set the reerence to 0.520 or more,according to your sensor to change.
                               //the unit is V
float tempValue = 0; 
float objtValue= 0;  
float current_temp=0;
float temp=0;
float temp1=0;
float temp2=0;
unsigned int temp3=0;
const float reference_vol=0.500;
float R=0;
float voltage=0;


long res[100]={
                 318300,302903,288329,274533,261471,249100,237381,226276,215750,205768,
                 196300,187316,178788,170691,163002,155700,148766,142183,135936,130012,
                 124400,119038,113928,109059,104420,100000,95788,91775,87950,84305,
                 80830,77517,74357,71342,68466,65720,63098,60595,58202,55916,
                 53730,51645,49652,47746,45924,44180,42511,40912,39380,37910,
                 36500,35155,33866,32631,31446,30311,29222,28177,27175,26213,
                 25290,24403,23554,22738,21955,21202,20479,19783,19115,18472,
                 17260,16688,16138,15608,15098,14608,14135,13680,13242,12819,
                 12412,12020,11642,11278,10926,10587,10260,9945,9641,9347,
                 9063,8789,8525,8270,8023,7785,7555,7333,7118,6911};
                 
float obj [13][12]={
/*0*/             { 0,-0.274,-0.58,-0.922,-1.301,-1.721,-2.183,-2.691,-3.247,-3.854,-4.516,-5.236}, //
/*1*/             { 0.271,0,-0.303,-0.642,-1.018,-1.434,-1.894,-2.398,-2.951,-3.556,-4.215,-4.931},  //→surrounding temperature,from -10,0,10,...100
/*2*/             { 0.567,0.3,0,-0.335,-0.708,-1.121,-1.577,-2.078,-2.628,-3.229,-3.884,-4.597},   //↓object temperature,from -10,0,10,...110
/*3*/             { 0.891,0.628,0.331,0,-0.369,-0.778,-1.23,-1.728,-2.274,-2.871,-3.523,-4.232},
/*4*/             { 1.244,0.985,0.692,0.365,0,-0.405,-0.853,-1.347,-1.889,-2.482,-3.13,-3.835},
/*5*/             { 1.628,1.372,1.084,0.761,0.401,0,-0.444,-0.933,-1.47,-2.059,-2.702,-3.403},
/*6*/             { 2.043,1.792,1.509,1.191,0.835,0.439,0,-0.484,-1.017,-1.601,-2.24,-2.936},
/*7*/             { 2.491,2.246,1.968,1.655,1.304,0.913,0.479,0,-0.528,-1.107,-1.74,-2.431},
/*8*/             { 2.975,2.735,2.462,2.155,1.809,1.424,0.996,0.522,0,-0.573,-1.201,-1.887},
/*9*/             { 3.495,3.261,2.994,2.692,2.353,1.974,1.552,1.084,0.568,0,-0.622,-1.301},
/*10*/            { 4.053,3.825,3.565,3.27,2.937,2.564,2.148,1.687,1.177,0.616,0,-0.673},
/*11*/            { 4.651,4.43,4.177,3.888,3.562,3.196,2.787,2.332,1.829,1.275,0.666,0},
/*12*/            { 5.29,5.076,4.83,4.549,4.231,3.872,3.47,3.023,2.527,1.98,1.379,0.72}
};


//***********  OLED  ***************

#define WIDTH      96
#define HEIGHT     64
#define PAGES       8

#define OLED_RST    9 
#define OLED_DC     8
#define OLED_CS    10
#define SPI_MOSI   11    /* connect to the DIN pin of OLED */
#define SPI_SCK    13     /* connect to the CLK pin of OLED */

uint8_t oled_buf[WIDTH * HEIGHT / 8];


unsigned char a='1',b='2',c='0',d='0';
int A=1, B=2, C=0, D=0;
unsigned long startMillis = 0;  //some global variables available anywhere in the program
unsigned long currentMillis;
const unsigned long period = 1000;  //the value is a number of milliseconds


char heart[4];
char temperature[2];
int he=0, te=0;


//*************** Bluetooth ****************

SoftwareSerial bluetooth(A2,A3); //RX=A2, TX=A3
int BluetoothData[3];



void setup()
{
  bluetooth.begin(9600);
  SSD1331_begin();
  SSD1331_clear();
//  SSD1331_string(0, 2, "A H M E R S", 12, 1, ORANGE); 
//  SSD1331_string(78, 52, "bpm", 12, 1, RED);
//  SSD1331_string(16, 52, "'C", 12, 1, YELLOW);
  SSD1331_string(10, 2, "C O N N E C T", 12, 1, BLUE); 
  SSD1331_string(38, 17, "T O", 12, 1, BLUE); 
  SSD1331_string(14, 34, "A H M E R S", 12, 1, ORANGE);
  SSD1331_string(32, 50, "A P P", 12, 1, BLUE);

  analogReference(INTERNAL);//set the refenrence voltage 1.1V,the distinguishability can up to 1mV.
//  analogReference(INTERNAL1V1);//(mega only)set the refenrence voltage 1.1V,the distinguishability can up to 1mV.



  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    while (1);
  }


  byte ledBrightness = 0x1F; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 3; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  int sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode); //Configure sensor with these settings

   BluetoothData[0] = 0;
   BluetoothData[1] = 0;
   BluetoothData[2] = 0;
  
}


int HB()
{
  long irValue = particleSensor.getIR();

  if (checkForBeat(irValue) == true)
  {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }

  if (irValue < 50000)
  {
    return -1;
  }
  else
  {
    return beatAvg;
  }
}


float binSearch(long x)// this function used for measure the surrounding temperature
{
  int low,mid,high;
  low=0;
  //mid=0;
  high=100;
  while (low<=high)
  {
    mid=(low+high)/2;
    if(x<res[mid])
      low= mid+1;
    else//(x>res[mid])
      high=mid-1;
  }
  return mid;
}

float arraysearch(float x,float y)//x is the surrounding temperature,y is the object temperature
{
  int i=0;
  float tem_coefficient=100;//Magnification of 100 times  
  i=(x/10)+1;//Ambient temperature      
  voltage=(float)y/tem_coefficient;//the original voltage   
      
  for(temp3=0;temp3<13;temp3++)   
  {     
    if((voltage>obj[temp3][i])&&(voltage<obj[temp3+1][i]))        
    {     
      return temp3;         
    }     
  }
}
float measureSurTemp()
{  
  unsigned char i=0;
  float current_temp1=0;    
  int signall=0;   
  tempValue=0;

  for(i=0;i<10;i++)   
  {     
    tempValue+= analogRead(SUR_TEMP_PIN);       
    delay(10);    
  }   
  tempValue=tempValue/10;   
  temp = tempValue*1.1/1023;    
  R=2000000*temp/(2.50-temp);   
  signall=binSearch(R);    
  current_temp=signall-1+temp_calibration+(res[signall-1]-R)/(res[signall-1]-res[signall]);
  //return current_temp;
}

float measureObjectTemp()
{
  measureSurTemp();
  unsigned char i=0;  
  unsigned char j=0;  
  float sur_temp=0;  
  unsigned int array_temp=0;  
  float temp1,temp2; 
  float final_temp=0;
  objtValue=0;  
  for(i=0;i<10;i++)
  {
    objtValue+= analogRead(OBJ_TEMP_PIN); 
    delay(10); 
    }       
  objtValue=objtValue/10;//Averaging processing     
  temp1=objtValue*1.1/1023;//+objt_calibration; 
  sur_temp=temp1-(reference_vol+offset_vol);             
  array_temp=arraysearch(current_temp,sur_temp*1000);        
  temp2=current_temp;        
  temp1=(temperature_range*voltage)/(obj[array_temp+1][(int)(temp2/10)+1]-obj[array_temp][(int)(temp2/10)+1]);        
  final_temp=temp2+temp1;        
  if((final_temp>100)||(final_temp<=-10))
    {
    Serial.println ("\t out of range!");
    }
  else
    {   
      return final_temp; 
    }
}


void loop()
{ 
  reset:

  if(updatetime == 1)
  {
    while(true)
    {  
      if(bluetooth.available() > 0)
      {
        while(count<5)
        {
          gettime();
          count++;
          delay(500);
        } 
       SSD1331_clear();
       SSD1331_string(0, 2, "A H M E R S", 12, 1, ORANGE); 
       SSD1331_string(78, 52, "bpm", 12, 1, RED);
       SSD1331_string(18, 52, "'C", 12, 1, YELLOW);
       SSD1331_mono_bitmap(85, 2, Bluetooth88, 8, 8, BLUE);
       currentMillis = millis();
       Interval = currentMillis;
       startMillis = currentMillis;
       goto reset;
      }
    }
  }
  
  
  SSD1331_char3216(0,16, a, BLUE);
  SSD1331_char3216(16,16, b, BLUE);
  SSD1331_char3216(40,16, ':', RED);
  SSD1331_char3216(64,16, c, GREEN);
  SSD1331_char3216(80,16, d, GREEN);

  r:
  currentMillis = millis();  
  

  if(flag == 1)
  {
    while(currentMillis - Interval <= 30000)
    {
      hb = HB();
      if(hb>200)
      {
        hb = 200;
      }
      if(hb<60 && hb != -1 && hb != -2)
      {
        hb = 60;
      }
      itoa(hb, heart, 10);
      if(analogRead(A7) > 1020)
      {
        itoa(-2, heart, 10);
      }
      if(strlen(heart) == 2)
      {
        strcat(heart, " ");
      }
      //SSD1331_string(58, 52, "   ", 12, 1, RED);
      SSD1331_string(58, 52, heart, 12, 1, RED);
      
      currentMillis = millis();
      hb=0;
 
    }
    Interval = currentMillis;
    BluetoothData[1] = hb;
    hb=0;
    flag = 2;
  }
  
  else if(flag == 2)
  {
    while(currentMillis - Interval <= 30000)
    {
      objtemp = measureObjectTemp();
      te = (int)objtemp;
      //te=(1.8*te)+32;
      itoa(te, temperature, 10);
      if(analogRead(A7) == 1023)
      {
        itoa(-2, temperature, 10);
      }
      SSD1331_string(2, 52, temperature, 12, 1, YELLOW);
      currentMillis = millis();
    }
    Interval = currentMillis;
    BluetoothData[2] = te;
    flag = 1;
  }
  

  if( updatetime == 0)
  {
    if (currentMillis - startMillis >= 60000) 
    {
      if(d=='0')
      {
        d='1';
        Time();
      }
      else if(d=='1')
      {
        d='2';
        Time();
      }
//      else if(d=='1')
//      {
//        d='2';
//        Time();
//      }
      else if(d=='2')
      {
        d='3';
        Time();
      }
      else if(d=='3')
      {
        d='4';
        Time();
      }
      else if(d=='4')
      {
        d='5';
        Time();
      }
      else if(d=='5')
      {
        d='6';
        Time();
      }
      else if(d=='6')
      {
        d='7';
        Time();
      }
      else if(d=='7')
      {
        d='8';
        Time();
      }
      else if(d=='8')
      {
        d='9';
        Time();
      }
      else if(a=='2' && b=='3' && c=='5' && d=='9')
        {
          A=0; a='0';
          B=0; b='0';
          C=0; c='0';
          D=0; d='0';
          goto r;
        } 
      else if(d=='9')
      {
        d='0';
        C++;
        if(C>5)
        {
          C=0; 
          c='0';
          B++;
          if(B>9)
          {
            B=0; 
            b='0';
            A++;
          }
        }
        Time();
      } 
      Blue();
    }   
  }
}


int Time()
{
      if(C==0){c='0';}
      else if(C==1){c='1';}
      else if(C==2){c='2';}
      else if(C==3){c='3';}
      else if(C==4){c='4';}
      else if(C==5){c='5';}
      if(B==1){b='1';}
      else if(B==2){b='2';}
      else if(B==3){b='3';}
      else if(B==4){b='4';}
      else if(B==5){b='5';}
      else if(B==6){b='6';}
      else if(B==7){b='7';}
      else if(B==8){b='8';}
      else if(B==9){b='9';}
      if(A==0){a='0';}
      else if(A==1){a='1';}
      else if(A==2){a='2';}

      startMillis = currentMillis;
}

int Blue()
{
  bluetooth.print(heart);
  bluetooth.print("|");
  bluetooth.print(temperature);
  //heart[0] = '\0';
  //temperature[0] = '\0';
}

int gettime()
{
    if(count == 1)
    {
      a=bluetooth.read(); 
      A=(int)a;
    }
    
    else if(count == 2)
    {
      b=bluetooth.read(); 
      B=(int)b;
    }
    
    else if(count == 3)
    {
      c=bluetooth.read(); 
      C=(int)c;
    }
    
    else if(count == 4)
    {
      d=bluetooth.read(); 
      D=(int)d; 
      updatetime = 0;
    }
}



