/*Dynamixel Library*/
#include <DynamixelSoftSerial.h>
#include <SoftwareSerial.h>

/*VCNL4010 Library*/
#include <Wire.h>
#include "Adafruit_VCNL4010.h"

#define TCAADDR 0x70

double start = 0;
double fin= 0;
int countime =0;

/* Assign a unique ID to this sensor at the same time */
Adafruit_VCNL4010 vcnl_6;
Adafruit_VCNL4010 vcnl_7;

/*Read Serial */
char rxChar;
double bps=1000000;
double ax_bps=9600;


/*Dynamixel Library*/
int k=0;
int grip_open[2]={450,574};
int grip_close[2]={512,512};
int comodin0[2]={522,502};
int comodin1[2]={532,492};
int comodin2[2]={542,482};
int comodin3[2]={552,472};
int comodin4[2]={562,462};
int comodin5[2]={572,452};

/*VCNL4010 Library*/
double data[4][2];
int i=0;
int j=0; 

/*Tymesync*/
long startTime ;                    // start time for stop watch
long elapsedTime ;                  // elapsed time for stop watch
unsigned long timer = 0;
long loopTime =1000;   // microseconds

/*Dynamixel Status*/
int Temperature,Voltage_left,Voltage_right,Position,Position2,Load_Left,Load_Right,counter; 

/*Smoothing*/
const int numReadings = 10;

int readings_left[numReadings];      // the readings from the analog input
int readings_right[numReadings];      // the readings from the analog input

int readIndex = 0;              // the index of the current reading
int total_left = 0;                  // the running total
int total_right = 0;
int average = 0;                // the average

int inputPin = A0;

// Global Variables
String g_recvString = "";
int g_scaleFactor = 1;
int new_command = 0;
int steps_counter = 0;

/*Continous sensing*/
boolean flag_slipping_detector=false;
int slipping_range=20;
double left_slipping_value=0;
double right_slipping_value=0;
float right_fit[4]={-223.0426713132407, 3.16599816, -1506.154955, 249944.380};
float left_fit[4]={-285.42, 4.0496748, -1919.41123, 314927.525};
float left[2] = {0,0};
float right[2] = {0,0};
int slipping_counter=0;
int actual_value[2]={512,512};
int slipping_sensitivity=1;
int que_pasa=0;

/*Fusion*/
float variances[2][2] = {{9.9242,94.53},{18.23,162.88}};


/*Useful Functions*/
/*--------------------------------------------------------------------------------------Dynamixel Library*/
void gripper_action(int desired_action[])
{
  for(k=1;k<=2;k++)
  {
    Dynamixel.moveRW(k,desired_action[k-1]);
  }
  Dynamixel.action();
  k=0;
  actual_value[0]=desired_action[0];
  actual_value[1]=desired_action[1];
}

void gripper_action_increasing(int desired_action[])
{
  Serial.end();
  Dynamixel.begin(ax_bps,2);
  for(k=1;k<=2;k++)
  {
    Dynamixel.moveRW(k,desired_action[k-1]);
  }
  Dynamixel.action();
  k=0;
  actual_value[0]=desired_action[0];
  actual_value[1]=desired_action[1];
  Dynamixel.end();
  Serial.begin(bps);
}

void activate_slipage_detector()
{ 
  double valor = 1500;
  int continous[2]={512,512};
  int counter=1;
  Serial.print("Slippage detector activated");
  delay(1000);
  flag_slipping_detector=!flag_slipping_detector;
  tcaselect(2);
   left_slipping_value = vcnl_6.readProximity();
  tcaselect(7);
   right_slipping_value = vcnl_7.readProximity();
   que_pasa=0;
}


void gripper_action_threshold()
{ 
  double bend_l,bend_r = 1500;
  int continous[2]={574,450};//{512,512};
  int counter=1;
  int flag=0;
  int desired_iterations=80;
  Load_Left=1000;
  Load_Right=0;
  
  do {
    Serial.end();                     // End the Serial Comunication
    Dynamixel.begin(ax_bps,2);         // Begin Servo Comunication
    counter=counter+1;
    if (counter>3)
      {
      Load_Left=Dynamixel.readLoad(1);
      Load_Right=Dynamixel.readLoad(2);
      if (Load_Right>=1030 and Load_Right<=1050){Load_Right=40;}//Small selective filter to avoid bad measurements due to normal applied force while closing false positives
      }
       actual_value[0]=actual_value[0]+1;
       actual_value[1]=actual_value[1]-1;
       Dynamixel.moveRW(1,actual_value[0]);
       Dynamixel.moveRW(2,actual_value[1]);
       Dynamixel.action();
//       delay(5);
//    if (bend_l<=1100 && counter>3){flag=1;break;}
    if (Load_Left>=1120 && counter>3){flag=2;break;}
    else if (Load_Right>=200 && counter>3){flag=3;break;}
    
  Dynamixel.end();                 // End Servo Comunication
  Serial.begin(bps);              // Begin Serial Comunication
  
  Serial.print(flag);
  Serial.print(" ");
  Serial.print(Load_Left);
  Serial.print(" ");
  Serial.print(Load_Right);
  Serial.print(" ");
  Serial.print(counter);
  Serial.println(" ");
//  Serial.end();                     // End the Serial Comunication
  }
   while (counter<desired_iterations and Load_Left>=-2 and Load_Right>=-2 ); //&& Load_Left>=999 && Load_Right>=0
   Dynamixel.end();                 // End Servo Comunication
  Serial.begin(bps);              // Begin Serial Comunication
  
   Serial.print("Sali a gripper_action_threshold");
   Serial.print(flag);
  Serial.print(" ");
  Serial.print(Load_Left);
  Serial.print(" ");
  Serial.print(Load_Right);
  Serial.print(" ");
  Serial.print(counter);
  Serial.println(" ");
}

/*------------------------------------------------------------- VCNL4010*/
void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

void initialize_TCA9548A()
{Serial.println("Sensor Inicia");
  /* Initialise the 1st sensor */
  tcaselect(2);
  if (!vcnl_6.begin()){
    Serial.println("Sensor 1 not found :(");
    while (1);
  }
  else
  {Serial.println("Sensor 1  found :)");}
  
  
  /* Initialise the 2nd sensor */
  tcaselect(7);
  if (!vcnl_7.begin()){
    Serial.println("Sensor 2 not found :(");
    while (1);
  }
  else
  {Serial.println("Sensor 2  found :)");}
 }
 
/*------------------------------------------------------------- Tymesync*/
void timeSync(unsigned long deltaT)
{
  unsigned long currTime = micros();
  long timeToDelay = deltaT - (currTime - timer);
  if (timeToDelay > 5000)
  {
    delay(timeToDelay / 1000);
    delayMicroseconds(timeToDelay % 1000);
  }
  else if (timeToDelay > 0)
  {
    delayMicroseconds(timeToDelay);
  }
  else
  {
      // timeToDelay is negative so we start immediately
  }
  timer = currTime + timeToDelay;
}


float fitting(double val, float desired_action[])
{return (desired_action[0]/100000.*pow(val,3))+(desired_action[1]*pow(val,2))+(desired_action[2]*float(val))+(desired_action[3]);}

float fuse_data(float scalled, double proximity, float varianc[])
  {return ((scalled*1/varianc[0])+(proximity*1/varianc[1]))/((1/varianc[0])+(1/varianc[1]));}

void setup(){

  SoftwareSerial softSerial(4,3);
  Dynamixel.setSerial(&softSerial);
  Dynamixel.begin(ax_bps,2);
  gripper_action(grip_close);
  delay(1000);
  gripper_action(grip_open);
  delay(1000);
  gripper_action(grip_close);
  delay(1000);
  Serial.begin(bps);              // Begin Serial Comunication after read data
  Wire.begin();
  initialize_TCA9548A();
  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH);  
  delay(1000);
//  Serial.print("Initializing");
  Serial.println("Flex Proximity Fused");
}

void loop()
{ 
//  timeSync(loopTime);
//start = micros();
  if(Serial.available() > 0)
  {
    rxChar = Serial.read();
    if(rxChar == '1'){
      digitalWrite(13,HIGH); gripper_action(grip_close);}
    else if(rxChar == '2'){gripper_action(grip_open);}
    else if(rxChar == '3'){gripper_action(comodin0);}
    else if(rxChar == '4'){gripper_action(comodin1);}
    else if(rxChar == '5'){gripper_action(comodin2);}
    else if(rxChar == '6'){gripper_action(comodin3);}
    else if(rxChar == '7'){gripper_action(comodin4);}
    else if(rxChar == '8'){gripper_action_threshold();}
    else if(rxChar == '9'){activate_slipage_detector();}
  }
  else
  {steps_counter=steps_counter+1;
  Voltage_left=analogRead(A6);Voltage_left=analogRead(A6);
  Voltage_right=analogRead(A7);Voltage_right=analogRead(A7);
  tcaselect(2);
  double Distance_Left = vcnl_6.readProximity();
  tcaselect(7);
  double Distance_Right = vcnl_7.readProximity(); /*Lets be careful with trying to read from distance measurement in case of that specific sensor was not found*/
    if (steps_counter>=10 )
    {
      Serial.end();                     // End the Serial Comunication just for read data
      Dynamixel.begin(ax_bps,2);         // Begin Servo Comunication
      Load_Left = Dynamixel.readLoad(1);
      Load_Right = Dynamixel.readLoad(2);
      Dynamixel.end();                 // End Servo Comunication
      Serial.begin(bps);              // Begin Serial Comunication after read data
      left[0]=fitting(Voltage_left,left_fit);
      right[0]=fitting(Voltage_right,right_fit);
      float fused_data_left = fuse_data(left[0],Distance_Left,variances[0]);
      float fused_data_right = fuse_data(right[0],Distance_Right,variances[1]);
      Serial.print(Load_Left);Serial.print(" ");
      Serial.print(Voltage_left);Serial.print(" ");
//      Serial.print(left[0]);Serial.print(" ");//Remove this line after testing
      Serial.print(Distance_Left);Serial.print(" ");
      Serial.print(Load_Right);Serial.print(" ");
      Serial.print(Voltage_right);Serial.print(" ");
      Serial.println(Distance_Right);
//      Serial.print(fused_data_left);;Serial.print(" ");
//      Serial.print(fused_data_right);Serial.println(" ");
        steps_counter=0;
    }
   
      if (flag_slipping_detector){
          if (que_pasa<=2){
          Serial.println("Simple slippage detector --------------------");
          left[0]=fitting(Voltage_left+slipping_sensitivity,left_fit);
          left[1]=fitting(Voltage_left-slipping_sensitivity,left_fit);
          right[0]=fitting(Voltage_right+slipping_sensitivity,right_fit);
          right[1]=fitting(Voltage_right-slipping_sensitivity,right_fit);
//           Serial.print(slipping_sensitivity);
//           Serial.print(left[0],6);Serial.print(" ");
//           Serial.print(left[1],6);Serial.print(" ");
//           Serial.print(right[0],6);Serial.print(" ");
//           Serial.println(right[1],6);
           que_pasa=que_pasa+1;}
          if ((Distance_Left<=left[0] or Distance_Right<=right[0] ) and slipping_counter<10)   // Uncomment after load times measuring
          {
           Serial.end();                     // End the Serial Comunication
           Dynamixel.begin(ax_bps,2);         // Begin Servo Comunication
           actual_value[0]=actual_value[0]+2;
           actual_value[1]=actual_value[1]-2;
           Dynamixel.moveRW(1,actual_value[0]);
           Dynamixel.moveRW(2,actual_value[1]);
           Dynamixel.action();
           left_slipping_value=vcnl_6.readProximity();
           right_slipping_value=vcnl_7.readProximity();
           slipping_counter=slipping_counter+1;
           Dynamixel.end();                 // End Servo Comunication
           Serial.begin(bps);              // Begin Serial Comunication
//           Serial.println("Modificado");
             if (slipping_counter==9){
              flag_slipping_detector=false;
              slipping_counter=0;
              }
           }
        }
  }//end else
}
