
//
///*Dynamixel Status*/
//int Temperature,Voltage_left,Voltage_right,Position,Position2,Load_Left,Load_Right,counter; 
//
//
//void gripper_action(int desired_action[])
//{
//  for(k=1;k<=2;k++)
//  {
//    Dynamixel.moveRW(k,desired_action[k-1]);
//  }
//  Dynamixel.action();
//  k=0;
//  actual_value[0]=desired_action[0];
//  actual_value[1]=desired_action[1];
//}
//
/*Scannign*/
///*******************************************************************************
//* Copyright 2016 ROBOTIS CO., LTD.
//*
//* Licensed under the Apache License, Version 2.0 (the "License");
//* you may not use this file except in compliance with the License.
//* You may obtain a copy of the License at
//*
//*     http://www.apache.org/licenses/LICENSE-2.0
//*
//* Unless required by applicable law or agreed to in writing, software
//* distributed under the License is distributed on an "AS IS" BASIS,
//* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//* See the License for the specific language governing permissions and
//* limitations under the License.
//*******************************************************************************/
////
//#include <Dynamixel2Arduino.h>
//
//  #define DXL_SERIAL   Serial1
//  #define DEBUG_SERIAL Serial
//  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
//
//#define MAX_BAUD  6
//const int32_t buad[MAX_BAUD] = {57600, 115200, 1000000, 2000000,3000000, 9600};
//
//Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
//
//void setup() {
//  // put your setup code here, to run once:
//  int8_t index = 0;
//  int8_t found_dynamixel = 0;
//
//  // Use UART port of DYNAMIXEL Shield to debug.
//  DEBUG_SERIAL.begin(115200);   //set debugging port baudrate to 115200bps
//  while(!DEBUG_SERIAL);         //Wait until the serial port is opened
//    
//  for(int8_t protocol = 1; protocol < 3; protocol++) {
//    // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
//    dxl.setPortProtocolVersion((float)protocol);
//    DEBUG_SERIAL.print("SCAN PROTOCOL ");
//    DEBUG_SERIAL.println(protocol);
//    
//    for(index = 0; index < MAX_BAUD; index++) {
//      // Set Port baudrate.
//      DEBUG_SERIAL.print("SCAN BAUDRATE ");
//      DEBUG_SERIAL.println(buad[index]);
//      dxl.begin(buad[index]);
//      for(int id = 0; id < DXL_BROADCAST_ID; id++) {
//        //iterate until all ID in each buadrate is scanned.
//        if(dxl.ping(id)) {
//          DEBUG_SERIAL.print("ID : ");
//          DEBUG_SERIAL.print(id);
//          DEBUG_SERIAL.print(", Model Number: ");
//          DEBUG_SERIAL.println(dxl.getModelNumber(id));
//          found_dynamixel++;
//        }
//      }
//    }
//  }
//  
//  DEBUG_SERIAL.print("Total ");
//  DEBUG_SERIAL.print(found_dynamixel);
//  DEBUG_SERIAL.println(" DYNAMIXEL(s) found!");
//}
//
//void loop() {
//  // put your main code here, to run repeatedly:
//}


#include <Dynamixel2Arduino.h>

/*VCNL4010 Library*/
#include <Wire1.h>
//Wire1.setSCL(16);
//Wire1.setSDA(17);
#include "Adafruit_VCNL4010.h"
#include "Adafruit_VCNL4010_w1.h"

#define TCAADDR 0x70
///* Assign a unique ID to this sensor at the same time */
Adafruit_VCNL4010 vcnl;
Adafruit_VCNL4010_w1 vcnl_w1;


  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL Serial
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
  const uint8_t DXL_IDL = 1;
  const uint8_t DXL_IDR = 2;
  const float DXL_PROTOCOL_VERSION = 1.0;

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
double ax_bps=1000000;
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

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

int val =0;

void set_up_dynamixel()
{
  // Set Port baudrate to 9600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(ax_bps);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(DXL_IDL);

  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(DXL_IDL);
  dxl.setOperatingMode(DXL_IDL, OP_POSITION);
  dxl.torqueOn(DXL_IDL);
  }
  
void gripper_action(int desired_action[])
{
  for(k=1;k<=2;k++)
  {
    dxl.setGoalPosition(k, desired_action[k-1]);
  }
//  Dynamixel.action();
  k=0;
  actual_value[0]=desired_action[0];
  actual_value[1]=desired_action[1];
}


void setup() {
  DEBUG_SERIAL.begin(1000000);
//  DEBUG_SERIAL.println("VCNL4010 test");
//
//  if (! vcnl.begin()){
//    DEBUG_SERIAL.println("Sensor not found :(");
//    while (1);
//  }
//  DEBUG_SERIAL.println("Found VCNL4010");
//
// Wire1.begin();
//  Wire1.setSCL(16);
//  Wire1.setSDA(17);
//  if (! vcnl_w1.begin()){
//    DEBUG_SERIAL.println("Sensor not found :(");
//    while (1);
//  }
//  DEBUG_SERIAL.println("Found VCNL4010");
//  analogReadRes(12);          // set ADC resolution to this many bits
//  analogReadAveraging(1);    // average this many readings
  set_up_dynamixel();
}


void loop() {
  byte resultado =0;
  gripper_action(grip_close);
//   Print present position in degree value
  DEBUG_SERIAL.print("Present Position(degree) : ");
  DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_IDL, UNIT_DEGREE));
  delay(1000);
  gripper_action(grip_open);
//  val = analogRead(9);
//  DEBUG_SERIAL.print("Voltage_Left: "); DEBUG_SERIAL.println(analogRead(8));
//  DEBUG_SERIAL.print("Voltage_Left: "); DEBUG_SERIAL.println(analogRead(9));
//   DEBUG_SERIAL.print("Ambient: "); DEBUG_SERIAL.println(vcnl.readAmbient());
//   DEBUG_SERIAL.print("  Proximity: "); DEBUG_SERIAL.println(vcnl.readProximity());
//   DEBUG_SERIAL.println(" --------------  ");
////   DEBUG_SERIAL.print("Voltage_Right: "); DEBUG_SERIAL.println(analogRead(1));
//   DEBUG_SERIAL.print("Ambient: "); DEBUG_SERIAL.println(vcnl_w1.readAmbient());
//   DEBUG_SERIAL.print("  Proximity: "); DEBUG_SERIAL.println(vcnl_w1.readProximity());
   delay(500);
}
