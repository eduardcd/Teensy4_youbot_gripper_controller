#include <Wire1.h>
//Wire1.setSCL(16);
//Wire1.setSDA(17);
#include "Adafruit_VCNL4010.h"
#include "Adafruit_VCNL4010_w1.h"

Adafruit_VCNL4010 vcnl;
Adafruit_VCNL4010_w1 vcnl_w1;
void setup() {
  Serial.begin(9600);
  Serial.println("VCNL4010 test");

  if (! vcnl.begin()){
    Serial.println("Sensor not found :(");
    while (1);
  }
  Serial.println("Found VCNL4010");

 Wire1.begin();
  Wire1.setSCL(16);
  Wire1.setSDA(17);
  if (! vcnl_w1.begin()){
    Serial.println("Sensor not found :(");
    while (1);
  }
  Serial.println("Found VCNL4010");
}


void loop() {
   Serial.print("Ambient: "); Serial.print(vcnl.readAmbient());
   Serial.print("  Proximity: "); Serial.println(vcnl.readProximity());
   Serial.println(" --------------  ");
   Serial.print("Ambient: "); Serial.print(vcnl_w1.readAmbient());
   Serial.print("  Proximity: "); Serial.println(vcnl_w1.readProximity());
   delay(500);
}
