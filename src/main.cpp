#include "Arduino.h"

#define SYSGOOD 13
#define ENABLE12V 14 
#define ENABLE5VD 15
#define ENABLE3V3D 16
#define STARTCHARGING 17
#define DETECTCHARGING 26
#define SCL 22
#define SDA 21

bool is12Venabled = 0;
bool is5VDenabled = 0;
bool is3V3Denabled = 0;
bool isCharging = 0;

//PCU FUNCTIONS:
void startCharging(bool isChargingDetected);
void enableRouterSpeakerPower();
void enable5VdigitalPower(void);
void enable3V3digitalPower(void);

//main functions
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(SYSGOOD, OUTPUT);
  pinMode(ENABLE12V, OUTPUT);
  pinMode(ENABLE3V3D, OUTPUT);
  pinMode(ENABLE5VD, OUTPUT);
  pinMode(STARTCHARGING, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

  //test codes -> uncomment certain function to use is separately
  enableRouterSpeakerPower();
  enable5VdigitalPower();
  enable3V3digitalPower();
  //startCharging(true);
}

//functions
void startCharging(bool isChargingDetected) {
  if (isChargingDetected) {
    digitalWrite(STARTCHARGING, HIGH);
    digitalWrite(ENABLE5VD, LOW);
    digitalWrite(ENABLE3V3D, LOW);
    Serial.println("charging started");
    delay(5000);
  } else {
    Serial.println("ERROR: CHARGING NOT DETECTED");
  }

  digitalWrite(STARTCHARGING, LOW);
  delay(2000);
}

void enableRouterSpeakerPower() {
  digitalWrite(ENABLE12V, HIGH);
  Serial.println("12V power rail is enabled");
  delay(1000);
  //Serial.println("12V power rail is disabled");
  //digitalWrite(ENABLE12V, LOW);
  //delay(20000);
}

void enable5VdigitalPower() {
  digitalWrite(ENABLE5VD, HIGH);
  Serial.println("5V digital rail is enabled");
  delay(1000);
  //Serial.println("5V digital rail is disabled");
  //digitalWrite(ENABLE5VD, LOW);
  //delay(20000);
}

void enable3V3digitalPower() {
  digitalWrite(ENABLE3V3D, HIGH);
  Serial.println("3V3 digital rail is enabled");
  delay(1000);
  //Serial.println("3V3 digital rail is disabled");
  //digitalWrite(ENABLE3V3D, LOW);
  //delay(20000);
}