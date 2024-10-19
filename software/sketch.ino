#include "SatNav.h"
#include "StageManager.h"
#include "VectorMath.h"

#define PIN_TRIG 12
#define PIN_ECHO 11
#define PIN_LAUNCH 10
#define PIN_IGNITION 9
#define DEBUG_ULTRASONIC_NO_OUTPUT_FOUND_LOW 390
#define DEBUG_ULTRASONIC_NO_OUTPUT_FOUND_HI 410
//#define USE_ALTERNATIVE_ACCURATE_SEEKING true

// drag
const float ROCKET_CROSS_SECTIONAL_AREA_M = 0.2;

Stage stage = StageManager().stage;
SatNav satNav = SatNav(stage);
VectorMath vectorMath = VectorMath(satNav, ROCKET_CROSS_SECTIONAL_AREA_M);

bool ignitionTrigger = false;
bool launched = false;

int getUltrasonicDistance() {
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  int duration = pulseIn(PIN_ECHO, HIGH);
  return duration / 58;
}

void launchSequence() {
  Serial.println(F("[LAUNCH] Starting countdown"));
  Serial.println(F("[LAUNCH] Launch in 10"));
  delay(1000);
  // on each print statement, broadcast on LoRa frequency
  Serial.println(F("[LAUNCH] Launch in 9"));
  delay(1000);
  Serial.println(F("[LAUNCH] Launch in 8"));
  delay(1000);
  Serial.println(F("[LAUNCH] Launch in 7"));
  delay(1000);
  Serial.println(F("[LAUNCH] Launch in 6"));
  delay(1000);
  Serial.println(F("[LAUNCH] Launch in 5"));
  delay(1000);
  digitalWrite(PIN_IGNITION, HIGH);
  Serial.println(F("[LAUNCH] Launch in 4"));
  delay(1000);
  Serial.println(F("[LAUNCH] Launch in 3"));
  delay(1000);
  Serial.println(F("[LAUNCH] Launch in 2"));
  delay(1000);
  Serial.println(F("[LAUNCH] Launch in 1"));
  delay(1000);
  
}

void setup() {
  Serial.begin(74800, SERIAL_8N1);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_LAUNCH, INPUT_PULLUP);
  pinMode(PIN_IGNITION, OUTPUT);
  stage = Stage::WAIT;
  Serial.println(F("Emerald I onboard computer initialized!"));

  float x, z;
  unsigned long millis_start = millis();
  #if USE_ALTERNATIVE_ACCURATE_SEEKING
  vectorMath.alternative_seeking(x, z);
  #else
  vectorMath.original_optimize(x, z);
  #endif
  Serial.println("[LAUNCH HELPER] Optimal launch angle found in " + String((millis() - millis_start) / 1000.0) + "s! X: " + String(x) + ", Z: " + String(z));
}

void loop() {
  int buttonValue = digitalRead(PIN_LAUNCH);

  if(buttonValue != HIGH) {
    ignitionTrigger = true;
  }

  if(ignitionTrigger && !launched) {
    stage = Stage::IGNITION;
    ignitionTrigger = false;
    int dist = getUltrasonicDistance();
    if(dist > DEBUG_ULTRASONIC_NO_OUTPUT_FOUND_LOW && dist < DEBUG_ULTRASONIC_NO_OUTPUT_FOUND_HI) {
      Serial.println(F("[LAUNCH HELPER] Rocket successfully changed to IGNITION stage!"));
      stage = Stage::IGNITION;
      launched = true;
      launchSequence();
    } else {
      Serial.println("[LAUNCH HELPER] Launch is unsafe! Rocket is pointing at an object " + String(dist) + "cm away");
    }
  }
  delay(50);
}
