#include <HC_SR04.h>

#define ECHO_PIN 2
#define TRIG_PIN 10
#define ECHO_INT 0

HC_SR04 distance_sensor(TRIG_PIN, ECHO_PIN, ECHO_INT);

void setup() {
  Serial.begin(9600);

  distance_sensor.begin();
  distance_sensor.start();
}

void loop() {
  
  if(distance_sensor.isFinished()){
    Serial.println(distance_sensor.getRange());
    distance_sensor.start();
  }
}
