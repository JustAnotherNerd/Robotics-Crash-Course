#include <motor_control.h>
void setup() {
  motor_setup();
  Serial.begin(9600);
}

void loop() {
  int ran = random(10);
  if(ran == 5){
    diff_right(180);
    Serial.println("Turned right");
  }
  else{
    dwell();
  }
  delay(1000);
}
