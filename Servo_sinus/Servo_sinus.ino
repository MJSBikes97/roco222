/*### Sinusoidal Servo Movement ###*/
#include <Servo.h>
#define servo_pin 5

Servo servo1;
unsigned int sin_table[157];
float sin_input = 0.0f;

void setup() {
  Serial.begin(9600);
  //Generate Sin table of angle values 0-180 (1.57rad)
  for (int n = 0; n<=156; n++) {
    sin_table[n] = (180u*sin(sin_input));
    sin_input = sin_input+0.01f; //Increment radians
  }
 servo1.attach(servo_pin); //enable servo
}

void loop() {
  //Write 0 to
  for (int n = 0; n<157; n++) {
    servo1.write(sin_table[n]);
    Serial.println(sin_table[n]);
    delay(5);
  }
  for (int n = 0; n<157; n++) {
    servo1.write(180-sin_table[n]);
    Serial.println(180-sin_table[n]);
    delay(5);
  }
}
