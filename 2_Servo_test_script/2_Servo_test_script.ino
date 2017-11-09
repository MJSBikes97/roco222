/*### 2-Servo Control Test ###*/
//Servo Library Definitions
#include <Servo.h>
Servo actuator_1;
Servo actuator_2;
#define actuator_1_pin 9
#define actuator_2_pin 10

//Standard Delay Time
int loop_delay_ms = 10;

void setup() {
  actuator_1.attach(actuator_1_pin,700,2000);
  actuator_2.attach(actuator_2_pin,700,2000);

}

void loop() {
  for (int n=0; n<=180; n++) {
    actuator_1.write(n);
    actuator_2.write(180-n);
    delay(loop_delay_ms);
  }
  for(int n=180; n>=0; n--) {
    actuator_1.write(n);
    actuator_2.write(180-n);
    delay(loop_delay_ms);
  }
}
