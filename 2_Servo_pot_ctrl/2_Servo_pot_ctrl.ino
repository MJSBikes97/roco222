/*### Potentiometer 2-Servo Control ###*/
//Servo Library Definitions
#include <Servo.h>
Servo actuator_1;
Servo actuator_2;
#define actuator_1_pin 9
#define actuator_2_pin 10

//Analog In Pin Definitions
#define act_1_ctrl A0
#define act_2_ctrl A1
//Standard Delay Length
int loop_delay_ms = 1;
//Serial Definitions
#define PC_baud 9600

/*## Setup ## */
void setup() {
//Attach Servos
actuator_1.attach(actuator_1_pin,700,2000);
actuator_2.attach(actuator_2_pin,700,2000);
Serial.begin(PC_baud);
}
/*## Main Loop ##*/
void loop() {
  actuator_1.write(map(analogRead(act_1_ctrl), 0, 1023, 0, 180));
  Serial.println(analogRead(act_1_ctrl));
  actuator_2.write(map(analogRead(act_2_ctrl), 0, 1023, 0, 180));
  Serial.println(analogRead(act_2_ctrl));
  delay(loop_delay_ms);

}
