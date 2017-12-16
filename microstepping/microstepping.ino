/*## Definitions ##*/

#define DIR_A 12
#define DIR_B 13
#define PWM_A 10
#define PWM_B 11
#define ZERO_SWITCH 5

#define Fwd HIGH
#define Rev LOW
int usteps = 360;
float amp = 127;
int pulseDelay_us = 40;
int a[360];
int b[360];
int idxG = 0;

void writeMotor(int value, int PWM_PIN, int DIR_PIN);
/*## Step Delay ##*/

//const int stepDelay_ms = 50;

void setup() {
/*## Sin/Cos Tables ##*/
for (int i=0; i<usteps; i++) {
  a[i] = amp*sin(i*2*PI/usteps);
  b[i] = amp*cos(i*2*PI/usteps);
}
/*## PinMode Config ##*/
pinMode(DIR_A, OUTPUT);
pinMode(DIR_B, OUTPUT);
pinMode(ZERO_SWITCH, INPUT);
// Turn off Braking for both Channels
pinMode(8, OUTPUT); digitalWrite(8, LOW);
pinMode(9, OUTPUT); digitalWrite(9, LOW);

}

void loop() {
  while (digitalRead(ZERO_SWITCH) == HIGH) {
    writeMotor(a[idxG], PWM_A, DIR_A);
    writeMotor(b[idxG], PWM_B, DIR_B);
  
    idxG++;
    if (idxG == usteps) {
      idxG = 0;
    }
    delayMicroseconds(pulseDelay_us);
  }
  delay(100);
  analogWrite(PWM_A, 0);
  analogWrite(PWM_B, 0);
  delay(2000);
}

void writeMotor(int value, int PWM_PIN, int DIR_PIN) {
  int absPWM = abs(value);
  analogWrite(PWM_PIN, absPWM);
  if (value > 0) {
    digitalWrite(DIR_PIN, Fwd);
  } else {
    digitalWrite(DIR_PIN, Rev);
  }
}


