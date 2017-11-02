/*### Stepper Motor Control w/Modes ###*/

/*## Definitions ##*/

#define DIR_A 12
#define DIR_B 13
#define PWM_A 3
#define PWM_B 11

#define Fwd HIGH
#define Rev LOW

/*## Step Delay ##*/

const int stepDelay_ms = 100;

void setup() {
/*## PinMode Config ##*/
pinMode(DIR_A, OUTPUT);
pinMode(DIR_B, OUTPUT);
// Turn off Braking for both Channels
pinMode(8, OUTPUT); digitalWrite(8, LOW);
pinMode(9, OUTPUT); digitalWrite(9, LOW);

}

void loop() {

doubStep();

}
/*## Directional Control Function ##*/
void setDirection(int A, int B) {
  digitalWrite(DIR_A, A);
  digitalWrite(DIR_B, B);

}
/*## CH A Control Functions ##*/
void ch_A_ON() {
  analogWrite(PWM_A, 255);
}

void ch_A_OFF() {
  analogWrite(PWM_A, 0);
}
/*## CH B Control Functions ##*/
void ch_B_ON() {
  analogWrite(PWM_B, 255);
}

void ch_B_OFF() {
  analogWrite(PWM_B, 0);
}
/*## Mode Functions ##*/
void fullStep() {
    setDirection(Fwd, Fwd);
  ch_A_ON();
  ch_B_OFF();
  delay(stepDelay_ms);

  ch_A_OFF();
  ch_B_ON();
  delay(stepDelay_ms);

  setDirection(Rev, Rev);
  ch_A_ON();
  ch_B_OFF();
  delay(stepDelay_ms);

  ch_A_OFF();
  ch_B_ON();
  delay(stepDelay_ms);
}

void doubStep() {
  setDirection(Fwd,Fwd);
  ch_A_ON();
  ch_B_ON();
  delay(stepDelay_ms);

  setDirection(Rev, Fwd);
  delay(stepDelay_ms);

  setDirection(Rev,Rev);
  delay(stepDelay_ms);

  setDirection(Fwd, Rev);
  delay(stepDelay_ms);
}

void halfStep() {
  //1
  setDirection(Fwd, Fwd);
  ch_A_ON();
  ch_B_OFF();
  delay(stepDelay_ms);
  //2
  ch_A_ON();
  ch_B_ON();
  delay(stepDelay_ms);
  //3
  ch_A_OFF();
  ch_B_ON();
  delay(stepDelay_ms);
  //4
  setDirection(Rev, Fwd);
  ch_A_ON();
  ch_B_ON();
  delay(stepDelay_ms);
  //5
  ch_A_ON();
  ch_B_OFF();
  delay(stepDelay_ms);
  //6
  setDirection(Rev, Rev);
  ch_A_ON();
  ch_B_ON();
  delay(stepDelay_ms);
  //7
  ch_A_OFF();
  ch_B_ON();
  delay(stepDelay_ms);
  //8
  setDirection(Fwd, Rev);
  ch_A_ON();
  ch_A_OFF();
}

  



