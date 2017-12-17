/*## Definitions ##*/

#define DIR_A 12
#define DIR_B 13
#define PWM_A 10
#define PWM_B 11
#define ZERO_SWITCH 5
#define ACT_CTRL A8

#define Fwd HIGH
#define Rev LOW
int usteps = 200;
float amp = 127;
int pulseDelay_us = 40;
int a[200];
int b[200];
int idxG = 0;
int steps;
/*## Prototypes ##*/
void writeMotor(int value, int PWM_PIN, int DIR_PIN);
void zeroMotor(void);
void runMotor(int dir);

void setup() {
/*## Sin/Cos Tables ##*/
for (int i=0; i<usteps; i++) {
  a[i] = amp*sin(i*2*PI/usteps);
  b[i] = amp*cos(i*2*PI/usteps);
}
Serial.begin(9600);
/*## PinMode Config ##*/
pinMode(DIR_A, OUTPUT);
pinMode(DIR_B, OUTPUT);
pinMode(ZERO_SWITCH, INPUT);
// Turn off Braking for both Channels
pinMode(8, OUTPUT); digitalWrite(8, LOW);
pinMode(9, OUTPUT); digitalWrite(9, LOW);

/*## Zero Stepper ##*/
zeroMotor();
//delay(2000);
}

void loop() {
  delay(1000);
  int val = analogRead(ACT_CTRL);
  //Serial.println(val);
  int target_step = map(val, 0, 1023, 0, 200);
  //int diff = abs(target_step - steps);
  //Serial.println(diff);
  //Serial.println(target_step);
  if (target_step > steps) {
    while (steps != target_step) {
      runMotor(Fwd);
    }
  } else {
    while (steps != target_step) {
      runMotor(Rev);
    }
  }
  delay(100);
  analogWrite(PWM_A, 0);
  analogWrite(PWM_B, 0);
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
void zeroMotor(void) {
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
  steps = 0;
}
void runMotor(int dir) {
  if (dir == Fwd) {
    writeMotor(a[idxG], PWM_A, DIR_A);
    writeMotor(b[idxG], PWM_B, DIR_B);
  
    idxG++;
    if (idxG == usteps) {
      idxG = 0;
      steps++;
    }
    delayMicroseconds(pulseDelay_us);
  } else {
    writeMotor(a[idxG], PWM_A, DIR_A);
    writeMotor(b[idxG], PWM_B, DIR_B);
  
    idxG--;
    if (idxG == 0) {
      idxG = usteps;
      steps--;
    }
    delayMicroseconds(pulseDelay_us);
  }
}

