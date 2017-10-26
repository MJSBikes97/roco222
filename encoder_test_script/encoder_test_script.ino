const byte ledPin = 13;
const byte interruptPin = 2;
volatile byte state = LOW;

void setup() {
pinMode(ledPin, OUTPUT);
pinMode(interruptPin, INPUT);

/*### config interrupt callback for every lo-hi transition ###*/
attachInterrupt(digitalPinToInterrupt(interruptPin), blink,
RISING);
}

void loop() {
digitalWrite(ledPin, state);

}

void blink() {
  state = !state;
}

