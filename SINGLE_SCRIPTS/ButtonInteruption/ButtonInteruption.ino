const byte ledPin = 13;
const byte interruptPin = 3;
volatile byte state = HIGH;

void setup() {
    Serial.begin(9600);

  //pinMode(ledPin, OUTPUT);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), blink1, RISING );
      Serial.println("start");

}

void loop() {
  //digitalWrite(ledPin, state);
  if (state == HIGH) {
    Serial.println("pressed");
    state = LOW;
  } else {
    Serial.println("NOT pressed");
  }
    delay(5000);

}

void blink1() {
       Serial.println("interupt");

   Serial.println(state);
  state = !state;
}
