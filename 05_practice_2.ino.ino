int ledPin = 7;
int blinkCount = 6;

void setup() {
  pinMode(ledPin, OUTPUT);
}

void loop() {

  digitalWrite(ledPin, LOW);
  delay(1000);

  for (int i = 0; i < blinkCount; i++) {
    digitalWrite(ledPin, LOW);
    delay(1000);
    digitalWrite(ledPin, HIGH);
    delay(1000);
  }

  digitalWrite(ledPin, HIGH);

  while (true) {
  }
}
