#define PIN_LED 7
int count = 0;

void setup() {
  pinMode(PIN_LED, OUTPUT);
}

void loop() {
  while (1) {
    if (count == 0) {
      count++;
      digitalWrite(PIN_LED, 0);
      delay(1000);
    }
    else if (count < 5) {
      count++;
      digitalWrite(PIN_LED, 0);
      delay(100);
      digitalWrite(PIN_LED, 1);
      delay(100);
    }
    else {
      digitalWrite(PIN_LED, 1);
    }
  }
}

