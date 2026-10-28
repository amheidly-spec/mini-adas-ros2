// Mini ADAS - Arduino side
// Reads an HC-SR04 ultrasonic sensor and streams distance to the Pi over USB serial.
// Receives the fused warning level from ROS2 and drives an LED + buzzer.
//
// Protocol (115200 baud, newline-terminated ASCII):
//   Arduino -> Pi : D:<distance_cm>   (-1 if no echo)
//   Pi -> Arduino : W:<0|1|2>         0 = SAFE, 1 = CAUTION, 2 = BRAKE
//
// Wiring: HC-SR04 VCC->5V, GND->GND, Trig->D9, Echo->D10
//         LED (220 ohm) -> D5, buzzer -> D6
// The HC-SR04 echo is a 5 V signal, so it lives on the Arduino, not the Pi's 3.3 V GPIO.

const uint8_t TRIG_PIN = 9;
const uint8_t ECHO_PIN = 10;
const uint8_t LED_PIN = 5;
const uint8_t BUZZER_PIN = 6;

const unsigned long SAMPLE_MS = 50;        // 20 Hz
const unsigned long ECHO_TIMEOUT_US = 25000; // ~4.3 m max
const unsigned long LINK_TIMEOUT_MS = 1000; // no command from Pi -> fail-safe CAUTION

uint8_t level = 1;  // start in CAUTION until ROS2 says otherwise
unsigned long lastSample = 0;
unsigned long lastCmd = 0;
char buf[16];
uint8_t bufLen = 0;

float readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long us = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  if (us == 0) return -1.0;
  return us * 0.0343 / 2.0;  // speed of sound ~343 m/s, round trip
}

void readCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      buf[bufLen] = '\0';
      if (bufLen >= 3 && buf[0] == 'W' && buf[1] == ':') {
        int v = buf[2] - '0';
        if (v >= 0 && v <= 2) {
          level = v;
          lastCmd = millis();
        }
      }
      bufLen = 0;
    } else if (bufLen < sizeof(buf) - 1) {
      buf[bufLen++] = c;
    }
  }
}

void driveOutputs(unsigned long now) {
  uint8_t effective = (now - lastCmd > LINK_TIMEOUT_MS) ? 1 : level;
  switch (effective) {
    case 0:  // SAFE: off
      digitalWrite(LED_PIN, LOW);
      noTone(BUZZER_PIN);
      break;
    case 1: {  // CAUTION: slow blink, short beeps
      bool on = (now / 300) % 2;
      digitalWrite(LED_PIN, on);
      if (on) tone(BUZZER_PIN, 1500); else noTone(BUZZER_PIN);
      break;
    }
    case 2:  // BRAKE: solid LED, continuous tone
      digitalWrite(LED_PIN, HIGH);
      tone(BUZZER_PIN, 2500);
      break;
  }
}

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  unsigned long now = millis();
  readCommands();
  if (now - lastSample >= SAMPLE_MS) {
    lastSample = now;
    float cm = readDistanceCm();
    Serial.print("D:");
    Serial.println(cm, 1);
  }
  driveOutputs(now);
}
