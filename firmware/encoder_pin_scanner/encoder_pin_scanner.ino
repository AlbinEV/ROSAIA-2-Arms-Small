#include <Arduino.h>

constexpr uint8_t MOTOR_ENABLE_PIN = 6;
constexpr uint8_t M1_PWM_PIN = 9;
constexpr uint8_t M2_PWM_PIN = 10;

constexpr uint8_t CANDIDATE_PINS[] = {
    2, 3, 11, 12, 13, A2, A3, A4, A5,
};
constexpr uint8_t CANDIDATE_COUNT =
    sizeof(CANDIDATE_PINS) / sizeof(CANDIDATE_PINS[0]);

char inputBuffer[32];
uint8_t inputLength = 0;

void forceMotorOutputsOff() {
  digitalWrite(MOTOR_ENABLE_PIN, LOW);
  analogWrite(M1_PWM_PIN, 0);
  analogWrite(M2_PWM_PIN, 0);
}

void scanPins(uint32_t durationMs) {
  forceMotorOutputsOff();
  uint8_t previous[CANDIDATE_COUNT];
  uint32_t transitions[CANDIDATE_COUNT] = {};
  for (uint8_t index = 0; index < CANDIDATE_COUNT; ++index) {
    previous[index] = static_cast<uint8_t>(digitalRead(CANDIDATE_PINS[index]));
  }

  Serial.print(F("SCAN_BEGIN,"));
  Serial.println(durationMs);
  const uint32_t started = millis();
  while (millis() - started < durationMs) {
    for (uint8_t index = 0; index < CANDIDATE_COUNT; ++index) {
      const uint8_t current =
          static_cast<uint8_t>(digitalRead(CANDIDATE_PINS[index]));
      if (current != previous[index]) {
        ++transitions[index];
        previous[index] = current;
      }
    }
  }

  forceMotorOutputsOff();
  Serial.print(F("SCAN_RESULT,"));
  Serial.print(durationMs);
  for (uint8_t index = 0; index < CANDIDATE_COUNT; ++index) {
    Serial.print(',');
    if (CANDIDATE_PINS[index] >= A0) {
      Serial.print('A');
      Serial.print(CANDIDATE_PINS[index] - A0);
    } else {
      Serial.print('D');
      Serial.print(CANDIDATE_PINS[index]);
    }
    Serial.print(',');
    Serial.print(transitions[index]);
  }
  Serial.println();
}

void handleLine(char* line) {
  char* separator = strchr(line, ',');
  if (separator == nullptr) {
    Serial.println(F("ERR,FORMAT"));
    return;
  }
  *separator = '\0';
  const long duration = strtol(separator + 1, nullptr, 10);
  if (strcmp(line, "SCAN") != 0 || duration < 1000 || duration > 15000) {
    Serial.println(F("ERR,SCAN"));
    return;
  }
  scanPins(static_cast<uint32_t>(duration));
}

void setup() {
  digitalWrite(MOTOR_ENABLE_PIN, LOW);
  digitalWrite(M1_PWM_PIN, LOW);
  digitalWrite(M2_PWM_PIN, LOW);
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  pinMode(M1_PWM_PIN, OUTPUT);
  pinMode(M2_PWM_PIN, OUTPUT);
  forceMotorOutputsOff();

  for (uint8_t index = 0; index < CANDIDATE_COUNT; ++index) {
    pinMode(CANDIDATE_PINS[index], INPUT_PULLUP);
  }
  Serial.begin(115200);
  Serial.println(F("BOOT,ENCODER_PIN_SCANNER,1"));
}

void loop() {
  forceMotorOutputsOff();
  while (Serial.available() > 0) {
    const char value = static_cast<char>(Serial.read());
    if (value == '\n') {
      inputBuffer[inputLength] = '\0';
      if (inputLength > 0) {
        handleLine(inputBuffer);
      }
      inputLength = 0;
    } else if (value != '\r' && inputLength < sizeof(inputBuffer) - 1) {
      inputBuffer[inputLength++] = value;
    }
  }
}
