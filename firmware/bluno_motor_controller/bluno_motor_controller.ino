#include <Arduino.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>

// Makerfabs H-Bridge Motor Shield v1.0.
constexpr uint8_t MOTOR_ENABLE_PIN = 6;
constexpr uint8_t M1_PWM_PIN = 9;
constexpr uint8_t M1_DIR1_PIN = 4;
constexpr uint8_t M1_DIR2_PIN = 5;
constexpr uint8_t M2_PWM_PIN = 10;
constexpr uint8_t M2_DIR1_PIN = 7;
constexpr uint8_t M2_DIR2_PIN = 8;

// Physical motor 1 feedback (axis 0 -> shield M2).
constexpr uint8_t AXIS0_ENCODER_A_PIN = 2;
constexpr uint8_t AXIS0_ENCODER_B_PIN = 3;
constexpr uint8_t AXIS0_CURRENT_PIN = A0;

// Physical motor 2 feedback (axis 1 -> shield M1). D11/D12 are the
// ATmega328P PCINT3/PCINT4 inputs; only channel A generates interrupts.
constexpr uint8_t AXIS1_ENCODER_A_PIN = 11;
constexpr uint8_t AXIS1_ENCODER_B_PIN = 12;
constexpr uint8_t AXIS1_CURRENT_PIN = A1;

// Bench-test cap. The ROS bridge applies the same bound independently.
constexpr int16_t MAX_ABS_COMMAND = 255;
constexpr int16_t MAX_RELATIVE_STEP_COUNTS = 50;
constexpr int8_t AXIS0_ENCODER_SIGN = -1;
constexpr int8_t AXIS1_ENCODER_SIGN = -1;
constexpr int8_t AXIS0_COMMAND_TO_OPERATIONAL_SIGN = 1;
constexpr int8_t AXIS1_COMMAND_TO_OPERATIONAL_SIGN = -1;
constexpr int32_t AXIS0_OPERATIONAL_MIN_COUNT = 0;
constexpr int32_t AXIS0_OPERATIONAL_MAX_COUNT = 300;
constexpr int32_t AXIS1_OPERATIONAL_MIN_COUNT = 0;
constexpr int32_t AXIS1_OPERATIONAL_MAX_COUNT = 300;
constexpr uint32_t COMMAND_TIMEOUT_MS = 300;
constexpr uint32_t RELATIVE_STEP_TIMEOUT_MS = 3000;
constexpr uint32_t TELEMETRY_PERIOD_MS = 20;
constexpr uint32_t CURRENT_SAMPLE_PERIOD_US = 2500;
constexpr uint16_t CURRENT_CALIBRATION_SAMPLES = 256;
constexpr int32_t CURRENT_UNAVAILABLE_MA = INT32_MIN;

enum FaultFlag : uint16_t {
  FLAG_COMMAND_TIMEOUT = 1U << 0,
  FLAG_MALFORMED_COMMAND = 1U << 1,
  // Bit 2 was AXIS1_REJECTED in firmware v1; retained as a reserved bit so
  // the remaining protocol flag values do not change.
  FLAG_RESERVED_BIT_2 = 1U << 2,
  FLAG_MOTION_LOCKED = 1U << 3,
  FLAG_CURRENT_SCALE_UNKNOWN = 1U << 4,
  FLAG_RX_OVERFLOW = 1U << 5,
  FLAG_POSITION_LIMIT = 1U << 6,
};

volatile int32_t axis0EncoderCount = 0;
volatile int32_t axis1EncoderCount = 0;

char receiveBuffer[64];
uint8_t receiveLength = 0;
bool receiveOverflow = false;

int16_t requestedAxis0Command = 0;
int16_t requestedAxis1Command = 0;
bool relativeStepActive = false;
uint8_t relativeStepAxis = 0;
int32_t relativeStepTarget = 0;
int32_t relativeStepStart = 0;
uint16_t relativeStepMagnitude = 0;
int8_t relativeStepDirection = 0;
uint32_t relativeStepStartedMs = 0;
uint32_t lastValidCommandMs = 0;
bool commandReceived = false;
uint16_t faultFlags = FLAG_CURRENT_SCALE_UNKNOWN |
    (MAX_ABS_COMMAND == 0 ? static_cast<uint16_t>(FLAG_MOTION_LOCKED) : 0U);

uint16_t currentZeroAdc[2] = {512, 512};
uint16_t currentFilteredAdc[2] = {512, 512};
uint32_t lastCurrentSampleUs = 0;
uint32_t lastTelemetryMs = 0;

void disableMotorOutputs() {
  // PWM is removed before the global enable line is lowered.
  analogWrite(M1_PWM_PIN, 0);
  analogWrite(M2_PWM_PIN, 0);
  digitalWrite(MOTOR_ENABLE_PIN, LOW);
  digitalWrite(M1_DIR1_PIN, LOW);
  digitalWrite(M1_DIR2_PIN, LOW);
  digitalWrite(M2_DIR1_PIN, LOW);
  digitalWrite(M2_DIR2_PIN, LOW);
  requestedAxis0Command = 0;
  requestedAxis1Command = 0;
  relativeStepActive = false;
  relativeStepDirection = 0;
}

void updateMotorEnable() {
  digitalWrite(
      MOTOR_ENABLE_PIN,
      (requestedAxis0Command != 0 || requestedAxis1Command != 0) ? HIGH : LOW);
}

void disableAxis0Output() {
  analogWrite(M2_PWM_PIN, 0);
  digitalWrite(M2_DIR1_PIN, LOW);
  digitalWrite(M2_DIR2_PIN, LOW);
  requestedAxis0Command = 0;
  updateMotorEnable();
}

void disableAxis1Output() {
  analogWrite(M1_PWM_PIN, 0);
  digitalWrite(M1_DIR1_PIN, LOW);
  digitalWrite(M1_DIR2_PIN, LOW);
  requestedAxis1Command = 0;
  updateMotorEnable();
}

void configureSafeOutputs() {
  // Preload the output latches LOW before changing each pin to OUTPUT.
  digitalWrite(MOTOR_ENABLE_PIN, LOW);
  digitalWrite(M1_PWM_PIN, LOW);
  digitalWrite(M1_DIR1_PIN, LOW);
  digitalWrite(M1_DIR2_PIN, LOW);
  digitalWrite(M2_PWM_PIN, LOW);
  digitalWrite(M2_DIR1_PIN, LOW);
  digitalWrite(M2_DIR2_PIN, LOW);

  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  pinMode(M1_PWM_PIN, OUTPUT);
  pinMode(M1_DIR1_PIN, OUTPUT);
  pinMode(M1_DIR2_PIN, OUTPUT);
  pinMode(M2_PWM_PIN, OUTPUT);
  pinMode(M2_DIR1_PIN, OUTPUT);
  pinMode(M2_DIR2_PIN, OUTPUT);
  disableMotorOutputs();
}

void applyAxis0Command(int16_t command) {
  if (MAX_ABS_COMMAND == 0 || command == 0) {
    disableAxis0Output();
    return;
  }

  const uint8_t pwm = static_cast<uint8_t>(abs(command));
  digitalWrite(M2_DIR1_PIN, command > 0 ? HIGH : LOW);
  digitalWrite(M2_DIR2_PIN, command > 0 ? LOW : HIGH);
  analogWrite(M2_PWM_PIN, pwm);
  requestedAxis0Command = command;
  updateMotorEnable();
}

void applyAxis1Command(int16_t command) {
  if (MAX_ABS_COMMAND == 0 || command == 0) {
    disableAxis1Output();
    return;
  }

  const uint8_t pwm = static_cast<uint8_t>(abs(command));
  digitalWrite(M1_DIR1_PIN, command > 0 ? HIGH : LOW);
  digitalWrite(M1_DIR2_PIN, command > 0 ? LOW : HIGH);
  analogWrite(M1_PWM_PIN, pwm);
  requestedAxis1Command = command;
  updateMotorEnable();
}

void applyMotorCommands(int16_t axis0Command, int16_t axis1Command) {
  // Remove the shared enable before changing either direction. This prevents
  // a stale PWM/direction combination on one bridge while updating the other.
  digitalWrite(MOTOR_ENABLE_PIN, LOW);
  analogWrite(M2_PWM_PIN, 0);
  analogWrite(M1_PWM_PIN, 0);

  if (axis0Command == 0) {
    digitalWrite(M2_DIR1_PIN, LOW);
    digitalWrite(M2_DIR2_PIN, LOW);
  } else {
    digitalWrite(M2_DIR1_PIN, axis0Command > 0 ? HIGH : LOW);
    digitalWrite(M2_DIR2_PIN, axis0Command > 0 ? LOW : HIGH);
    analogWrite(M2_PWM_PIN, static_cast<uint8_t>(abs(axis0Command)));
  }

  if (axis1Command == 0) {
    digitalWrite(M1_DIR1_PIN, LOW);
    digitalWrite(M1_DIR2_PIN, LOW);
  } else {
    digitalWrite(M1_DIR1_PIN, axis1Command > 0 ? HIGH : LOW);
    digitalWrite(M1_DIR2_PIN, axis1Command > 0 ? LOW : HIGH);
    analogWrite(M1_PWM_PIN, static_cast<uint8_t>(abs(axis1Command)));
  }

  requestedAxis0Command = axis0Command;
  requestedAxis1Command = axis1Command;
  updateMotorEnable();
}

void encoderAxis0Isr() {
  const bool channelA = digitalRead(AXIS0_ENCODER_A_PIN);
  const bool channelB = digitalRead(AXIS0_ENCODER_B_PIN);
  axis0EncoderCount += channelA == channelB ? 1 : -1;
}

ISR(PCINT0_vect) {
  const bool channelA = digitalRead(AXIS1_ENCODER_A_PIN);
  const bool channelB = digitalRead(AXIS1_ENCODER_B_PIN);
  axis1EncoderCount += channelA == channelB ? 1 : -1;
}

int32_t readAxis0EncoderAtomic() {
  noInterrupts();
  const int32_t count = axis0EncoderCount;
  interrupts();
  return count;
}

int32_t readAxis1EncoderAtomic() {
  noInterrupts();
  const int32_t count = axis1EncoderCount;
  interrupts();
  return count;
}

int32_t readAxis0OperationalCount() {
  return AXIS0_ENCODER_SIGN * readAxis0EncoderAtomic();
}

int32_t readAxis1OperationalCount() {
  return AXIS1_ENCODER_SIGN * readAxis1EncoderAtomic();
}

void zeroAxis0Encoder() {
  noInterrupts();
  axis0EncoderCount = 0;
  interrupts();
}

void zeroAxis1Encoder() {
  noInterrupts();
  axis1EncoderCount = 0;
  interrupts();
}

void calibrateCurrentZero() {
  disableMotorOutputs();
  (void)analogRead(AXIS0_CURRENT_PIN);
  (void)analogRead(AXIS1_CURRENT_PIN);
  uint32_t sum0 = 0;
  uint32_t sum1 = 0;
  for (uint16_t sample = 0; sample < CURRENT_CALIBRATION_SAMPLES; ++sample) {
    sum0 += analogRead(AXIS0_CURRENT_PIN);
    sum1 += analogRead(AXIS1_CURRENT_PIN);
    delay(2);
  }
  currentZeroAdc[0] = static_cast<uint16_t>(
      sum0 / CURRENT_CALIBRATION_SAMPLES);
  currentZeroAdc[1] = static_cast<uint16_t>(
      sum1 / CURRENT_CALIBRATION_SAMPLES);
  currentFilteredAdc[0] = currentZeroAdc[0];
  currentFilteredAdc[1] = currentZeroAdc[1];
}

bool parseLongStrict(char* token, long& value) {
  if (token == nullptr || *token == '\0') {
    return false;
  }
  char* end = nullptr;
  value = strtol(token, &end, 10);
  return end != token && *end == '\0';
}

void sendError(const char* reason) {
  Serial.print(F("ERR,"));
  Serial.println(reason);
}

void handleCommand(char* line) {
  if (strcmp(line, "STOP") == 0) {
    disableMotorOutputs();
    commandReceived = false;
    faultFlags &= static_cast<uint16_t>(~FLAG_COMMAND_TIMEOUT);
    Serial.println(F("OK,STOP"));
    return;
  }

  if (strcmp(line, "ZERO_ENC") == 0) {
    disableMotorOutputs();
    zeroAxis0Encoder();
    zeroAxis1Encoder();
    faultFlags &= static_cast<uint16_t>(~FLAG_POSITION_LIMIT);
    Serial.println(F("OK,ZERO_ENC"));
    return;
  }

  if (strcmp(line, "CAL_CURRENT") == 0) {
    calibrateCurrentZero();
    Serial.print(F("OK,CAL_CURRENT,"));
    Serial.print(currentZeroAdc[0]);
    Serial.print(',');
    Serial.println(currentZeroAdc[1]);
    return;
  }

  char* savePointer = nullptr;
  char* command = strtok_r(line, ",", &savePointer);
  if (command == nullptr) {
    faultFlags |= FLAG_MALFORMED_COMMAND;
    sendError("EMPTY");
    return;
  }

  if (strcmp(command, "PING") == 0) {
    char* sequenceToken = strtok_r(nullptr, ",", &savePointer);
    char* extraToken = strtok_r(nullptr, ",", &savePointer);
    long sequence = 0;
    if (!parseLongStrict(sequenceToken, sequence) || sequence < 0 ||
        extraToken != nullptr) {
      faultFlags |= FLAG_MALFORMED_COMMAND;
      sendError("PING_FORMAT");
      return;
    }
    Serial.print(F("PONG,"));
    Serial.println(sequence);
    return;
  }

  if (strcmp(command, "CMD") == 0) {
    char* sequenceToken = strtok_r(nullptr, ",", &savePointer);
    char* axis0Token = strtok_r(nullptr, ",", &savePointer);
    char* axis1Token = strtok_r(nullptr, ",", &savePointer);
    char* extraToken = strtok_r(nullptr, ",", &savePointer);
    long sequence = 0;
    long axis0 = 0;
    long axis1 = 0;
    if (!parseLongStrict(sequenceToken, sequence) || sequence < 0 ||
        !parseLongStrict(axis0Token, axis0) ||
        !parseLongStrict(axis1Token, axis1) || extraToken != nullptr ||
        axis0 < -255 || axis0 > 255 || axis1 < -255 || axis1 > 255) {
      disableMotorOutputs();
      faultFlags |= FLAG_MALFORMED_COMMAND;
      sendError("CMD_FORMAT");
      return;
    }

    if (abs(axis0) > MAX_ABS_COMMAND || abs(axis1) > MAX_ABS_COMMAND) {
      disableMotorOutputs();
      faultFlags |= FLAG_MOTION_LOCKED;
      sendError("MOTION_LOCKED");
      return;
    }

    relativeStepActive = false;
    relativeStepDirection = 0;
    faultFlags &= static_cast<uint16_t>(~FLAG_POSITION_LIMIT);
    applyMotorCommands(
        static_cast<int16_t>(axis0), static_cast<int16_t>(axis1));
    lastValidCommandMs = millis();
    commandReceived = true;
    faultFlags &= static_cast<uint16_t>(~FLAG_COMMAND_TIMEOUT);
    return;
  }

  if (strcmp(command, "STEP") == 0) {
    char* sequenceToken = strtok_r(nullptr, ",", &savePointer);
    char* deltaToken = strtok_r(nullptr, ",", &savePointer);
    char* pwmToken = strtok_r(nullptr, ",", &savePointer);
    char* extraToken = strtok_r(nullptr, ",", &savePointer);
    long sequence = 0;
    long delta = 0;
    long pwm = 0;
    if (!parseLongStrict(sequenceToken, sequence) || sequence < 0 ||
        !parseLongStrict(deltaToken, delta) ||
        !parseLongStrict(pwmToken, pwm) || extraToken != nullptr ||
        delta == 0 || delta < -MAX_RELATIVE_STEP_COUNTS ||
        delta > MAX_RELATIVE_STEP_COUNTS || pwm <= 0 ||
        pwm > MAX_ABS_COMMAND) {
      disableMotorOutputs();
      faultFlags |= FLAG_MALFORMED_COMMAND;
      sendError("STEP_FORMAT");
      return;
    }

    // STEP remains the backward-compatible axis-0 primitive. Stop any prior
    // two-axis velocity command before starting the bounded move.
    disableMotorOutputs();
    relativeStepAxis = 0;
    const int32_t startCount = readAxis0EncoderAtomic();
    relativeStepTarget = startCount + static_cast<int32_t>(delta);
    relativeStepStart = startCount;
    relativeStepMagnitude = static_cast<uint16_t>(abs(delta));
    relativeStepDirection = delta > 0 ? 1 : -1;
    relativeStepActive = true;
    relativeStepStartedMs = millis();
    applyAxis0Command(static_cast<int16_t>(
        relativeStepDirection * static_cast<int16_t>(pwm)));
    lastValidCommandMs = millis();
    commandReceived = true;
    faultFlags &= static_cast<uint16_t>(~FLAG_COMMAND_TIMEOUT);
    Serial.print(F("OK,STEP,"));
    Serial.print(sequence);
    Serial.print(',');
    Serial.println(relativeStepTarget);
    return;
  }

  if (strcmp(command, "STEP_AXIS") == 0) {
    char* sequenceToken = strtok_r(nullptr, ",", &savePointer);
    char* axisToken = strtok_r(nullptr, ",", &savePointer);
    char* deltaToken = strtok_r(nullptr, ",", &savePointer);
    char* pwmToken = strtok_r(nullptr, ",", &savePointer);
    char* extraToken = strtok_r(nullptr, ",", &savePointer);
    long sequence = 0;
    long axis = 0;
    long delta = 0;
    long pwm = 0;
    if (!parseLongStrict(sequenceToken, sequence) || sequence < 0 ||
        !parseLongStrict(axisToken, axis) || axis < 0 || axis > 1 ||
        !parseLongStrict(deltaToken, delta) ||
        !parseLongStrict(pwmToken, pwm) || extraToken != nullptr ||
        delta == 0 || delta < -MAX_RELATIVE_STEP_COUNTS ||
        delta > MAX_RELATIVE_STEP_COUNTS || pwm <= 0 ||
        pwm > MAX_ABS_COMMAND) {
      disableMotorOutputs();
      faultFlags |= FLAG_MALFORMED_COMMAND;
      sendError("STEP_AXIS_FORMAT");
      return;
    }

    // A bounded raw-encoder displacement is also available on axis 1. This
    // primitive intentionally does not depend on the operational zero, which
    // is lost whenever opening the USB serial port resets the ATmega328P.
    disableMotorOutputs();
    relativeStepAxis = static_cast<uint8_t>(axis);
    const int32_t startCount = relativeStepAxis == 0
        ? readAxis0EncoderAtomic() : readAxis1EncoderAtomic();
    relativeStepTarget = startCount + static_cast<int32_t>(delta);
    relativeStepStart = startCount;
    relativeStepMagnitude = static_cast<uint16_t>(abs(delta));
    relativeStepDirection = delta > 0 ? 1 : -1;
    relativeStepActive = true;
    relativeStepStartedMs = millis();
    const int16_t commandValue = static_cast<int16_t>(
        relativeStepDirection * static_cast<int16_t>(pwm));
    if (relativeStepAxis == 0) {
      applyAxis0Command(commandValue);
    } else {
      applyAxis1Command(commandValue);
    }
    lastValidCommandMs = millis();
    commandReceived = true;
    faultFlags &= static_cast<uint16_t>(~FLAG_COMMAND_TIMEOUT);
    faultFlags &= static_cast<uint16_t>(~FLAG_POSITION_LIMIT);
    Serial.print(F("OK,STEP_AXIS,"));
    Serial.print(sequence);
    Serial.print(',');
    Serial.print(relativeStepAxis);
    Serial.print(',');
    Serial.println(relativeStepTarget);
    return;
  }

  faultFlags |= FLAG_MALFORMED_COMMAND;
  sendError("UNKNOWN_COMMAND");
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    const char value = static_cast<char>(Serial.read());
    if (value == '\n') {
      if (receiveOverflow) {
        faultFlags |= FLAG_RX_OVERFLOW;
        sendError("RX_OVERFLOW");
      } else if (receiveLength > 0) {
        if (receiveBuffer[receiveLength - 1] == '\r') {
          --receiveLength;
        }
        receiveBuffer[receiveLength] = '\0';
        handleCommand(receiveBuffer);
      }
      receiveLength = 0;
      receiveOverflow = false;
      continue;
    }

    if (!receiveOverflow) {
      if (receiveLength < sizeof(receiveBuffer) - 1) {
        receiveBuffer[receiveLength++] = value;
      } else {
        receiveOverflow = true;
      }
    }
  }
}

void sampleCurrentIfDue() {
  const uint32_t nowUs = micros();
  if (nowUs - lastCurrentSampleUs < CURRENT_SAMPLE_PERIOD_US) {
    return;
  }
  lastCurrentSampleUs = nowUs;
  const uint16_t raw0 = analogRead(AXIS0_CURRENT_PIN);
  const uint16_t raw1 = analogRead(AXIS1_CURRENT_PIN);
  currentFilteredAdc[0] = static_cast<uint16_t>(
      (7UL * currentFilteredAdc[0] + raw0 + 4UL) / 8UL);
  currentFilteredAdc[1] = static_cast<uint16_t>(
      (7UL * currentFilteredAdc[1] + raw1 + 4UL) / 8UL);
}

void enforceWatchdog() {
  // Relative STEP motions have their own encoder-target/timeout termination.
  if (!commandReceived || relativeStepActive) {
    return;
  }
  if (millis() - lastValidCommandMs > COMMAND_TIMEOUT_MS) {
    disableMotorOutputs();
    commandReceived = false;
    faultFlags |= FLAG_COMMAND_TIMEOUT;
  }
}

void enforceRelativeStepTarget() {
  if (!relativeStepActive) {
    return;
  }
  const int32_t count = relativeStepAxis == 0
      ? readAxis0EncoderAtomic() : readAxis1EncoderAtomic();
  const int32_t displacement = count - relativeStepStart;
  const bool reached = abs(displacement) >= relativeStepMagnitude;
  if (reached || millis() - relativeStepStartedMs > RELATIVE_STEP_TIMEOUT_MS) {
    if (!reached) {
      faultFlags |= FLAG_COMMAND_TIMEOUT;
    }
    disableMotorOutputs();
    commandReceived = false;
  }
}

void enforceOperationalPositionLimits() {
  if (requestedAxis0Command != 0 &&
      !(relativeStepActive && relativeStepAxis == 0)) {
    const int32_t position = readAxis0OperationalCount();
    const int16_t operationalDirection =
        AXIS0_COMMAND_TO_OPERATIONAL_SIGN * requestedAxis0Command;
    const bool atLimit =
        (operationalDirection > 0 &&
         position >= AXIS0_OPERATIONAL_MAX_COUNT) ||
        (operationalDirection < 0 &&
         position <= AXIS0_OPERATIONAL_MIN_COUNT);
    if (atLimit) {
      disableAxis0Output();
      faultFlags |= FLAG_POSITION_LIMIT;
    }
  }

  if (requestedAxis1Command != 0 &&
      !(relativeStepActive && relativeStepAxis == 1)) {
    const int32_t position = readAxis1OperationalCount();
    const int16_t operationalDirection =
        AXIS1_COMMAND_TO_OPERATIONAL_SIGN * requestedAxis1Command;
    const bool atLimit =
        (operationalDirection > 0 &&
         position >= AXIS1_OPERATIONAL_MAX_COUNT) ||
        (operationalDirection < 0 &&
         position <= AXIS1_OPERATIONAL_MIN_COUNT);
    if (atLimit) {
      disableAxis1Output();
      faultFlags |= FLAG_POSITION_LIMIT;
    }
  }

  if (requestedAxis0Command == 0 && requestedAxis1Command == 0) {
    commandReceived = false;
  }
}

void sendTelemetryIfDue() {
  const uint32_t nowMs = millis();
  if (nowMs - lastTelemetryMs < TELEMETRY_PERIOD_MS) {
    return;
  }
  lastTelemetryMs = nowMs;

  static uint32_t telemetrySequence = 0;
  Serial.print(F("STATE,"));
  Serial.print(telemetrySequence++);
  Serial.print(',');
  Serial.print(micros());
  Serial.print(',');
  Serial.print(readAxis0EncoderAtomic());
  Serial.print(',');
  Serial.print(readAxis1EncoderAtomic());
  Serial.print(',');
  Serial.print(currentFilteredAdc[0]);
  Serial.print(',');
  Serial.print(currentFilteredAdc[1]);
  Serial.print(',');
  Serial.print(CURRENT_UNAVAILABLE_MA);
  Serial.print(',');
  Serial.print(CURRENT_UNAVAILABLE_MA);
  Serial.print(',');
  Serial.println(faultFlags);
}

void setup() {
  configureSafeOutputs();

  pinMode(AXIS0_ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(AXIS0_ENCODER_B_PIN, INPUT_PULLUP);
  pinMode(AXIS1_ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(AXIS1_ENCODER_B_PIN, INPUT_PULLUP);
  pinMode(AXIS0_CURRENT_PIN, INPUT);
  pinMode(AXIS1_CURRENT_PIN, INPUT);

  Serial.begin(115200);
  calibrateCurrentZero();
  attachInterrupt(
      digitalPinToInterrupt(AXIS0_ENCODER_A_PIN), encoderAxis0Isr, CHANGE);

  // Enable pin-change interrupts for PCINT3 (D11) only. Channel B (D12) is
  // sampled inside the ISR, matching the x2 decoding used on axis 0.
  PCIFR |= _BV(PCIF0);
  PCMSK0 |= _BV(PCINT3);
  PCICR |= _BV(PCIE0);

  lastTelemetryMs = millis();
  lastCurrentSampleUs = micros();
  Serial.print(F("BOOT,BLUNO_MOTOR_CONTROLLER,2,MAX_PWM,"));
  Serial.println(MAX_ABS_COMMAND);
}

void loop() {
  readSerialCommands();
  enforceRelativeStepTarget();
  enforceOperationalPositionLimits();
  enforceWatchdog();
  sampleCurrentIfDue();
  sendTelemetryIfDue();
}
