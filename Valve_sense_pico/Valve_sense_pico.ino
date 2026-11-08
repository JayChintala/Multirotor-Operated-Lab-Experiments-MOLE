// Raspberry Pi Pico 2 W, using the Earle Philhower Arduino-Pico core.
// Select "Raspberry Pi Pico 2 W" in Arduino IDE before uploading.
// Pin numbers are GPIO numbers, not physical header pin numbers:
// GP4 (pin 6) and GP5 (pin 7): water sensor polarity drive
// GP26/A0 (pin 31): water sensor analog signal
// GP8 (pin 11) and GP9 (pin 12): valve driver logic inputs
// Use the Pico's 3.3 V supply for the sensor network. Never put 5 V on a GPIO
// or ADC input. Drive solenoids through suitable external drivers, not GPIO.
constexpr uint8_t PIN_1 = 4;
constexpr uint8_t PIN_2 = 5;
constexpr uint8_t VALVE_1 = 8;
constexpr uint8_t VALVE_2 = 9;
constexpr uint8_t WATER_PIN = A0;  // GP26, physical pin 31
constexpr uint8_t WATER_LED = LED_BUILTIN;

constexpr uint32_t POLARITY_INTERVAL_US = 1000;  // 500 Hz complete cycle
constexpr uint32_t SETTLE_TIME_US = 50;
constexpr uint32_t VALVE_ON_TIME_MS = 10000;

// The Uno's 3.2 V and 1.8 V thresholds were fractions of its 5 V supply.
// Preserve those fractions for the Pico's 3.3 V GPIO and ADC range.
constexpr float ADC_REFERENCE_V = 3.3f;
constexpr float THRESHOLD_HIGH_LOW_V = ADC_REFERENCE_V * (3.2f / 5.0f);
constexpr float THRESHOLD_LOW_HIGH_V = ADC_REFERENCE_V * (1.8f / 5.0f);

uint32_t lastSwitchUs = 0;
uint32_t valve1StartMs = 0;
uint32_t valve2StartMs = 0;
uint8_t valvesUsed = 0;

bool highLowPolarity = true;
bool samplePending = false;
bool haveHighLowSample = false;
bool haveLowHighSample = false;
bool waterHighLow = false;
bool waterLowHigh = false;
bool waterState = false;
bool valve1On = false;
bool valve2On = false;

void setSensorPolarity(bool highLow) {
  // Turn off the previously high pin before driving the other pin high.
  if (highLow) {
    digitalWrite(PIN_2, LOW);
    digitalWrite(PIN_1, HIGH);
  } else {
    digitalWrite(PIN_1, LOW);
    digitalWrite(PIN_2, HIGH);
  }
}

void setValveOn(uint8_t valveNumber) {
  if (valveNumber == 1) {
    valve1StartMs = millis();
    digitalWrite(VALVE_1, HIGH);
    valve1On = true;
    Serial.println("Valve 1 on");
  } else if (valveNumber == 2) {
    valve2StartMs = millis();
    digitalWrite(VALVE_2, HIGH);
    valve2On = true;
    Serial.println("Valve 2 on");
  }
}

void setValveOff(uint8_t valveNumber) {
  if (valveNumber == 1) {
    digitalWrite(VALVE_1, LOW);
    valve1On = false;
    Serial.println("Valve 1 off");
  } else if (valveNumber == 2) {
    digitalWrite(VALVE_2, LOW);
    valve2On = false;
    Serial.println("Valve 2 off");
  }
}

void updateWaterState() {
  // Evaluate only after a fresh reading from each polarity.
  if (!haveHighLowSample || !haveLowHighSample) return;
  haveHighLowSample = false;
  haveLowHighSample = false;

  const bool waterDetected = waterHighLow && waterLowHigh;
  if (waterDetected && !waterState) {
    if (valvesUsed < 2) {
      ++valvesUsed;
      setValveOn(valvesUsed);
    } else {
      Serial.println("Both solenoids already filled");
    }
  }
  waterState = waterDetected;
  digitalWrite(WATER_LED, waterDetected ? HIGH : LOW);
}

void setup() {
  // Keep the solenoid drivers off at startup.
  digitalWrite(VALVE_1, LOW);
  digitalWrite(VALVE_2, LOW);
  pinMode(VALVE_1, OUTPUT);
  pinMode(VALVE_2, OUTPUT);

  digitalWrite(PIN_1, HIGH);
  digitalWrite(PIN_2, LOW);
  pinMode(PIN_1, OUTPUT);
  pinMode(PIN_2, OUTPUT);
  pinMode(WATER_PIN, INPUT);
  pinMode(WATER_LED, OUTPUT);
  digitalWrite(WATER_LED, LOW);

  analogReadResolution(12);  // ADC readings: 0..4095
  Serial.begin(115200);
  lastSwitchUs = micros();
}

void loop() {
  const uint32_t nowMs = millis();
  if (valve1On && nowMs - valve1StartMs >= VALVE_ON_TIME_MS) setValveOff(1);
  if (valve2On && nowMs - valve2StartMs >= VALVE_ON_TIME_MS) setValveOff(2);

  const uint32_t nowUs = micros();
  if (samplePending && nowUs - lastSwitchUs >= SETTLE_TIME_US) {
    const float voltage = analogRead(WATER_PIN) * ADC_REFERENCE_V / 4095.0f;
    if (highLowPolarity) {
      waterHighLow = voltage < THRESHOLD_HIGH_LOW_V;
      haveHighLowSample = true;
    } else {
      waterLowHigh = voltage > THRESHOLD_LOW_HIGH_V;
      haveLowHighSample = true;
    }
    samplePending = false;
    updateWaterState();
  }

  // Switch only after the previous polarity has been sampled. The elapsed-time
  // subtraction stays correct across micros() rollover.
  if (!samplePending && micros() - lastSwitchUs >= POLARITY_INTERVAL_US) {
    highLowPolarity = !highLowPolarity;
    setSensorPolarity(highLowPolarity);
    lastSwitchUs = micros();
    samplePending = true;
  }
}

