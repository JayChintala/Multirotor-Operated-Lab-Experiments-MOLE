int Pin1 = 4;
int Pin2 = 5;

int R1 = 10000;
int R2 = 10000;
int R3 = 10000;

const int waterPin = A0;
const int LED = 13;

const float VCC = 5.0;

// Dry-circuit expected values
const float dryHighLow = VCC * (R2 + R3) / (R1 + R2 + R3); // 3.33 V
const float dryLowHigh = VCC * R1 / (R1 + R2 + R3);         // 1.67 V

// Water detection thresholds
// P1 HIGH / P2 LOW : water causes voltage to DROP
const float threshold1 = 3.2;

// P1 LOW / P2 HIGH : water causes voltage to RISE
const float threshold2 = 1.8;


// Timer interrupt frequency.
// Polarity changes every 1 ms.
// Complete polarity cycle = 2 ms = 500 Hz.
const unsigned long interruptFrequency = 1000;


// Variables shared with interrupt
volatile bool polarity = false;
volatile bool measurementReady = false;


// Results from the two polarity measurements
bool waterHighLow = false;
bool waterLowHigh = false;


// --------------------------------------------------
// SETUP
// --------------------------------------------------

void setup() {

  pinMode(Pin1, OUTPUT);
  pinMode(Pin2, OUTPUT);

  pinMode(waterPin, INPUT);
  pinMode(LED, OUTPUT);

  digitalWrite(LED, LOW);

  Serial.begin(115200);

  // Start with P1 HIGH, P2 LOW
  digitalWrite(Pin1, HIGH);
  digitalWrite(Pin2, LOW);

  // Timer1
  cli();

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  // CTC mode
  TCCR1B |= (1 << WGM12);

  // Prescaler = 8
  TCCR1B |= (1 << CS11);

  // 16 MHz / 8 = 2 MHz
  // 2,000,000 / 1000 - 1 = 1999
  OCR1A = 1999;

  TIMSK1 |= (1 << OCIE1A);

  sei();
}


// --------------------------------------------------
// MAIN LOOP
// --------------------------------------------------

void loop() {

  if (measurementReady) {

    noInterrupts();

    measurementReady = false;
    bool currentPolarity = polarity;

    interrupts();

    // Allow resistor network to settle after switching
    delayMicroseconds(50);

    int adc = analogRead(waterPin);

    float voltage = adc * VCC / 1023.0;


    // ----------------------------------------------
    // P1 HIGH / P2 LOW
    // ----------------------------------------------

    if (currentPolarity == false) {

      // Water causes voltage to DROP
      waterHighLow = (voltage < threshold1);

      // Serial.print("P1 HIGH / P2 LOW: ");
      // Serial.print(voltage);
      // Serial.print(" V | Water condition: ");
      // Serial.println(waterHighLow ? "YES" : "NO");
      
    }


    // ----------------------------------------------
    // P1 LOW / P2 HIGH
    // ----------------------------------------------

    else {

      // Water causes voltage to RISE
      waterLowHigh = (voltage > threshold2);

      // Serial.print("P1 LOW / P2 HIGH: ");
      // Serial.print(voltage);
      // Serial.print(" V | Water condition: ");
      // Serial.println(waterLowHigh ? "YES" : "NO");
    }


    // ----------------------------------------------
    // WATER DETECTION
    // ----------------------------------------------

    if (waterHighLow && waterLowHigh) {

      // Both polarities indicate water
      digitalWrite(LED, HIGH);

    }
    else {

      // Water not detected
      digitalWrite(LED, LOW);
    }
  }
  // delay(1000);
}


// --------------------------------------------------
// TIMER1 INTERRUPT
// --------------------------------------------------

ISR(TIMER1_COMPA_vect) {

  polarity = !polarity;

  if (polarity == false) {

    // P1 HIGH
    // P2 LOW

    PORTD = (PORTD & 0xCF) | 0x10;
  }

  else {

    // P1 LOW
    // P2 HIGH

    PORTD = (PORTD & 0xCF) | 0x20;
  }

  measurementReady = true;
}