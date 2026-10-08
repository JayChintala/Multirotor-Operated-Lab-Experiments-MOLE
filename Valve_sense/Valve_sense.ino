int Pin1 = 4;
int Pin2 = 5;

int Valve1 = 8;
int Valve2 = 9;


int R1 = 10000;
int R2 = 10000;
int R3 = 10000;

int count = 0; // No valve is on

const int waterPin = A0;
const int LED = 13;

uint32_t time_start1 = 0;
uint32_t time_start2 = 0;
uint32_t time_current = 0;

int time_s = 10;

bool state_Valve1 = false; // true if valve receives power, false is off
bool state_Valve2 = false;
bool water_state = false; // Initialise with assuming no water detected. Imp for prototype 2

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


void setValveOn(int valve_num);
void setValveOff(int count);


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

pinMode(Valve1, OUTPUT);
pinMode(Valve2, OUTPUT);

digitalWrite(Valve1, LOW);
digitalWrite(Valve2, LOW);
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
  // digitalWrite(Valve1, HIGH);
  time_current=millis();
  if (state_Valve1 == true){
    if ((time_current - time_start1)>(time_s *1000)){
      setValveOff(1);
    }
  }
  if (state_Valve2 == true){
    if ((time_current - time_start2)>(time_s *1000)){
      setValveOff(2);
    }
  }
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
      if (water_state == false) {
        count+=1;
        setValveOn(count);
      }
      // Both polarities indicate water
      digitalWrite(LED, HIGH);
      water_state = true;
    }
    else {
      water_state= false;
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


// --------------------------------------------------
// Function Definitions
// --------------------------------------------------
void setValveOn(int valve_num){
  if (valve_num == 1){
    time_start1 = millis();
    digitalWrite(Valve1, HIGH);
    Serial.println("Valve 1 on");
    state_Valve1 = true;
  } else if (valve_num == 2){
    time_start2 = millis();
    digitalWrite(Valve2, HIGH);
    Serial.println("Valve 2 on");
    state_Valve2 = true;
  } else {
    // Throw an error.
    Serial.println("Both Solenoids already filled");
  }
}

void setValveOff(int valve_num){
  if (valve_num == 1){
    digitalWrite(Valve1, LOW);
    Serial.println("Valve 1 off");
    state_Valve1 = false;
  } else if (valve_num == 2){
    digitalWrite(Valve2, LOW);
    Serial.println("Valve 2 off");
    state_Valve2= false;
  } else {
    // Throw an error.
    Serial.println("FUCK YOU");
  }
}