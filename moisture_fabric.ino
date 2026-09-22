#include <EEPROM.h>

// ---------------- Pins ----------------
const int PIN_EXC   = 2;    // Excitation pin: outputs HIGH only during measurement
const int PIN_SENSE = A0;   // Voltage divider midpoint
const int LED_G = 5, LED_Y = 6, LED_R = 7;

// ---------------- Calibration ----------------
const int DEF_LO = 30;      // Default dry-point ADC value
const int DEF_HI = 700;     // Default wet-point ADC value
int loCal = DEF_LO, hiCal = DEF_HI;
const int SPAN_MIN = 100;   // Wet point must be at least this much higher than dry point

const int PCT_FULL = 100;         // Full-scale percentage
const int TH1 = 30, TH2 = 60;     // Yellow and red thresholds (%)
const int HYST = 5;               // Hysteresis (%)

struct Cal { uint8_t magic; int lo, hi; };
const uint8_t MAGIC = 0xC3;

// ---------------- State ----------------
char mode = 'm';
int  level = 0;                   // 0 Dry, 1 Slightly Wet, 2 Very Wet
unsigned long lastMeas = 0, lastPrint = 0;
int lastRaw = 0, lastPct = 0;

// ================= Calibration Storage =================
void saveCal() {
  Cal c; c.magic = MAGIC; c.lo = loCal; c.hi = hiCal;
  EEPROM.put(0, c);
}

bool loadCal() {
  Cal c; EEPROM.get(0, c);
  if (c.magic != MAGIC) return false;
  if (c.hi - c.lo < SPAN_MIN) return false;
  if (c.lo < 0 || c.hi > 1023) return false;
  loCal = c.lo; hiCal = c.hi;
  return true;
}

int toPct(int raw) {
  if (raw <= loCal) return 0;
  if (raw >= hiCal) return PCT_FULL;
  return (int)((long)(raw - loCal) * PCT_FULL / (hiCal - loCal));
}

// ================= Measurement =================
// Pulsed excitation + trimmed mean:
// Power on -> wait 50 ms for the 100 nF capacitor to stabilize
// (RC ≈ 10 ms, so wait approximately 5× RC) ->
// Take 8 samples over 20 ms (helps suppress 50 Hz mains interference) ->
// Power off -> sort the samples and average the middle 4 values.
int readRaw() {
  digitalWrite(PIN_EXC, HIGH);
  delay(50);

  int v[8];

  for (int i = 0; i < 8; i++) {
    v[i] = analogRead(PIN_SENSE);
    delayMicroseconds(2500);
  }

  digitalWrite(PIN_EXC, LOW);

  // Insertion sort
  for (int i = 1; i < 8; i++) {
    int key = v[i], j = i - 1;

    while (j >= 0 && v[j] > key) {
      v[j + 1] = v[j];
      j--;
    }

    v[j + 1] = key;
  }

  long s = 0;

  for (int i = 2; i < 6; i++) {
    s += v[i];
  }

  return (int)(s / 4);
}

// ================= Level Classification & LEDs =================
void updateLevel(int pct) {
  // Three-level classification with hysteresis
  // to prevent rapid switching around the thresholds.
  if (level == 0) {
    if (pct >= TH2) level = 2;
    else if (pct >= TH1) level = 1;
  } 
  else if (level == 1) {
    if (pct >= TH2) level = 2;
    else if (pct < TH1 - HYST) level = 0;
  } 
  else { // level == 2
    if (pct < TH2 - HYST) {
      level = (pct >= TH1) ? 1 : 0;
    }
  }
}

void showLED() {
  digitalWrite(LED_G, level == 0 ? HIGH : LOW);
  digitalWrite(LED_Y, level == 1 ? HIGH : LOW);
  digitalWrite(LED_R, level == 2 ? HIGH : LOW);
}

// ================= Calibration Commands =================
void setDry() {
  int raw = readRaw();

  if (hiCal - raw < SPAN_MIN) {
    Serial.print(F("# Rejected: dry point "));
    Serial.print(raw);
    Serial.print(F(" is too close to wet point "));
    Serial.print(hiCal);
    Serial.println(F(". Difference must be at least 100. Use d to restore defaults or measure again."));
    return;
  }

  loCal = raw;
  saveCal();

  Serial.print(F("# Dry point recorded: ADC "));
  Serial.println(loCal);
}

void setWet() {
  int raw = readRaw();

  if (raw - loCal < SPAN_MIN) {
    Serial.print(F("# Rejected: wet point "));
    Serial.print(raw);
    Serial.print(F(" is less than 100 above dry point "));
    Serial.print(loCal);
    Serial.println(F(". Make the fabric wetter or check the electrodes."));
    return;
  }

  hiCal = raw;
  saveCal();

  Serial.print(F("# Wet point recorded: ADC "));
  Serial.println(hiCal);
}

void printCal() {
  Serial.print(F("# Calibration: Dry point "));
  Serial.print(loCal);

  Serial.print(F("  Wet point "));
  Serial.print(hiCal);

  Serial.print(F("  Yellow LED @ "));
  Serial.print(TH1);

  Serial.print(F("%  Red LED @ "));
  Serial.print(TH2);

  Serial.println(F("%"));
}

void setDefaults() {
  loCal = DEF_LO;
  hiCal = DEF_HI;
  saveCal();

  Serial.println(F("# Default calibration restored (30/700)"));
}

void printHelp() {
  Serial.println(
    F("# z=Dry point  u=Wet point  t=Calibration  d=Defaults  m=Monitor  p=Plot  c=Raw ADC  h=Help")
  );
}

// ================= Main Process =================
void setup() {
  pinMode(PIN_EXC, OUTPUT);
  digitalWrite(PIN_EXC, LOW);

  pinMode(LED_G, OUTPUT);
  pinMode(LED_Y, OUTPUT);
  pinMode(LED_R, OUTPUT);

  Serial.begin(115200);

  Serial.println(F("# Fabric Moisture Test v1  Demonstration Prototype"));

  if (loadCal()) {
    Serial.println(F("# Previous calibration loaded"));
  } 
  else {
    Serial.println(
      F("# Using default calibration (Dry=30, Wet=700). Recommended: send z, then u to recalibrate.")
    );
  }

  Serial.println(
    F("# Two-step calibration: completely dry -> send z | wettest demonstration state -> send u | Done")
  );

  printHelp();
}

void loop() {
  // Commands
  if (Serial.available()) {
    char ch = Serial.read();

    if (ch == 'm' || ch == 'p' || ch == 'c') {
      mode = ch;

      if (mode != 'p') {
        Serial.print(F("# mode="));
        Serial.println(ch);
      }
    }
    else if (ch == 'z') setDry();
    else if (ch == 'u') setWet();
    else if (ch == 't') printCal();
    else if (ch == 'd') setDefaults();
    else if (ch == 'h' || ch == '?') printHelp();
  }

  // Measure every 500 ms
  // The excitation pin is powered only during the approximately 70 ms measurement period.
  if (millis() - lastMeas >= 500) {
    lastMeas = millis();

    lastRaw = readRaw();
    lastPct = toPct(lastRaw);

    updateLevel(lastPct);
    showLED();
  }

  // Print one line every 500 ms
  if (millis() - lastPrint >= 500) {
    lastPrint = millis();

    if (mode == 'p') {
      Serial.print(F("Wetness:"));
      Serial.print(lastPct);

      Serial.print(F(",T1:"));
      Serial.print(TH1);

      Serial.print(F(",T2:"));
      Serial.println(TH2);
    } 
    else if (mode == 'c') {
      Serial.print(F("ADC "));
      Serial.println(lastRaw);
    } 
    else {
      Serial.print(F("Moisture "));
      Serial.print(lastPct);

      Serial.print(F("%  ADC "));
      Serial.print(lastRaw);

      Serial.print(F("  Level "));

      Serial.print(
        level == 0 ? F("Dry (Green)") :
        level == 1 ? F("Slightly Wet (Yellow)") :
                     F("Very Wet (Red)")
      );

      Serial.println();
    }
  }
}