#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C
Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

const int PH_PIN = A0;
const float AREF_V = 5.0;

float NEUTRAL_V      = 4.35;
float SLOPE_VperPH   = -0.1152416;

const uint8_t NUM_SAMPLES = 15;
const uint16_t SAMPLE_DELAY_MS = 20;

const int A_IN1 = 9;
const int A_IN2 = 8;
const int B_IN1 = 6;
const int B_IN2 = 7;

float setpoint = 6.5;
const float window = 0.2;

const int BUTTON_DEC_PIN = 2;
const int BUTTON_INC_PIN = 3;

unsigned long lastDecPress = 0;
unsigned long lastIncPress = 0;
unsigned long showSetpointUntil = 0;

const unsigned long DOSE_DURATION = 1000;
const unsigned long COOLDOWN_TIME = 100;
unsigned long lastDoseTime = 0;

const unsigned long DEBOUNCE_MS = 80;

float readPHVoltage() {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < NUM_SAMPLES; i++) {
    sum += analogRead(PH_PIN);
    delay(SAMPLE_DELAY_MS);
  }
  float avgCounts = sum / float(NUM_SAMPLES);
  return (avgCounts * AREF_V) / 1023.0;
}

void drawScreen_PH(float pH, float volts, int raw) {
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  display.print(F("pH Monitor"));

  display.setTextSize(3);
  display.setCursor(0, 14);
  display.print(F("pH "));
  display.print(pH, 2);

  display.setTextSize(1);
  display.setCursor(0, 48);
  display.print(F("V="));
  display.print(volts, 3);
  display.print(F("  ADC="));
  display.print(raw);

  display.display();
}

void drawScreen_Setpoint() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  display.print(F("Setpoint"));

  display.setTextSize(3);
  display.setCursor(0, 25);
  display.print(setpoint, 1);

  display.display();
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  if (!display.begin(OLED_ADDR, true)) {
    while (1) { delay(10); }
  }

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 20);
  display.println(F("Starting..."));
  display.display();

  analogReference(DEFAULT);
  delay(500);

  pinMode(A_IN1, OUTPUT);
  pinMode(A_IN2, OUTPUT);
  pinMode(B_IN1, OUTPUT);
  pinMode(B_IN2, OUTPUT);

  pinMode(BUTTON_DEC_PIN, INPUT_PULLUP);
  pinMode(BUTTON_INC_PIN, INPUT_PULLUP);
}

void loop() {
  unsigned long now = millis();

  if (digitalRead(BUTTON_DEC_PIN) == LOW) {
    if (now - lastDecPress > DEBOUNCE_MS) {
      setpoint -= 0.1;
      if (setpoint < 4.0) setpoint = 4.0;
      showSetpointUntil = now + 5000;
      lastDecPress = now;
    }
  }

  if (digitalRead(BUTTON_INC_PIN) == LOW) {
    if (now - lastIncPress > DEBOUNCE_MS) {
      setpoint += 0.1;
      if (setpoint > 9.0) setpoint = 9.0;
      showSetpointUntil = now + 5000;
      lastIncPress = now;
    }
  }

  float volts = readPHVoltage();
  int raw = analogRead(PH_PIN);

  float pH = 7.0 + (volts - NEUTRAL_V) / SLOPE_VperPH;
  if (pH < 0) pH = 0;
  if (pH > 14) pH = 14;

  Serial.print(F("pH=")); Serial.print(pH, 2);
  Serial.print(F("  Set=")); Serial.println(setpoint, 1);

  if (now < showSetpointUntil) {
    drawScreen_Setpoint();
  } else {
    drawScreen_PH(pH, volts, raw);
  }

  bool inCooldown = (now - lastDoseTime < COOLDOWN_TIME);

  float highThresh = setpoint + window;
  float lowThresh  = setpoint - window;
  //return;
  if (!inCooldown) {
    if (pH > highThresh) {
      Serial.println("Dosing acid...");
      digitalWrite(A_IN2, LOW);
      analogWrite(A_IN1, 255);
      delay(DOSE_DURATION);
      analogWrite(A_IN1, 0);
      lastDoseTime = millis();
    }
    else if (pH < lowThresh) {
      Serial.println("Dosing base...");
      digitalWrite(B_IN2, LOW);
      analogWrite(B_IN1, 255);
      delay(DOSE_DURATION);
      analogWrite(B_IN1, 0);
      lastDoseTime = millis();
    }
  }
}
