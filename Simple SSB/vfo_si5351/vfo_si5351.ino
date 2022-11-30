#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Rotary.h>
#include <si5351.h>

const uint8_t encoderPinA = 9;
const uint8_t encoderPinB = 8;
const uint8_t encoderSwitchPin = 10;

const uint8_t pttPin = 11;
const uint8_t pttEnablePin = 12;
uint8_t pttState = HIGH;
uint8_t pttLastState = HIGH;

unsigned long lastDebounceTime = 0;
const uint8_t debounceDelay = 50;

LiquidCrystal_I2C lcd(0x3F, 16, 2);

Rotary r = Rotary(encoderPinA, encoderPinB);

Si5351 si5351;
const int32_t Si5351Cal = 119300;
const uint16_t minStep = 1;
const uint16_t maxStep = 10000;
volatile uint32_t freq = 7200000;
const uint32_t bfo = 8998500;
uint32_t lastFreq = 0;
uint32_t freqStep = 1000;

void setup() {
  Serial.begin (9600);
  delay(10000);
  setupPtt();
  Serial.println("Setting Encoder...");
  setupEncoder();
  Serial.println("Setting Si5351...");
  setupSi5351();
  Serial.println("Setting display...");
  setupDisplay();
  Serial.println("Complete.");
}

void loop() {
  int switchState = digitalRead(encoderSwitchPin);
  if (switchState == LOW) {
    nextStep();
    delay(300);
  }
  
  if (lastFreq != freq) {
    lastFreq = freq;
    setVfo();
    displayFreq();
  }

  readPtt();
}

void setupPtt() {
    pinMode(pttPin, INPUT);
    pinMode(pttEnablePin, OUTPUT);
}

void setupEncoder() {
  pinMode(encoderSwitchPin, INPUT_PULLUP);
  r.begin();
  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT4) | (1 << PCINT5);
  sei();
}

void setupSi5351() {
  bool i2cFound  = si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
  if(!i2cFound) {
    Serial.println("Device not found on I2C bus!");
  } else {
    si5351.output_enable(SI5351_CLK0, 1);
    si5351.output_enable(SI5351_CLK1, 0);
    si5351.output_enable(SI5351_CLK2, 1);

    si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
    si5351.set_correction(Si5351Cal, SI5351_PLL_INPUT_XO);

    si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
    si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_8MA);

    setVfo();
    setBfo();

    si5351.update_status();

    Serial.print("SYS_INIT: ");
    Serial.print(si5351.dev_status.SYS_INIT);
    Serial.print("  LOL_A: ");
    Serial.print(si5351.dev_status.LOL_A);
    Serial.print("  LOL_B: ");
    Serial.print(si5351.dev_status.LOL_B);
    Serial.print("  LOS: ");
    Serial.print(si5351.dev_status.LOS);
    Serial.print("  REVID: ");
    Serial.println(si5351.dev_status.REVID);
    si5351.update_status();
  }
}

void setVfo() {
  si5351.set_freq((freq + bfo) * SI5351_FREQ_MULT, SI5351_CLK2);
}

void setBfo() {
    si5351.set_freq(bfo * SI5351_FREQ_MULT, SI5351_CLK0);
}

void setupDisplay() {
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);                   
  lcd.print("HELLO");  
  delay(2000);
  lcd.clear();
}

ISR(PCINT0_vect) {
  unsigned char result = r.process();
  if (result == DIR_CW) {
    setFreq(1);
  }
  else if (result == DIR_CCW) {
    setFreq(-1);
  }
}

void setFreq(int dir) {
  freq = freq + (dir * freqStep);
}

void displayFreq() {
  uint16_t mhz = freq / 1000000;
  uint16_t khz = freq % 1000000 / 1000;
  uint16_t hz = freq % 1000;
  char buffer[11];
  snprintf(buffer, sizeof(buffer), "%02d.%03d.%03d", mhz, khz, hz);
  lcd.setCursor(0, 0);                   
  lcd.print(buffer);
  Serial.print(buffer);
}

void nextStep() {
  freqStep *= 10;
  if (freqStep > maxStep) {
    freqStep = minStep;
  }
  if (freqStep < 1000) {
    long f = freq % 1000;
    freq -= f;
  }
}

bool readPtt() {
  int reading = digitalRead(pttPin);
  if (reading != pttLastState) {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != pttState) {
      pttState = reading;
      if (reading == HIGH) {
        digitalWrite(pttEnablePin, LOW);
      } else {
        digitalWrite(pttEnablePin, HIGH);
      }
    }
  }
  pttLastState = reading;
}
