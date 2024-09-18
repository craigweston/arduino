#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Rotary.h>
#include <si5351.h>

const uint8_t encoderPinA = 8;
const uint8_t encoderPinB =  9;
const uint8_t encoderSwitchPin = 7;

const int32_t Si5351Cal = 127850;
const uint16_t minStep = 1;
const uint16_t maxStep = 10000;

LiquidCrystal_I2C lcd(0x3F, 16, 2);
Rotary r = Rotary(encoderPinA, encoderPinB);
Si5351 si5351;

volatile uint32_t freq = 7200000;
uint32_t lastFreq = 0;
uint32_t freqStep = 1000;

void setup() {
  Serial.begin(9600);
  delay(10000);
  setupEncoder();
  setupSi5351();
  setupDisplay();
}

void loop() {
  int switchState = digitalRead(encoderSwitchPin);
  if (switchState == LOW) {
    nextStep();
    delay(300);
  }
  if (lastFreq != freq) {
    lastFreq = freq;
    setSi5351();
    displayFreq();
  }
}

void setupEncoder() {
  pinMode(encoderSwitchPin, INPUT_PULLUP);
  r.begin();
  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT0) | (1 << PCINT1);
  sei();
}

void setupSi5351() {
  bool i2cFound  = si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
  if(i2cFound) {
    si5351.output_enable(SI5351_CLK0, 1);
    si5351.output_enable(SI5351_CLK1, 0);
    si5351.output_enable(SI5351_CLK2, 0);

    si5351.set_correction(Si5351Cal, SI5351_PLL_INPUT_XO);
    si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_2MA);

    setSi5351();

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
  } else {
    Serial.println("Device not found on I2C bus!");
  } 
}

void setSi5351() {
  si5351.set_freq(freq * SI5351_FREQ_MULT, SI5351_CLK0);
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
  Serial.println("INTERUPT");
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
}

void nextStep() {
  freqStep *= 10;
  if (freqStep > maxStep) {
    freqStep = minStep;
  }
  if (freqStep < 1000) {
    freq -= freq % 1000;
  }
}