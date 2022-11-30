#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Rotary.h>
#include <si5351.h>

#define ENCODER_PIN_A 8
#define ENCODER_PIN_B 9
#define ENCODER_SWITCH_PIN 10 

#define SI5351_CAL  127850 
#define MIN_STEP 1
#define MAX_STEP 10000

LiquidCrystal_I2C lcd(0x3F, 16, 2);
Rotary r = Rotary(ENCODER_PIN_A, ENCODER_PIN_B);
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
  int switchState = digitalRead(ENCODER_SWITCH_PIN);
  if (switchState == LOW) {
    nextStep();
    delay(300);
  }
  if (lastFreq != freq) {
    lastFreq = freq;
    Serial.println("Setting Si5351...");
    setSi5351();
    displayFreq();
  }
}

void setupEncoder() {
  pinMode(ENCODER_SWITCH_PIN, INPUT_PULLUP);
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
    si5351.output_enable(SI5351_CLK2, 0);

    si5351.set_correction(SI5351_CAL, SI5351_PLL_INPUT_XO);
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
  uint32_t mhz = freq / 1000000;
  uint32_t khz = (freq % 1000000) / 1000;
  uint32_t hz = freq % 1000;

  char buffer[11];
  sprintf(buffer, "%02d.%03d.%03d", mhz, khz, hz);

  lcd.setCursor(0, 0);                   
  lcd.print(buffer);
}

void nextStep() {
  freqStep *= 10;
  if (freqStep > MAX_STEP) {
    freqStep = MIN_STEP;
  }
  if (freqStep < 1000) {
    freq -= freq % 1000;
  }
}