// =============================================================================
// Arduino Pro Micro — DMX Relay Controller
// =============================================================================
// Receives DMX512 and drives 8 relays.
// DMX start address and on-threshold are adjustable via 4 buttons + 16×2 LCD.
// Settings save to EEPROM when returning to the home screen.
//
// Hardware
//   Relay outputs : pins 4, 5, 6, 7, 8, 9, 10, 16  (active-LOW relay module)
//   Buttons       : UP=A0  DOWN=A1  ENTER=A2  BACK=A3  (INPUT_PULLUP, LOW=pressed)
//   LCD           : 16×2 I2C at 0x27
//   DMX input     : MAX485 on Serial1 (pins 0/1), RE+DE tied LOW
//
// UI
//   Home screen : shows address, threshold, relay states, DMX status.
//   ENTER       : open settings.
//   Settings    : cursor selects Address or Threshold row.
//                 UP/DOWN change the value.
//                 ENTER moves to the next item (or saves+exits on the last one).
//                 BACK  saves and returns to home immediately.
// =============================================================================

#include <DMXSerial.h>
#include <Wire.h>
#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>
#include <util/delay.h>

// Use CPU cycle-counting instead of delay() during setup().
// On ATmega32U4, USB interrupts starve Timer0 at cold boot, making millis()
// and delay() run ~8× too slowly.  _delay_ms() is immune to this.
static void waitMs(uint16_t ms) { while (ms--) _delay_ms(1); }

// ── Hardware ─────────────────────────────────────────────────────────────────
LiquidCrystal_I2C lcd(0x27, 16, 2);

const uint8_t RELAY_PINS[8] = {4, 5, 6, 7, 8, 9, 10, 16};
const uint8_t BTN_UP    = A0;
const uint8_t BTN_DOWN  = A1;
const uint8_t BTN_ENTER = A2;
const uint8_t BTN_BACK  = A3;

// ── Config ───────────────────────────────────────────────────────────────────
struct Config { uint16_t magic; uint16_t address; uint8_t threshold; };
const uint16_t MAGIC   = 0xDA7A;
const int      EE_ADDR = 0;

uint16_t dmxAddress = 1;    // DMX start channel, 1–505
uint8_t  threshold  = 128;  // relay ON  when value >= threshold
                             // relay OFF when value <  (threshold - 8)  [hysteresis]

void loadConfig() {
  Config c;
  EEPROM.get(EE_ADDR, c);
  if (c.magic == MAGIC) {
    dmxAddress = constrain((uint16_t)c.address,   (uint16_t)1, (uint16_t)505);
    threshold  = constrain((uint8_t) c.threshold, (uint8_t) 1, (uint8_t) 255);
  }
}

void saveConfig() {
  Config c = {MAGIC, dmxAddress, threshold};
  EEPROM.put(EE_ADDR, c);
}

// ── Relays ───────────────────────────────────────────────────────────────────
bool relayOn[8] = {};

void applyRelays() {
  for (uint8_t i = 0; i < 8; i++) {
    uint8_t v   = DMXSerial.read(dmxAddress + i);
    uint8_t off = (threshold > 8) ? threshold - 8 : 0;
    if (!relayOn[i] && v >= threshold) relayOn[i] = true;
    if ( relayOn[i] && v <  off)       relayOn[i] = false;
    digitalWrite(RELAY_PINS[i], relayOn[i] ? LOW : HIGH);  // active-LOW
  }
}

void allRelaysOff() {
  for (uint8_t i = 0; i < 8; i++) {
    relayOn[i] = false;
    digitalWrite(RELAY_PINS[i], HIGH);
  }
}

// ── Buttons ───────────────────────────────────────────────────────────────────
struct Button {
  uint8_t       pin;
  uint8_t       lastStable;
  uint8_t       lastRead;
  unsigned long lastFlip;
};

Button btns[4] = {
  {BTN_UP,    HIGH, HIGH, 0},
  {BTN_DOWN,  HIGH, HIGH, 0},
  {BTN_ENTER, HIGH, HIGH, 0},
  {BTN_BACK,  HIGH, HIGH, 0},
};

// Returns true once per press (falling edge, 25 ms debounce).
bool pressed(uint8_t pin) {
  for (uint8_t i = 0; i < 4; i++) {
    if (btns[i].pin != pin) continue;
    uint8_t r = digitalRead(pin);
    if (r != btns[i].lastRead) { btns[i].lastRead = r; btns[i].lastFlip = millis(); }
    if (millis() - btns[i].lastFlip > 25 && r != btns[i].lastStable) {
      btns[i].lastStable = r;
      if (r == LOW) return true;
    }
  }
  return false;
}

// ── LCD helpers ───────────────────────────────────────────────────────────────
// Right-align a number in a field of `width` chars, space-padded on the left.
void lcdNum(uint16_t v, uint8_t width) {
  char buf[6];
  snprintf(buf, sizeof(buf), "%u", v);
  uint8_t len = strlen(buf);
  for (uint8_t i = len; i < width; i++) lcd.print(' ');
  lcd.print(buf);
}

// ── Home screen ───────────────────────────────────────────────────────────────
// Row 0: "Adr:001  Thr:128"   (16 chars)
// Row 1: "12345678  DMX   "   (relay pattern + DMX status)

void drawHome() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Adr:");  lcdNum(dmxAddress, 3);
  lcd.print("  Thr:"); lcdNum(threshold, 3);
}

void updateHome() {
  bool dmxOk = (DMXSerial.noDataSince() < 1000);
  lcd.setCursor(0, 1);
  for (uint8_t i = 0; i < 8; i++) lcd.print(relayOn[i] ? char('1' + i) : '-');
  lcd.print(dmxOk ? "  DMX   " : "  NO DMX");
}

// ── Settings screen ───────────────────────────────────────────────────────────
// Row 0: ">Adr:001        "   or  " Adr:001        "
// Row 1: " Thr:128        "   or  ">Thr:128        "

uint8_t settingIdx = 0;   // 0 = address, 1 = threshold

void drawSettings() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(settingIdx == 0 ? '>' : ' ');
  lcd.print("Adr:"); lcdNum(dmxAddress, 3);
  lcd.print("        ");

  lcd.setCursor(0, 1);
  lcd.print(settingIdx == 1 ? '>' : ' ');
  lcd.print("Thr:"); lcdNum(threshold, 3);
  lcd.print("        ");
}

// ── Screen state ─────────────────────────────────────────────────────────────
enum Screen { HOME, SETTINGS } screen = HOME;
unsigned long lastHomeUpdate = 0;

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  // Drive relays HIGH (de-energised) immediately — before anything else.
  // Active-LOW module: floating input = relay ON. Fix that right away.
  for (uint8_t i = 0; i < 8; i++) {
    pinMode(RELAY_PINS[i], OUTPUT);
    digitalWrite(RELAY_PINS[i], HIGH);
  }

  pinMode(BTN_UP,    INPUT_PULLUP);
  pinMode(BTN_DOWN,  INPUT_PULLUP);
  pinMode(BTN_ENTER, INPUT_PULLUP);
  pinMode(BTN_BACK,  INPUT_PULLUP);

  // Wait for PSU / LCD backpack to stabilise.
  // Must use waitMs, not delay() — millis() is unreliable during cold-boot USB
  // enumeration on ATmega32U4 (USB ISRs starve Timer0).
  waitMs(500);

  loadConfig();

  // DMXReceiver: interrupt-driven, completely non-blocking.
  // (DMXProbe uses delay() internally — do not use it.)
  DMXSerial.init(DMXReceiver);

  // I2C + LCD
  Wire.begin();
  Wire.setWireTimeout(10000, true);  // 10 ms timeout; auto-resets TWI on hang
  lcd.init();
  lcd.backlight();

  drawHome();
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
  // If Wire timed out (I2C hang), re-initialise the LCD.
  if (Wire.getWireTimeoutFlag()) {
    Wire.clearWireTimeoutFlag();
    Wire.begin();
    Wire.setWireTimeout(10000, true);
    lcd.init();
    lcd.backlight();
    if (screen == HOME) drawHome(); else drawSettings();
  }

  // DMX → relays (non-blocking — DMXReceiver runs in background ISR).
  if (DMXSerial.noDataSince() < 1000) {
    applyRelays();
  } else {
    allRelaysOff();
  }

  // ── UI ──────────────────────────────────────────────────────────────────
  if (screen == HOME) {
    // Refresh relay/DMX status line at most every 200 ms.
    unsigned long now = millis();
    if (now - lastHomeUpdate >= 200) {
      lastHomeUpdate = now;
      updateHome();
    }

    if (pressed(BTN_ENTER)) {
      screen = SETTINGS;
      settingIdx = 0;
      drawSettings();
    }

  } else {  // SETTINGS

    if (pressed(BTN_UP)) {
      if (settingIdx == 0) { if (dmxAddress < 505) dmxAddress++; }
      else                 { if (threshold  < 255) threshold++;  }
      drawSettings();
    }
    if (pressed(BTN_DOWN)) {
      if (settingIdx == 0) { if (dmxAddress > 1) dmxAddress--; }
      else                 { if (threshold  > 1) threshold--;   }
      drawSettings();
    }
    if (pressed(BTN_ENTER)) {
      if (settingIdx < 1) {
        settingIdx++;
        drawSettings();
      } else {
        saveConfig();
        screen = HOME;
        drawHome();
      }
    }
    if (pressed(BTN_BACK)) {
      saveConfig();
      screen = HOME;
      drawHome();
    }
  }
}
