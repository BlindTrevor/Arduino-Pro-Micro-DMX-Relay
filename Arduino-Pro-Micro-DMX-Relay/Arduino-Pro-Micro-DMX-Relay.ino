// =============================================================================
// Arduino UNO — DMX Relay Controller
// =============================================================================
// Receives DMX512 and drives 8 relays.
// DMX start address, on-threshold, and no-DMX behaviour are adjustable via
// 4 buttons + 16×2 LCD.  Settings save to EEPROM when returning to home.
//
// Pinout
//   DMX input     : pin 0 (RX) via MAX485, RE+DE tied LOW to GND
//                   pin 1 (TX) — leave unconnected or tie HIGH via 10k
//   Relay 1–8     : pins 2, 3, 4, 5, 6, 7, 8, 9  (active-LOW relay module)
//   Button UP     : pin A0  (other leg to GND)
//   Button DOWN   : pin A1  (other leg to GND)
//   Button ENTER  : pin A2  (other leg to GND)
//   Button BACK   : pin A3  (other leg to GND)
//   LCD SDA       : pin A4  (I2C, 16×2 LCD with PCF8574 backpack at 0x27)
//   LCD SCL       : pin A5  (I2C)
//   LCD 5 V / GND : 5 V and GND rails
//
// NOTE: pins 0 and 1 are used exclusively by DMX (hardware Serial).
//       Do NOT connect anything else to them while DMX is active.
//       Disconnect the MAX485 before uploading new sketches via USB.
//
// UI
//   Home screen : shows address, relay states, and DMX status.
//   ENTER       : open settings.
//   Settings    : cursor selects Address, Threshold, or No DMX row.
//                 UP/DOWN change the value.  Hold UP/DOWN to change faster:
//                   tap = one step; hold >1 s = slow repeat; hold >2 s = fast repeat.
//                 ENTER moves to the next item (or saves+exits on the last one).
//                 BACK  saves and returns to home immediately.
//   No DMX      : OFF = all relays off when DMX signal is lost.
//                 HOLD = relays keep their last state when DMX is lost.
// =============================================================================

#include <DMXSerial.h>
#include <Wire.h>
#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>
#include <util/delay.h>

// _delay_ms() uses CPU cycle-counting and is immune to any Timer0 jitter.
// Safer than delay() during setup() even on UNO.
static void waitMs(uint16_t ms) { while (ms--) _delay_ms(1); }

// ── Hardware ─────────────────────────────────────────────────────────────────
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Relay outputs: pins 2–9.  Pins 0 and 1 are reserved for DMX (Serial).
const uint8_t RELAY_PINS[8] = {2, 3, 4, 5, 6, 7, 8, 9};
const uint8_t BTN_UP    = A0;
const uint8_t BTN_DOWN  = A1;
const uint8_t BTN_ENTER = A2;
const uint8_t BTN_BACK  = A3;

// ── Config ───────────────────────────────────────────────────────────────────
struct Config { uint16_t magic; uint16_t address; uint8_t threshold; uint8_t noDmxBehavior; };
const uint16_t MAGIC   = 0xDA7A;
const int      EE_ADDR = 0;

uint16_t dmxAddress     = 1;    // DMX start channel, 1–505
uint8_t  threshold      = 128;  // relay ON  when value >= threshold
                                 // relay OFF when value <  (threshold - 8)  [hysteresis]
uint8_t  noDmxBehavior  = 0;    // 0 = OFF (all relays off when no DMX), 1 = HOLD (keep last state)

void loadConfig() {
  Config c;
  EEPROM.get(EE_ADDR, c);
  if (c.magic == MAGIC) {
    dmxAddress    = constrain((uint16_t)c.address,       (uint16_t)1, (uint16_t)505);
    threshold     = constrain((uint8_t) c.threshold,     (uint8_t) 1, (uint8_t) 255);
    noDmxBehavior = (c.noDmxBehavior <= 1) ? c.noDmxBehavior : 0;
  }
}

void saveConfig() {
  Config c = {MAGIC, dmxAddress, threshold, noDmxBehavior};
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
  unsigned long pressedAt;   // millis() when button went stably LOW; 0 if not held
  unsigned long lastRepeat;  // millis() of last auto-repeat fire
};

Button btns[4] = {
  {BTN_UP,    HIGH, HIGH, 0, 0, 0},
  {BTN_DOWN,  HIGH, HIGH, 0, 0, 0},
  {BTN_ENTER, HIGH, HIGH, 0, 0, 0},
  {BTN_BACK,  HIGH, HIGH, 0, 0, 0},
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

// Fires on the initial press, then auto-repeats while held with an accelerating
// rate: slow repeat (every 150 ms) after 1 s, fast repeat (every 40 ms) after 2 s.
bool held(uint8_t pin) {
  for (uint8_t i = 0; i < 4; i++) {
    if (btns[i].pin != pin) continue;
    uint8_t r = digitalRead(pin);
    if (r != btns[i].lastRead) { btns[i].lastRead = r; btns[i].lastFlip = millis(); }
    if (millis() - btns[i].lastFlip <= 25) return false;  // still bouncing
    unsigned long now = millis();
    if (r == HIGH) {
      btns[i].lastStable = HIGH;
      btns[i].pressedAt  = 0;
      return false;
    }
    // Button is stably LOW.
    if (btns[i].lastStable == HIGH) {
      // Initial press — fire once and start hold timer.
      btns[i].lastStable = LOW;
      btns[i].pressedAt  = now;
      btns[i].lastRepeat = now;
      return true;
    }
    // Auto-repeat: interval shrinks the longer the button is held.
    unsigned long holdDur  = now - btns[i].pressedAt;
    uint16_t      interval = (holdDur < 1000) ? 0    // no repeat in first second
                           : (holdDur < 2000) ? 150  // slow: ~6 steps/s
                           :                    40;   // fast: 25 steps/s
    if (interval && (now - btns[i].lastRepeat >= interval)) {
      btns[i].lastRepeat = now;
      return true;
    }
    return false;
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
// Row 0: "Address:001     "   (16 chars)
// Row 1: "12345678  DMX   "   (relay pattern + DMX status)

void drawHome() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Address:"); lcdNum(dmxAddress, 3);
  lcd.print("     ");
}

void updateHome() {
  bool dmxOk = (DMXSerial.noDataSince() < 1000);
  lcd.setCursor(0, 1);
  for (uint8_t i = 0; i < 8; i++) lcd.print(relayOn[i] ? char('1' + i) : '-');
  lcd.print(dmxOk ? "  DMX OK" : "  NO DMX");
}

// ── Settings screen ───────────────────────────────────────────────────────────
// Three settings scroll through a 2-row window:
//   settingIdx 0 or 1 : row0 = Address, row1 = Threshold
//   settingIdx 2       : row0 = Threshold, row1 = No DMX
//
// Row format: ">Address:001    " / " Address:001    "
//             ">Thr:128        " / " Thr:128        "
//             ">No DMX:HOLD    " / " No DMX:OFF     "

uint8_t settingIdx = 0;   // 0 = address, 1 = threshold, 2 = no-DMX behaviour

void drawSettings() {
  lcd.clear();

  // Determine which two items occupy row 0 and row 1.
  // Window shifts down when settingIdx reaches 2.
  uint8_t topItem = (settingIdx <= 1) ? 0 : 1;   // 0=Address,1=Threshold,2=NoDMX

  for (uint8_t row = 0; row < 2; row++) {
    uint8_t item = topItem + row;
    bool    sel  = (item == settingIdx);
    lcd.setCursor(0, row);
    lcd.print(sel ? '>' : ' ');

    if (item == 0) {
      lcd.print("Address:"); lcdNum(dmxAddress, 3);
      lcd.print("    ");
    } else if (item == 1) {
      lcd.print("Thr:"); lcdNum(threshold, 3);
      lcd.print("        ");
    } else {
      lcd.print("No DMX:");
      lcd.print(noDmxBehavior ? "HOLD    " : "OFF     ");
    }
  }
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

  // Wait 500 ms for PSU and LCD backpack to stabilise.
  waitMs(500);

  loadConfig();

  // DMXReceiver: interrupt-driven on the hardware Serial (pins 0/1).
  DMXSerial.init(DMXReceiver);

  // I2C + LCD
  Wire.begin();
  Wire.setWireTimeout(10000, true);  // 10 ms timeout; auto-resets TWI on hang
  lcd.init();
  lcd.backlight();

  // Splash screen — shown for 1.5 s on power-up.
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(" Sonic Lighting ");
  lcd.setCursor(0, 1); lcd.print("Relay Controller");
  waitMs(1500);

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
  } else if (noDmxBehavior == 0) {
    allRelaysOff();
  }
  // noDmxBehavior == 1 (HOLD): do nothing — relays keep their last state.

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

    if (held(BTN_UP)) {
      if (settingIdx == 0) { if (dmxAddress < 505) dmxAddress++; }
      else if (settingIdx == 1) { if (threshold  < 255) threshold++;  }
      else                      { noDmxBehavior = 1; }
      drawSettings();
    }
    if (held(BTN_DOWN)) {
      if (settingIdx == 0) { if (dmxAddress > 1) dmxAddress--; }
      else if (settingIdx == 1) { if (threshold  > 1) threshold--;   }
      else                      { noDmxBehavior = 0; }
      drawSettings();
    }
    if (pressed(BTN_ENTER)) {
      if (settingIdx < 2) {
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
