#include <DMXSerial.h>
#include <Wire.h>
#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>

// ================= LCD =================
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ================= Hardware =================
const uint8_t relayPins[8] = {4, 5, 6, 7, 8, 9, 10, 16};

const uint8_t BTN_UP    = A0;
const uint8_t BTN_DOWN  = A1;
const uint8_t BTN_ENTER = A2;
const uint8_t BTN_BACK  = A3;

const bool RELAY_ACTIVE_LOW = true;

// ================= Behaviour =================
uint16_t dmxStart = 1;

uint8_t  ON_THRESHOLD  = 128;
uint8_t  OFF_THRESHOLD = 120;

// DMX probe + presence
uint16_t DMX_RECEIVE_WAIT_MS = 30;   // how long each probe waits for a packet
uint16_t DMX_TIMEOUT_MS      = 1000; // declare NO DMX if no valid packet for this long
bool FAILSAFE_ALL_OFF        = true; // true: all off on NO DMX; false: hold last

bool dmxPresent = false;
unsigned long lastGoodDmxMs = 0;

// ================= EEPROM =================
struct Config {
  uint16_t magic;
  uint16_t startAddr;
  uint8_t  onTh;
  uint8_t  offTh;
  uint16_t rxWaitMs;
  uint16_t timeoutMs;
  uint8_t  flags; // bit0 = failsafeAllOff
};

const uint16_t CFG_MAGIC = 0xBEEF;
const int EEPROM_ADDR = 0;

bool pendingSave = false;
unsigned long lastChangeMs = 0;
const unsigned long SAVE_DELAY_MS = 1500;

// ================= UI =================
enum Screen { HOME, MENU, MANUAL, SETTINGS };
Screen screen = HOME;
uint8_t menuIndex = 0;
uint8_t manualIndex = 0;

bool manualOverride = false;
bool manualRelayState[8] = {0};

// relay hysteresis shadow
bool relayState[8] = {0};

// ================= Button debounce =================
struct DebBtn {
  uint8_t pin;
  uint8_t lastStable;
  uint8_t lastRead;
  unsigned long lastFlipMs;
};

DebBtn btns[4] = {
  {BTN_UP,    HIGH, HIGH, 0},
  {BTN_DOWN,  HIGH, HIGH, 0},
  {BTN_ENTER, HIGH, HIGH, 0},
  {BTN_BACK,  HIGH, HIGH, 0}
};

bool buttonPressed(uint8_t pin) {
  const unsigned long DEBOUNCE_MS = 25;

  for (uint8_t i = 0; i < 4; i++) {
    if (btns[i].pin != pin) continue;

    uint8_t r = digitalRead(pin); // HIGH idle, LOW pressed

    if (r != btns[i].lastRead) {
      btns[i].lastRead = r;
      btns[i].lastFlipMs = millis();
    }

    if ((millis() - btns[i].lastFlipMs) > DEBOUNCE_MS && r != btns[i].lastStable) {
      btns[i].lastStable = r;
      if (btns[i].lastStable == LOW) return true; // just pressed
    }
  }
  return false;
}

// ================= Helpers =================
void setRelay(uint8_t i, bool on) {
  if (RELAY_ACTIVE_LOW) digitalWrite(relayPins[i], on ? LOW : HIGH);
  else                  digitalWrite(relayPins[i], on ? HIGH : LOW);
}

void allRelaysOff() {
  for (uint8_t i = 0; i < 8; i++) setRelay(i, false);
}

void markChanged() {
  pendingSave = true;
  lastChangeMs = millis();
}

void clampStartAddr() {
  if (dmxStart < 1) dmxStart = 1;
  if (dmxStart > 505) dmxStart = 505;
}

void loadConfig() {
  Config c;
  EEPROM.get(EEPROM_ADDR, c);

  if (c.magic == CFG_MAGIC) {
    dmxStart = c.startAddr;
    ON_THRESHOLD = c.onTh;
    OFF_THRESHOLD = c.offTh;
    DMX_RECEIVE_WAIT_MS = c.rxWaitMs;
    DMX_TIMEOUT_MS = c.timeoutMs;
    FAILSAFE_ALL_OFF = (c.flags & 0x01);
  }

  clampStartAddr();

  if (ON_THRESHOLD < 1) ON_THRESHOLD = 128;
  if (OFF_THRESHOLD > ON_THRESHOLD) OFF_THRESHOLD = (ON_THRESHOLD > 8) ? (ON_THRESHOLD - 8) : 0;

  if (DMX_RECEIVE_WAIT_MS < 5) DMX_RECEIVE_WAIT_MS = 5;
  if (DMX_RECEIVE_WAIT_MS > 80) DMX_RECEIVE_WAIT_MS = 80;

  if (DMX_TIMEOUT_MS < 100) DMX_TIMEOUT_MS = 100;
  if (DMX_TIMEOUT_MS > 10000) DMX_TIMEOUT_MS = 10000;
}

void saveConfigNow() {
  Config c;
  c.magic = CFG_MAGIC;
  c.startAddr = dmxStart;
  c.onTh = ON_THRESHOLD;
  c.offTh = OFF_THRESHOLD;
  c.rxWaitMs = DMX_RECEIVE_WAIT_MS;
  c.timeoutMs = DMX_TIMEOUT_MS;
  c.flags = (FAILSAFE_ALL_OFF ? 0x01 : 0x00);

  EEPROM.put(EEPROM_ADDR, c);
  pendingSave = false;
}

// ================= DMX (Probe mode) =================
// DMXProbe + receive(timeout) waits for a packet and returns true/false. [3](https://docs.arduino.cc/libraries/arduinodmx/)[1](https://github.com/mathertel/DMXSerial/blob/master/src/DMXSerial.h)
void pollDmx() {
  bool gotPacket = DMXSerial.receive((uint8_t)DMX_RECEIVE_WAIT_MS); // [1](https://github.com/mathertel/DMXSerial/blob/master/src/DMXSerial.h)

  if (gotPacket) {
    lastGoodDmxMs = millis();
    dmxPresent = true;
  } else {
    dmxPresent = (millis() - lastGoodDmxMs) < DMX_TIMEOUT_MS;
  }
}

void applyDmxToRelays() {
  for (uint8_t i = 0; i < 8; i++) {
    uint8_t v = DMXSerial.read(dmxStart + i); // [1](https://github.com/mathertel/DMXSerial/blob/master/src/DMXSerial.h)

    if (!relayState[i] && v >= ON_THRESHOLD)  relayState[i] = true;
    if ( relayState[i] && v <= OFF_THRESHOLD) relayState[i] = false;

    setRelay(i, relayState[i]);
  }
}

void applyManualRelays() {
  for (uint8_t i = 0; i < 8; i++) setRelay(i, manualRelayState[i]);
}

// ================= LCD screens =================
void drawHome() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Addr:");
  if (dmxStart < 100) lcd.print('0');
  if (dmxStart < 10)  lcd.print('0');
  lcd.print(dmxStart);
  lcd.print(manualOverride ? " MAN" : " DMX");

  lcd.setCursor(0,1);
  for (uint8_t i = 0; i < 8; i++) {
    bool on = manualOverride ? manualRelayState[i] : relayState[i];
    lcd.print(on ? (char)('1' + i) : '-');
  }
}

void drawMenu() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Menu:");
  lcd.setCursor(0,1);
  if (menuIndex == 0) lcd.print(">Manual Control");
  else                lcd.print(">Settings");
}

void drawManual() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Relay ");
  lcd.print(manualIndex + 1);
  lcd.print(": ");
  lcd.print(manualRelayState[manualIndex] ? "ON " : "OFF");

  lcd.setCursor(0,1);
  lcd.print("Up/Dn Sel EntTgl");
}

void drawSettings() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Thr ");
  lcd.print(ON_THRESHOLD);
  lcd.print(" W");
  lcd.print(DMX_RECEIVE_WAIT_MS);

  lcd.setCursor(0,1);
  lcd.print("To ");
  lcd.print(DMX_TIMEOUT_MS);
  lcd.print(FAILSAFE_ALL_OFF ? " FS:OFF" : " FS:HLD");
}

// ================= SETUP / LOOP =================
void setup() {
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_ENTER, INPUT_PULLUP);
  pinMode(BTN_BACK, INPUT_PULLUP);

  for (uint8_t i = 0; i < 8; i++) {
    pinMode(relayPins[i], OUTPUT);
    setRelay(i, false);
  }

  loadConfig();

  // IMPORTANT: init DMX first (prevents your earlier hang). [2](https://flexpcb.org/arduino-pro-micro-pinout-connection-pins-for-the-atmega32u4-based-microcontroller/)[1](https://github.com/mathertel/DMXSerial/blob/master/src/DMXSerial.h)
  // Use DMXProbe mode so we can call receive(timeout) explicitly. [3](https://docs.arduino.cc/libraries/arduinodmx/)[1](https://github.com/mathertel/DMXSerial/blob/master/src/DMXSerial.h)
  DMXSerial.init(DMXProbe);

  // Then LCD/I2C
  Wire.begin();
  lcd.init();
  lcd.backlight();

  lastGoodDmxMs = 0;
  dmxPresent = false;

  drawHome();
}

void loop() {
  // Poll DMX (blocks for DMX_RECEIVE_WAIT_MS max)
  pollDmx();

  // Save config after idle
  if (pendingSave && (millis() - lastChangeMs) > SAVE_DELAY_MS) {
    saveConfigNow();
  }

  // -------- UI --------
  if (screen == HOME) {
    if (!manualOverride) {
      if (buttonPressed(BTN_UP)) {
        if (dmxStart < 505) { dmxStart++; markChanged(); }
        drawHome();
      }
      if (buttonPressed(BTN_DOWN)) {
        if (dmxStart > 1) { dmxStart--; markChanged(); }
        drawHome();
      }
    }

    if (buttonPressed(BTN_ENTER)) {
      screen = MENU;
      menuIndex = 0;
      drawMenu();
    }

    if (buttonPressed(BTN_BACK)) {
      manualOverride = !manualOverride;
      drawHome();
    }
  }
  else if (screen == MENU) {
    if (buttonPressed(BTN_UP) || buttonPressed(BTN_DOWN)) {
      menuIndex = (menuIndex == 0) ? 1 : 0;
      drawMenu();
    }

    if (buttonPressed(BTN_ENTER)) {
      if (menuIndex == 0) {
        screen = MANUAL;
        manualIndex = 0;
        manualOverride = true;
        drawManual();
      } else {
        screen = SETTINGS;
        drawSettings();
      }
    }

    if (buttonPressed(BTN_BACK)) {
      screen = HOME;
      drawHome();
    }
  }
  else if (screen == MANUAL) {
    if (buttonPressed(BTN_UP)) {
      manualIndex = (manualIndex == 0) ? 7 : (manualIndex - 1);
      drawManual();
    }
    if (buttonPressed(BTN_DOWN)) {
      manualIndex = (manualIndex + 1) % 8;
      drawManual();
    }
    if (buttonPressed(BTN_ENTER)) {
      manualRelayState[manualIndex] = !manualRelayState[manualIndex];
      drawManual();
    }
    if (buttonPressed(BTN_BACK)) {
      screen = HOME;
      drawHome();
    }
  }
  else if (screen == SETTINGS) {
    // UP/DOWN adjusts threshold; ENTER toggles failsafe; BACK returns
    if (buttonPressed(BTN_UP)) {
      if (ON_THRESHOLD <= 250) ON_THRESHOLD += 5;
      OFF_THRESHOLD = (ON_THRESHOLD > 8) ? (ON_THRESHOLD - 8) : 0;
      markChanged();
      drawSettings();
    }
    if (buttonPressed(BTN_DOWN)) {
      if (ON_THRESHOLD >= 5) ON_THRESHOLD -= 5;
      OFF_THRESHOLD = (ON_THRESHOLD > 8) ? (ON_THRESHOLD - 8) : 0;
      markChanged();
      drawSettings();
    }
    if (buttonPressed(BTN_ENTER)) {
      FAILSAFE_ALL_OFF = !FAILSAFE_ALL_OFF;
      markChanged();
      drawSettings();
    }
    if (buttonPressed(BTN_BACK)) {
      screen = HOME;
      drawHome();
    }
  }

  // -------- Outputs --------
  if (manualOverride) {
    applyManualRelays();
  } else {
    if (!dmxPresent) {
      if (FAILSAFE_ALL_OFF) allRelaysOff();
      // else HOLD: do nothing
    } else {
      applyDmxToRelays();
    }
  }

  delay(5);
}
