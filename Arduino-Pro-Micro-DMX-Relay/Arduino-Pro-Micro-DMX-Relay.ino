#include <DMXSerial.h>
#include <Wire.h>
#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>

// ================= LCD =================
LiquidCrystal_I2C lcd(0x27, 16, 2);
const uint8_t LCD_COLS = 16;

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

// Measured wall-clock time the last pollDmx() call took (ms).
// Displayed on the home screen so slow blocking is immediately visible.
unsigned long lastPollMs = 0;

// Timestamp of the last updateHomeDiagLine() LCD write.
// The diag line is throttled to 500 ms to avoid flooding the I2C bus —
// each unthrottled call does ~40 I2C transactions which, if Wire timeouts
// are firing, can block for hundreds of ms and make the loop appear frozen.
unsigned long lastDiagMs = 0;

// Timestamp of the last updateHomeRelayLine() LCD write.
// Throttled to 200 ms: if relay states toggle rapidly (e.g. dmxPresent
// flickering due to a bad EEPROM timeout value), the ~36 I2C transactions
// per call would flood the bus and compound the slowness every loop iteration.
unsigned long lastRelayLineMs = 0;

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

// last relay states drawn on the home screen (for change detection)
bool lastDrawnRelayState[8] = {0};

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

  if (DMX_RECEIVE_WAIT_MS < 5)  DMX_RECEIVE_WAIT_MS = 5;
  // Cap at 30 ms: a higher value blocks loop() for that long whenever a DMX
  // packet doesn't arrive in the poll window, starving button reads and making
  // the heartbeat crawl.  Any EEPROM value > 30 (e.g. the old 80 ms max) is
  // silently clamped here so it self-corrects on the next config save.
  if (DMX_RECEIVE_WAIT_MS > 30) DMX_RECEIVE_WAIT_MS = 30;

  // Enforce that the DMX-loss timeout is large enough that a handful of missed
  // poll windows doesn't falsely declare "no DMX" and trigger the failsafe.
  // Minimum = 5× the poll wait (covers 4 consecutive misses) but never < 500 ms.
  {
    uint16_t minTimeout = (uint16_t)DMX_RECEIVE_WAIT_MS * 5;
    if (minTimeout < 500) minTimeout = 500;
    if (DMX_TIMEOUT_MS < minTimeout) DMX_TIMEOUT_MS = minTimeout;
  }
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
void lcdPrintCentered(uint8_t row, const char* str) {
  uint8_t len = strlen(str);
  uint8_t col = (len < LCD_COLS) ? ((LCD_COLS - len) / 2) : 0;
  lcd.setCursor(col, row);
  lcd.print(str);
}

bool activeRelayState(uint8_t i) {
  return manualOverride ? manualRelayState[i] : relayState[i];
}

void updateHomeRelayLine() {
  // Throttle to 200 ms using the global lastRelayLineMs so drawHome() can
  // reset it to force an immediate paint on screen transitions.
  unsigned long now = millis();
  if (now - lastRelayLineMs < 200) return;
  lastRelayLineMs = now;
  lcd.setCursor(4, 1);  // 8 relay chars centered: (LCD_COLS-8)/2 = 4
  for (uint8_t i = 0; i < 8; i++) {
    bool on = activeRelayState(i);
    lcd.print(on ? (char)('1' + i) : '-');
    lastDrawnRelayState[i] = on;
  }
}

// Diagnostic line (row 1): real poll time left of relays, configured wait right.
//   Cols 0-3  "P:XX"  – actual measured pollDmx() duration in ms; "P:HI" if > 99 ms
//   Cols 4-11          – relay state chars (written by updateHomeRelayLine)
//   Cols 12-15 "W:XX"  – configured DMX_RECEIVE_WAIT_MS (confirms EEPROM value)
// If P:XX matches W:XX the library is behaving normally.
// If P:XX >> W:XX (or shows P:HI) DMXSerial.receive() is blocking much longer than expected.
void updateHomeDiagLine() {
  if (screen != HOME) return;
  // Throttle to 500 ms: each call performs ~40 I2C transactions; without
  // throttling, Wire timeouts (10 ms each) can add hundreds of ms per loop.
  unsigned long now = millis();
  if (now - lastDiagMs < 500) return;
  lastDiagMs = now;
  char buf[5];
  lcd.setCursor(0, 1);
  if (lastPollMs > 99) {
    lcd.print("P:HI");
  } else {
    snprintf(buf, sizeof(buf), "P:%02u", (unsigned)lastPollMs);
    lcd.print(buf);
  }
  lcd.setCursor(12, 1);
  snprintf(buf, sizeof(buf), "W:%02u", (unsigned)DMX_RECEIVE_WAIT_MS);
  lcd.print(buf);
}

void drawHome() {
  lcd.clear();
  lcd.setCursor(2, 0);  // "Addr:XXX DMX" = 12 chars centered: (LCD_COLS-12)/2 = 2
  lcd.print("Addr:");
  if (dmxStart < 100) lcd.print('0');
  if (dmxStart < 10)  lcd.print('0');
  lcd.print(dmxStart);
  lcd.print(manualOverride ? " MAN" : " DMX");
  // Cols 14-15 are the heartbeat area; clear them so updateHeartbeat() starts fresh
  lcd.setCursor(14, 0);
  lcd.print("  ");

  // Reset throttle timers so both sub-functions paint immediately on a full
  // redraw rather than being suppressed by the in-progress timer intervals.
  lastDiagMs = 0;
  lastRelayLineMs = 0;
  updateHomeDiagLine();
  updateHomeRelayLine();
}

// Boot splash: displayed briefly after LCD init so that a cold-PSU boot can be
// diagnosed visually.
//   Row 0: "Rly:OK  DMX:W30 " – relay init and configured DMX wait time
//   Row 1: "LCD:A1  Wire:OK " – how many lcd.init() attempts were needed, and
//           whether any I2C timeout occurred.  "A1" means first attempt worked.
//           "A3" and "Wire:ERR" means it took 3 attempts and still has errors.
//           Seeing A>1 on a cold boot confirms the PCF8574 power-on timing issue.
void drawBootSplash(uint8_t lcdAttempts, bool wireErr) {
  char buf[LCD_COLS + 1];
  lcd.clear();
  // Row 0: relay pin setup + DMX serial init + configured wait time
  snprintf(buf, sizeof(buf), "Rly:OK  DMX:W%02u ", (unsigned)DMX_RECEIVE_WAIT_MS);
  lcd.setCursor(0, 0);
  lcd.print(buf);
  // Row 1: LCD init attempt count + Wire timeout flag
  snprintf(buf, sizeof(buf), "LCD:A%u  Wire:%s  ",
           (unsigned)lcdAttempts, wireErr ? "ERR" : "OK ");
  buf[LCD_COLS] = '\0';
  lcd.setCursor(0, 1);
  lcd.print(buf);
}

void drawMenu() {
  lcd.clear();
  lcdPrintCentered(0, "Menu:");
  if (menuIndex == 0) lcdPrintCentered(1, ">Manual Control");
  else                lcdPrintCentered(1, ">Settings");
}

void drawManual() {
  char buf[LCD_COLS + 1];
  snprintf(buf, sizeof(buf), "Relay %d: %s", manualIndex + 1,
           manualRelayState[manualIndex] ? "ON " : "OFF");
  lcd.clear();
  lcdPrintCentered(0, buf);
  lcdPrintCentered(1, "Up/Dn Sel EntTgl");
}

void drawSettings() {
  char buf[LCD_COLS + 1];
  lcd.clear();
  snprintf(buf, sizeof(buf), "Thr %d W%d", ON_THRESHOLD, DMX_RECEIVE_WAIT_MS);
  lcdPrintCentered(0, buf);
  snprintf(buf, sizeof(buf), "To %d %s", DMX_TIMEOUT_MS,
           FAILSAFE_ALL_OFF ? "FS:OFF" : "FS:HLD");
  lcdPrintCentered(1, buf);
}

// ================= Heartbeat (diagnostic) =================
// A spinning char in col 15, row 0 of the home screen proves loop() is alive.
// Col 14 shows 'E' if the Wire library's timeout flag has been set, meaning at
// least one I2C transfer timed out (LCD commands may be silently failing).
const char SPINNER[] = {'-', '\\', '|', '/'};
uint8_t       heartbeatTick    = 0;
unsigned long lastHeartbeatMs  = 0;

void updateHeartbeat() {
  if (screen != HOME) return;
  unsigned long now = millis();
  if (now - lastHeartbeatMs < 100) return;  // 100 ms gives ~10 steps/sec — much more readable
  lastHeartbeatMs = now;
  heartbeatTick = (heartbeatTick + 1) & 3;

  bool wireErr = Wire.getWireTimeoutFlag();
  lcd.setCursor(14, 0);
  lcd.print(wireErr ? 'E' : ' ');
  lcd.setCursor(15, 0);
  lcd.print(SPINNER[heartbeatTick]);
}

// ================= I2C + LCD initialisation (shared helper) =================
// Encapsulates the full sequence: bus recovery → Wire.begin() →
// Wire.setWireTimeout() → lcd.init() → lcd.backlight().
// Clears the Wire timeout flag before the attempt and returns true if the
// attempt completed without a timeout (i.e. LCD is now responsive).
// Called from setup() (possibly multiple times) and from the runtime recovery
// path in loop().
bool initI2cAndLcd() {
  Wire.clearWireTimeoutFlag();
  recoverI2cBus();
  Wire.begin();
  // 10 ms I2C timeout — prevents any single transaction from hanging the
  // firmware forever.  reset_with_timeout=true auto-resets the TWI hardware
  // when a timeout fires, which is necessary to recover the master state
  // machine.  The downside is the PCF8574 may be confused afterwards, which
  // is why we check the flag and retry / re-init when it is set.
  Wire.setWireTimeout(10000, true);
  lcd.init();
  lcd.backlight();
  return !Wire.getWireTimeoutFlag();
}

// ================= I2C bus recovery =================
// Pro Micro I2C pins (ATmega32U4 hardware mapping)
const uint8_t I2C_SDA_PIN = 2;
const uint8_t I2C_SCL_PIN = 3;

// I2C recovery timing constants (100 kHz bus → half-period ≈ 5 µs)
const uint8_t I2C_RECOVERY_CLOCKS    = 9;  // max clocks needed to free a stuck slave
const uint8_t I2C_RECOVERY_HALF_US   = 5;  // half-period in microseconds

// Send 9 SCL clock pulses unconditionally to clock out any partial byte a
// slave may be mid-transmitting, then issue a STOP condition.  Run this
// before Wire.begin() so the bus is always in a known IDLE state regardless
// of what happened at power-on (the early-return-on-SDA-HIGH heuristic was
// not reliable enough — SDA can be HIGH mid-transaction).
void recoverI2cBus() {
  // Float both lines first so we can sense the bus without fighting any slave
  pinMode(I2C_SCL_PIN, INPUT);
  pinMode(I2C_SDA_PIN, INPUT);
  delayMicroseconds(I2C_RECOVERY_HALF_US);

  // Clock out up to 9 pulses — enough to complete any partial byte
  pinMode(I2C_SCL_PIN, OUTPUT);
  for (uint8_t i = 0; i < I2C_RECOVERY_CLOCKS; i++) {
    digitalWrite(I2C_SCL_PIN, HIGH);
    delayMicroseconds(I2C_RECOVERY_HALF_US);
    digitalWrite(I2C_SCL_PIN, LOW);
    delayMicroseconds(I2C_RECOVERY_HALF_US);
  }

  // Issue a STOP condition (SDA LOW→HIGH while SCL HIGH)
  pinMode(I2C_SDA_PIN, OUTPUT);
  digitalWrite(I2C_SDA_PIN, LOW);
  delayMicroseconds(I2C_RECOVERY_HALF_US);
  digitalWrite(I2C_SCL_PIN, HIGH);
  delayMicroseconds(I2C_RECOVERY_HALF_US);
  digitalWrite(I2C_SDA_PIN, HIGH);
  delayMicroseconds(I2C_RECOVERY_HALF_US);
}

// ================= SETUP / LOOP =================
void setup() {
  // ── Relay pins: drive HIGH before anything else ─────────────────────────────
  // Relay pins float as inputs (high-Z) from reset.  On an active-LOW relay
  // module a floating input can pull the coil down and briefly energise the
  // relay before setup() configures the pin.  Drive OUTPUT HIGH immediately so
  // the module stays de-energised throughout the rest of initialisation.
  for (uint8_t i = 0; i < 8; i++) {
    pinMode(relayPins[i], OUTPUT);
    setRelay(i, false);
  }

  // ── Startup delay ───────────────────────────────────────────────────────────
  // When powered from an external 5 V supply the ATmega32U4 boots immediately
  // on power-on reset (the caterina bootloader skips the 8-second USB window
  // for PORF resets).  An Arduino IDE upload, by contrast, triggers a DTR
  // reset: the bootloader runs for ~8 seconds giving all peripherals ample
  // time to stabilise before the sketch starts — which is exactly why the
  // system "works after upload but fails after power cycle."
  // A 2-second delay here gives the external PSU, relay module, LCD backpack
  // (PCF8574), and MAX485 time to reach stable operating voltage before the
  // sketch tries to communicate with them.
  // BTN_ENTER is pulled up before the delay so the factory-reset check works.
  pinMode(BTN_ENTER, INPUT_PULLUP);
  delay(2000);

  pinMode(BTN_UP,   INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_BACK, INPUT_PULLUP);

  loadConfig();

  // IMPORTANT: init DMX first (prevents your earlier hang).
  // Use DMXProbe mode so we can call receive(timeout) explicitly.
  DMXSerial.init(DMXProbe);

  // ── I2C + LCD init with cold-boot retry ────────────────────────────────────
  // On a cold power-cycle the PCF8574 LCD backpack may not respond on the
  // first attempt (supply still stabilising, or a stray I2C glitch).  When
  // Wire.setWireTimeout fires and resets the TWI hardware, the PCF8574 is
  // left in an undefined state.  Every subsequent LCD call then times out at
  // 10 ms each — 36 calls × 10 ms = 360 ms of blocking per relay-line update,
  // making the loop appear frozen and buttons completely unresponsive.
  // We retry the full init sequence (bus-recovery → Wire.begin → lcd.init)
  // until it succeeds or we exhaust our attempts, backing off between tries.
  uint8_t lcdInitAttempts = 0;
  bool    lcdOk           = false;
  while (!lcdOk && lcdInitAttempts < 5) {
    lcdOk = initI2cAndLcd();
    lcdInitAttempts++;
    if (!lcdOk) delay(500);  // back off before retrying
  }

  // Show boot splash: attempt count + Wire status tells us whether cold-boot
  // I2C instability occurred (A>1 or Wire:ERR on a cold start is diagnostic).
  drawBootSplash(lcdInitAttempts, !lcdOk);

  // ── Factory reset ─────────────────────────────────────────────────────────
  // Hold ENTER during the splash to wipe EEPROM and restore firmware defaults.
  // This recovers from bad EEPROM values (e.g. W:80 blocking the loop) without
  // needing to reflash.  The splash stays on screen for 2 s so there is plenty
  // of time to release the button before normal operation begins.
  {
    unsigned long splashStart = millis();
    bool resetDone = false;
    while (millis() - splashStart < 2000) {
      if (!resetDone && digitalRead(BTN_ENTER) == LOW) {
        // Wipe the magic number so loadConfig() ignores the stored block.
        uint16_t blank = 0xFFFF;
        EEPROM.put(EEPROM_ADDR, blank);
        // Reload — no valid magic means all variables keep their firmware
        // defaults (dmxStart=1, DMX_RECEIVE_WAIT_MS=30, DMX_TIMEOUT_MS=1000 …)
        loadConfig();
        resetDone = true;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Factory Reset!  ");
        lcd.setCursor(0, 1);
        lcd.print("Defaults loaded ");
      }
      delay(10);
    }
  }

  // Seed lastGoodDmxMs to now so the DMX-timeout window is measured from
  // the moment the sketch is fully initialised, not from time zero.
  // If we left it at 0, the first call to pollDmx() with no signal would
  // evaluate (millis() – 0) < DMX_TIMEOUT_MS → true for the first second,
  // falsely reporting DMX present and calling applyDmxToRelays() with
  // all-zero channel values instead of honouring the failsafe setting.
  lastGoodDmxMs = millis();
  dmxPresent = false;

  drawHome();
}

void loop() {
  // ── Runtime I2C recovery ──────────────────────────────────────────────────
  // If Wire.setWireTimeout() fired since the last iteration, the TWI hardware
  // was auto-reset.  The PCF8574 may now be in a confused state, and every
  // subsequent LCD call will NAK/timeout at 10 ms each, stalling the loop for
  // hundreds of ms per update and making buttons appear dead.  Re-run the full
  // I2C + LCD init sequence before any LCD writes happen this iteration.
  if (Wire.getWireTimeoutFlag()) {
    if (initI2cAndLcd()) {
      drawHome();  // repaint screen only if re-init succeeded
    }
    // If re-init also failed, the flag is set again; we try again next loop.
  }

  // Poll DMX and measure how long it actually blocks.
  // lastPollMs is displayed on the home screen as "P:XX" so slow blocking is
  // immediately visible — if P:XX >> W:XX the DMXSerial library is not honouring
  // the receive timeout and is the root cause of slow loop / unresponsive buttons.
  {
    unsigned long t0 = millis();
    pollDmx();
    lastPollMs = millis() - t0;
  }

  // Save config after idle
  if (pendingSave && (millis() - lastChangeMs) > SAVE_DELAY_MS) {
    saveConfigNow();
  }

  // Heartbeat: spinning char in top-right of home screen proves loop() is alive
  updateHeartbeat();

  // Diagnostic line: update poll-time reading every loop so the value is fresh
  updateHomeDiagLine();

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

  // -------- Refresh home relay line if states changed --------
  if (screen == HOME) {
    for (uint8_t i = 0; i < 8; i++) {
      bool on = activeRelayState(i);
      if (on != lastDrawnRelayState[i]) {
        updateHomeRelayLine();
        break;
      }
    }
  }

  delay(5);
}
