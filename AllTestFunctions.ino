#include <Arduino.h>
#include <math.h>
#include <ctype.h>
#include <string.h>
#include <limits.h>

/*
  Project: Phys124 Computer Pianist - Unified Arduino Script
  Authors: Dylan Dahlke and Ryan Moore (original phase design)
  Generated: Feb 2026

  This single sketch includes all project functions as selectable modes:
    1) Solenoid Blink Test
    2) Comparator Tuner
    3) ISR Echo Test
    4) Solenoid One-Shot Test
    5) Latency Calibrator
    6) Variable Speed Verification
    7) Final 4-Lane Integration

  Open Serial Monitor at 115200 and type "help" for commands.
*/

// -----------------------------------------------------------------------------
// Hardware Pin Map
// -----------------------------------------------------------------------------
const uint8_t NUM_LANES = 4;
const uint8_t SOL_PINS[NUM_LANES] = {3, 5, 6, 9};
const uint8_t SENSOR_PINS[NUM_LANES] = {2, 4, 7, 8};

// -----------------------------------------------------------------------------
// User Calibration (used by Variable Speed + Final modes)
// -----------------------------------------------------------------------------
float gSensorDistMm = 40.0f;      // Distance from sensor to solenoid hit point
int gMechLagMs = 17;              // Measured mechanical lag
float gScrollSpeedMmPerMs = 0.30f; // Tile speed in mm/ms

// Final mode timing
unsigned long gFinalHoldMs = 25;
unsigned long gFinalCooldownMs = 80;
unsigned long gWatchdogMaxOnMs = 150;

// -----------------------------------------------------------------------------
// Timing constants for test modes
// -----------------------------------------------------------------------------
const unsigned long SOL_BLINK_ON_MS = 100;
const unsigned long SOL_BLINK_OFF_MS = 900;
const unsigned long COMP_TUNER_PRINT_MS = 50;
const unsigned long ONE_SHOT_HOLD_MS = 20;
const unsigned long LATENCY_TEST_DELAY_MS = 1000;
const unsigned long LATENCY_COOLDOWN_MS = 2000;
const unsigned long FINAL_ISR_DEBOUNCE_MS = 12;

// -----------------------------------------------------------------------------
// Runtime Mode Selection
// -----------------------------------------------------------------------------
enum RunMode {
  MODE_FINAL = 0,
  MODE_SOL_BLINK,
  MODE_COMP_TUNER,
  MODE_ISR_ECHO,
  MODE_ONESHOT,
  MODE_LATENCY,
  MODE_VAR_SPEED
};

volatile uint8_t gMode = MODE_FINAL;

// -----------------------------------------------------------------------------
// Final mode lane state machine
// -----------------------------------------------------------------------------
enum FinalState {
  FINAL_IDLE = 0,
  FINAL_WAITING,
  FINAL_FIRING,
  FINAL_COOLDOWN
};

struct FinalLane {
  volatile uint8_t state;
  volatile bool triggerFlag;
  volatile unsigned long detectUsIsr;
  unsigned long detectUsMain;
  unsigned long stateStartMs;
  volatile unsigned long lastEdgeMs;
  bool outputOn;
  unsigned long onSinceMs;
};

FinalLane gFinal[NUM_LANES];
bool gFinalInvalidPhysicsPrinted = false;

// -----------------------------------------------------------------------------
// Mode-specific state
// -----------------------------------------------------------------------------
// Solenoid Blink
uint8_t gBlinkLane = 0;
bool gBlinkIsOn = false;
unsigned long gBlinkChangeMs = 0;

// Comparator tuner
unsigned long gCompLastPrintMs = 0;

// ISR Echo
volatile bool gIsrEchoFlag[NUM_LANES] = {false, false, false, false};
volatile unsigned long gIsrEchoTimeUs[NUM_LANES] = {0, 0, 0, 0};

// One-shot
bool gOneShotActive[NUM_LANES] = {false, false, false, false};
unsigned long gOneShotStartMs[NUM_LANES] = {0, 0, 0, 0};

// Latency calibrator
enum LatencyState {
  LAT_IDLE = 0,
  LAT_WAIT,
  LAT_FIRE,
  LAT_COOLDOWN
};
uint8_t gLatencyState = LAT_IDLE;
volatile bool gLatencyTriggerFlag = false;
volatile unsigned long gLatencyDetectUs = 0;
unsigned long gLatencyDetectMs = 0;
unsigned long gLatencyStateMs = 0;

// Variable speed test
enum VarState {
  VAR_IDLE = 0,
  VAR_WAIT,
  VAR_FIRE
};
uint8_t gVarState = VAR_IDLE;
volatile bool gVarTriggerFlag = false;
volatile unsigned long gVarDetectUs = 0;
unsigned long gVarDetectMs = 0;
unsigned long gVarStateMs = 0;
bool gVarInvalidPhysicsPrinted = false;

// Serial command buffer
char gCmdBuffer[64];
uint8_t gCmdLen = 0;

// -----------------------------------------------------------------------------
// Pin Change Interrupt tracking
// -----------------------------------------------------------------------------
const uint8_t PORTD_WATCH_MASK = _BV(PD2) | _BV(PD4) | _BV(PD7);
const uint8_t PORTB_WATCH_MASK = _BV(PB0);
volatile uint8_t gLastPortDWatched = 0;
volatile uint8_t gLastPortBWatched = 0;

// -----------------------------------------------------------------------------
// Utility
// -----------------------------------------------------------------------------
void allSolenoidsOff() {
  for (uint8_t i = 0; i < NUM_LANES; i++) {
    digitalWrite(SOL_PINS[i], LOW);
    gOneShotActive[i] = false;
    gFinal[i].outputOn = false;
  }
}

long computeWaitDelayMs() {
  if (gScrollSpeedMmPerMs <= 0.0001f) {
    return LONG_MIN;
  }
  const float travelMs = gSensorDistMm / gScrollSpeedMmPerMs;
  const float waitMs = travelMs - (float)gMechLagMs;
  if (waitMs >= 0.0f) {
    return (long)(waitMs + 0.5f);
  }
  return (long)(waitMs - 0.5f);
}

void trimWhitespace(char *s) {
  size_t len = strlen(s);
  size_t start = 0;
  while (start < len && isspace((unsigned char)s[start])) {
    start++;
  }

  size_t end = len;
  while (end > start && isspace((unsigned char)s[end - 1])) {
    end--;
  }

  if (start > 0) {
    memmove(s, s + start, end - start);
  }
  s[end - start] = '\0';
}

void toLowerInPlace(char *s) {
  for (size_t i = 0; s[i] != '\0'; i++) {
    s[i] = (char)tolower((unsigned char)s[i]);
  }
}

bool isLaneBurst(const char *s) {
  if (s[0] == '\0') {
    return false;
  }
  for (size_t i = 0; s[i] != '\0'; i++) {
    if (s[i] < '1' || s[i] > '4') {
      return false;
    }
  }
  return true;
}

void printModeName(uint8_t mode) {
  switch (mode) {
    case MODE_SOL_BLINK:
      Serial.print(F("sol_blink"));
      return;
    case MODE_COMP_TUNER:
      Serial.print(F("comp_tuner"));
      return;
    case MODE_ISR_ECHO:
      Serial.print(F("isr_echo"));
      return;
    case MODE_ONESHOT:
      Serial.print(F("oneshot"));
      return;
    case MODE_LATENCY:
      Serial.print(F("latency"));
      return;
    case MODE_VAR_SPEED:
      Serial.print(F("speed"));
      return;
    case MODE_FINAL:
    default:
      Serial.print(F("final"));
      return;
  }
}

bool parseModeName(const char *arg, uint8_t &outMode) {
  if (strcmp(arg, "final") == 0) {
    outMode = MODE_FINAL;
    return true;
  }
  if (strcmp(arg, "blink") == 0 || strcmp(arg, "sol_blink") == 0) {
    outMode = MODE_SOL_BLINK;
    return true;
  }
  if (strcmp(arg, "tuner") == 0 || strcmp(arg, "comp_tuner") == 0) {
    outMode = MODE_COMP_TUNER;
    return true;
  }
  if (strcmp(arg, "isr") == 0 || strcmp(arg, "isr_echo") == 0) {
    outMode = MODE_ISR_ECHO;
    return true;
  }
  if (strcmp(arg, "oneshot") == 0 || strcmp(arg, "one_shot") == 0) {
    outMode = MODE_ONESHOT;
    return true;
  }
  if (strcmp(arg, "latency") == 0) {
    outMode = MODE_LATENCY;
    return true;
  }
  if (strcmp(arg, "speed") == 0 || strcmp(arg, "variable_speed") == 0) {
    outMode = MODE_VAR_SPEED;
    return true;
  }

  return false;
}

void printStatus() {
  Serial.println(F("--- Status ---"));
  Serial.print(F("Mode: "));
  printModeName(gMode);
  Serial.println();

  Serial.print(F("SENSOR_DIST_MM = "));
  Serial.println(gSensorDistMm, 3);
  Serial.print(F("MECH_LAG_MS    = "));
  Serial.println(gMechLagMs);
  Serial.print(F("SCROLL_SPEED   = "));
  Serial.println(gScrollSpeedMmPerMs, 4);

  long waitMs = computeWaitDelayMs();
  if (waitMs == LONG_MIN) {
    Serial.println(F("CALC_WAIT_MS   = INVALID (speed must be > 0)"));
  } else {
    Serial.print(F("CALC_WAIT_MS   = "));
    Serial.println(waitMs);
  }
}

void printHelp() {
  Serial.println(F("--- Commands (115200 baud) ---"));
  Serial.println(F("help                         -> show this menu"));
  Serial.println(F("status                       -> print active mode + calibration"));
  Serial.println(F("mode final                   -> final 4-lane integrated controller"));
  Serial.println(F("mode blink                   -> solenoid blink sanity test"));
  Serial.println(F("mode tuner                   -> comparator tuner (print sensors)"));
  Serial.println(F("mode isr                     -> ISR echo test"));
  Serial.println(F("mode oneshot                 -> non-blocking one-shot test"));
  Serial.println(F("mode latency                 -> 1-lane latency camera test"));
  Serial.println(F("mode speed                   -> 1-lane variable-speed verification"));
  Serial.println(F("set dist <mm>                -> update SENSOR_DIST_MM"));
  Serial.println(F("set lag <ms>                 -> update MECH_LAG_MS"));
  Serial.println(F("set speed <mm_per_ms>        -> update SCROLL_SPEED"));
  Serial.println(F("fire <lane>                  -> in oneshot mode, fire lane 1-4"));
  Serial.println(F("1234                         -> in oneshot mode, burst fire lanes"));
}

void fireOneShotLane(uint8_t lane, bool printLine) {
  if (lane >= NUM_LANES) {
    return;
  }
  if (gOneShotActive[lane]) {
    return;
  }

  digitalWrite(SOL_PINS[lane], HIGH);
  gOneShotActive[lane] = true;
  gOneShotStartMs[lane] = millis();

  if (printLine) {
    Serial.print(F("FIRING Lane "));
    Serial.println(lane + 1);
  }
}

void resetSharedFlagsAndState() {
  noInterrupts();

  for (uint8_t i = 0; i < NUM_LANES; i++) {
    gIsrEchoFlag[i] = false;
    gIsrEchoTimeUs[i] = 0;

    gFinal[i].state = FINAL_IDLE;
    gFinal[i].triggerFlag = false;
    gFinal[i].detectUsIsr = 0;
    gFinal[i].lastEdgeMs = 0;
  }

  gLatencyTriggerFlag = false;
  gLatencyDetectUs = 0;

  gVarTriggerFlag = false;
  gVarDetectUs = 0;

  interrupts();

  for (uint8_t i = 0; i < NUM_LANES; i++) {
    gFinal[i].detectUsMain = 0;
    gFinal[i].stateStartMs = 0;
    gFinal[i].outputOn = false;
    gFinal[i].onSinceMs = 0;

    gOneShotActive[i] = false;
    gOneShotStartMs[i] = 0;
  }

  gBlinkLane = 0;
  gBlinkIsOn = false;
  gBlinkChangeMs = millis() - SOL_BLINK_OFF_MS;

  gCompLastPrintMs = 0;

  gLatencyState = LAT_IDLE;
  gLatencyDetectMs = 0;
  gLatencyStateMs = 0;

  gVarState = VAR_IDLE;
  gVarDetectMs = 0;
  gVarStateMs = 0;

  gFinalInvalidPhysicsPrinted = false;
  gVarInvalidPhysicsPrinted = false;
}

void enterMode(uint8_t newMode) {
  if (newMode > MODE_VAR_SPEED) {
    return;
  }

  allSolenoidsOff();

  noInterrupts();
  gMode = newMode;
  interrupts();

  resetSharedFlagsAndState();

  Serial.print(F("Mode set to: "));
  printModeName(newMode);
  Serial.println();

  if (newMode == MODE_COMP_TUNER) {
    Serial.println(F("L1\tL2\tL3\tL4"));
  } else if (newMode == MODE_ONESHOT) {
    Serial.println(F("Type 1-4, 1234, or 'fire <lane>' to trigger lanes."));
  } else if (newMode == MODE_ISR_ECHO) {
    Serial.println(F("Waiting for falling edges on sensor pins 2,4,7,8..."));
  }
}

// -----------------------------------------------------------------------------
// Command processing
// -----------------------------------------------------------------------------
void processCommand(char *line) {
  trimWhitespace(line);
  if (line[0] == '\0') {
    return;
  }

  char cmd[64];
  strncpy(cmd, line, sizeof(cmd) - 1);
  cmd[sizeof(cmd) - 1] = '\0';
  toLowerInPlace(cmd);

  if (strcmp(cmd, "help") == 0) {
    printHelp();
    return;
  }

  if (strcmp(cmd, "status") == 0) {
    printStatus();
    return;
  }

  if (isLaneBurst(cmd)) {
    if (gMode != MODE_ONESHOT) {
      Serial.println(F("Lane burst is enabled only in mode oneshot."));
      return;
    }
    for (size_t i = 0; cmd[i] != '\0'; i++) {
      fireOneShotLane((uint8_t)(cmd[i] - '1'), true);
    }
    return;
  }

  char *token = strtok(cmd, " ");
  if (token == NULL) {
    return;
  }

  if (strcmp(token, "mode") == 0) {
    char *arg = strtok(NULL, " ");
    if (arg == NULL) {
      Serial.println(F("Usage: mode <final|blink|tuner|isr|oneshot|latency|speed>"));
      return;
    }

    uint8_t nextMode = MODE_FINAL;
    if (!parseModeName(arg, nextMode)) {
      Serial.println(F("Unknown mode."));
      return;
    }

    enterMode(nextMode);
    return;
  }

  if (strcmp(token, "fire") == 0) {
    if (gMode != MODE_ONESHOT) {
      Serial.println(F("Use 'mode oneshot' before manual lane firing."));
      return;
    }

    char *arg = strtok(NULL, " ");
    if (arg == NULL) {
      Serial.println(F("Usage: fire <1-4>"));
      return;
    }

    int lane = atoi(arg);
    if (lane < 1 || lane > 4) {
      Serial.println(F("Lane must be 1..4."));
      return;
    }

    fireOneShotLane((uint8_t)(lane - 1), true);
    return;
  }

  if (strcmp(token, "set") == 0) {
    char *key = strtok(NULL, " ");
    char *val = strtok(NULL, " ");

    if (key == NULL || val == NULL) {
      Serial.println(F("Usage: set <dist|lag|speed> <value>"));
      return;
    }

    if (strcmp(key, "dist") == 0) {
      float v = atof(val);
      if (v <= 0.0f) {
        Serial.println(F("Distance must be > 0 mm."));
        return;
      }
      gSensorDistMm = v;
    } else if (strcmp(key, "lag") == 0) {
      long v = atol(val);
      if (v < 0) {
        Serial.println(F("Lag must be >= 0 ms."));
        return;
      }
      gMechLagMs = (int)v;
    } else if (strcmp(key, "speed") == 0) {
      float v = atof(val);
      if (v <= 0.0f) {
        Serial.println(F("Speed must be > 0 mm/ms."));
        return;
      }
      gScrollSpeedMmPerMs = v;
    } else {
      Serial.println(F("Unknown set key. Use dist, lag, or speed."));
      return;
    }

    Serial.println(F("Calibration updated."));
    printStatus();
    return;
  }

  Serial.println(F("Unknown command. Type 'help'."));
}

void handleSerialInput() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\r' || c == '\n') {
      if (gCmdLen > 0) {
        gCmdBuffer[gCmdLen] = '\0';
        processCommand(gCmdBuffer);
        gCmdLen = 0;
      }
      continue;
    }

    if (gCmdLen < sizeof(gCmdBuffer) - 1) {
      gCmdBuffer[gCmdLen++] = c;
    }
  }
}

// -----------------------------------------------------------------------------
// Mode runners
// -----------------------------------------------------------------------------
void runSolBlink(unsigned long now) {
  if (!gBlinkIsOn) {
    if (now - gBlinkChangeMs >= SOL_BLINK_OFF_MS) {
      digitalWrite(SOL_PINS[gBlinkLane], HIGH);
      gBlinkIsOn = true;
      gBlinkChangeMs = now;
      Serial.print(F("Firing Lane "));
      Serial.println(gBlinkLane + 1);
    }
    return;
  }

  if (now - gBlinkChangeMs >= SOL_BLINK_ON_MS) {
    digitalWrite(SOL_PINS[gBlinkLane], LOW);
    gBlinkIsOn = false;
    gBlinkChangeMs = now;

    gBlinkLane = (uint8_t)((gBlinkLane + 1) % NUM_LANES);
    if (gBlinkLane == 0) {
      Serial.println(F("--- Cycle Complete ---"));
    }
  }
}

void runCompTuner(unsigned long now) {
  if (now - gCompLastPrintMs < COMP_TUNER_PRINT_MS) {
    return;
  }

  gCompLastPrintMs = now;

  Serial.print(digitalRead(SENSOR_PINS[0]));
  Serial.print('\t');
  Serial.print(digitalRead(SENSOR_PINS[1]));
  Serial.print('\t');
  Serial.print(digitalRead(SENSOR_PINS[2]));
  Serial.print('\t');
  Serial.println(digitalRead(SENSOR_PINS[3]));
}

void runIsrEcho() {
  for (uint8_t i = 0; i < NUM_LANES; i++) {
    bool fired = false;
    unsigned long stampUs = 0;

    noInterrupts();
    if (gIsrEchoFlag[i]) {
      fired = true;
      stampUs = gIsrEchoTimeUs[i];
      gIsrEchoFlag[i] = false;
    }
    interrupts();

    if (fired) {
      Serial.print(F(">> Lane "));
      Serial.print(i + 1);
      Serial.print(F(" Triggered at "));
      Serial.print(stampUs);
      Serial.println(F(" us"));
    }
  }
}

void runOneShot(unsigned long now) {
  for (uint8_t i = 0; i < NUM_LANES; i++) {
    if (!gOneShotActive[i]) {
      continue;
    }

    if (now - gOneShotStartMs[i] >= ONE_SHOT_HOLD_MS) {
      digitalWrite(SOL_PINS[i], LOW);
      gOneShotActive[i] = false;
    }
  }
}

void runLatencyCal(unsigned long now) {
  switch (gLatencyState) {
    case LAT_IDLE: {
      bool detected = false;
      unsigned long detectUs = 0;

      noInterrupts();
      if (gLatencyTriggerFlag) {
        detected = true;
        detectUs = gLatencyDetectUs;
        gLatencyTriggerFlag = false;
      }
      interrupts();

      if (detected) {
        gLatencyDetectMs = detectUs / 1000UL;
        gLatencyState = LAT_WAIT;
        Serial.println(F("Trigger detected. Starting 1000 ms delay..."));
      }
      break;
    }

    case LAT_WAIT:
      if (now - gLatencyDetectMs >= LATENCY_TEST_DELAY_MS) {
        digitalWrite(SOL_PINS[0], HIGH);
        gLatencyState = LAT_FIRE;
        gLatencyStateMs = now;
        Serial.println(F("FIRING!"));
      }
      break;

    case LAT_FIRE:
      if (now - gLatencyStateMs >= ONE_SHOT_HOLD_MS) {
        digitalWrite(SOL_PINS[0], LOW);
        gLatencyState = LAT_COOLDOWN;
        gLatencyStateMs = now;
        Serial.println(F("Resetting..."));
      }
      break;

    case LAT_COOLDOWN:
      if (now - gLatencyStateMs >= LATENCY_COOLDOWN_MS) {
        gLatencyState = LAT_IDLE;
      }
      break;

    default:
      gLatencyState = LAT_IDLE;
      break;
  }
}

void runVariableSpeed(unsigned long now) {
  long waitDelayMs = computeWaitDelayMs();
  if (waitDelayMs == LONG_MIN || waitDelayMs < 0) {
    if (!gVarInvalidPhysicsPrinted) {
      Serial.println(F("ERROR: Invalid wait delay. Check dist/lag/speed calibration."));
      gVarInvalidPhysicsPrinted = true;
    }
    allSolenoidsOff();
    gVarState = VAR_IDLE;
    return;
  }
  gVarInvalidPhysicsPrinted = false;

  switch (gVarState) {
    case VAR_IDLE: {
      bool detected = false;
      unsigned long detectUs = 0;

      noInterrupts();
      if (gVarTriggerFlag) {
        detected = true;
        detectUs = gVarDetectUs;
        gVarTriggerFlag = false;
      }
      interrupts();

      if (detected) {
        gVarDetectMs = detectUs / 1000UL;
        gVarState = VAR_WAIT;
        Serial.print(F("Note detected. Waiting "));
        Serial.print(waitDelayMs);
        Serial.println(F(" ms before firing..."));
      }
      break;
    }

    case VAR_WAIT:
      if (now - gVarDetectMs >= (unsigned long)waitDelayMs) {
        digitalWrite(SOL_PINS[0], HIGH);
        gVarState = VAR_FIRE;
        gVarStateMs = now;
        Serial.println(F("FIRING!"));
      }
      break;

    case VAR_FIRE:
      if (now - gVarStateMs >= ONE_SHOT_HOLD_MS) {
        digitalWrite(SOL_PINS[0], LOW);
        gVarState = VAR_IDLE;
      }
      break;

    default:
      gVarState = VAR_IDLE;
      break;
  }
}

void runFinal(unsigned long now) {
  long waitDelayMs = computeWaitDelayMs();

  if (waitDelayMs == LONG_MIN || waitDelayMs < 0) {
    if (!gFinalInvalidPhysicsPrinted) {
      Serial.println(F("CRITICAL: waitDelay < 0. Adjust dist/lag/speed (status + set commands)."));
      gFinalInvalidPhysicsPrinted = true;
    }

    allSolenoidsOff();
    for (uint8_t i = 0; i < NUM_LANES; i++) {
      gFinal[i].state = FINAL_IDLE;
      gFinal[i].triggerFlag = false;
    }
    return;
  }

  gFinalInvalidPhysicsPrinted = false;

  for (uint8_t i = 0; i < NUM_LANES; i++) {
    FinalLane &lane = gFinal[i];

    switch (lane.state) {
      case FINAL_IDLE: {
        bool hasTrigger = false;
        unsigned long detectUs = 0;

        noInterrupts();
        if (lane.triggerFlag) {
          hasTrigger = true;
          detectUs = lane.detectUsIsr;
          lane.triggerFlag = false;
        }
        interrupts();

        if (hasTrigger) {
          lane.detectUsMain = detectUs;
          lane.state = FINAL_WAITING;
        }
        break;
      }

      case FINAL_WAITING:
        if (micros() - lane.detectUsMain >= (unsigned long)waitDelayMs * 1000UL) {
          digitalWrite(SOL_PINS[i], HIGH);
          lane.outputOn = true;
          lane.onSinceMs = now;
          lane.stateStartMs = now;
          lane.state = FINAL_FIRING;
        }
        break;

      case FINAL_FIRING:
        if (now - lane.stateStartMs >= gFinalHoldMs) {
          digitalWrite(SOL_PINS[i], LOW);
          lane.outputOn = false;
          lane.stateStartMs = now;
          lane.state = FINAL_COOLDOWN;
        } else if (lane.outputOn && now - lane.onSinceMs >= gWatchdogMaxOnMs) {
          digitalWrite(SOL_PINS[i], LOW);
          lane.outputOn = false;
          lane.stateStartMs = now;
          lane.state = FINAL_COOLDOWN;
          Serial.print(F("WATCHDOG released lane "));
          Serial.println(i + 1);
        }
        break;

      case FINAL_COOLDOWN:
        if (now - lane.stateStartMs >= gFinalCooldownMs) {
          lane.state = FINAL_IDLE;
        }
        break;

      default:
        lane.state = FINAL_IDLE;
        break;
    }
  }
}

void runCurrentMode(unsigned long now) {
  switch (gMode) {
    case MODE_SOL_BLINK:
      runSolBlink(now);
      break;

    case MODE_COMP_TUNER:
      runCompTuner(now);
      break;

    case MODE_ISR_ECHO:
      runIsrEcho();
      break;

    case MODE_ONESHOT:
      runOneShot(now);
      break;

    case MODE_LATENCY:
      runLatencyCal(now);
      break;

    case MODE_VAR_SPEED:
      runVariableSpeed(now);
      break;

    case MODE_FINAL:
    default:
      runFinal(now);
      break;
  }
}

// -----------------------------------------------------------------------------
// ISR routing
// -----------------------------------------------------------------------------
void onSensorFallingISR(uint8_t lane, unsigned long irqUs) {
  const uint8_t mode = gMode;

  if (mode == MODE_ISR_ECHO) {
    gIsrEchoFlag[lane] = true;
    gIsrEchoTimeUs[lane] = irqUs;
    return;
  }

  if (mode == MODE_LATENCY) {
    if (lane == 0 && !gLatencyTriggerFlag) {
      gLatencyDetectUs = irqUs;
      gLatencyTriggerFlag = true;
    }
    return;
  }

  if (mode == MODE_VAR_SPEED) {
    if (lane == 0 && !gVarTriggerFlag) {
      gVarDetectUs = irqUs;
      gVarTriggerFlag = true;
    }
    return;
  }

  if (mode == MODE_FINAL) {
    FinalLane &L = gFinal[lane];
    if (L.state == FINAL_IDLE) {
      const unsigned long nowMs = irqUs / 1000UL;
      if (nowMs - L.lastEdgeMs >= FINAL_ISR_DEBOUNCE_MS) {
        L.detectUsIsr = irqUs;
        L.triggerFlag = true;
        L.lastEdgeMs = nowMs;
      }
    }
  }
}

ISR(PCINT2_vect) {
  const uint8_t currD = PIND & PORTD_WATCH_MASK;
  const unsigned long irqUs = micros();

  const uint8_t changed = (currD ^ gLastPortDWatched) & PORTD_WATCH_MASK;

  if (changed & _BV(PD2)) {
    const bool wasHigh = (gLastPortDWatched & _BV(PD2)) != 0;
    const bool isHigh = (currD & _BV(PD2)) != 0;
    if (wasHigh && !isHigh) {
      onSensorFallingISR(0, irqUs);
    }
  }

  if (changed & _BV(PD4)) {
    const bool wasHigh = (gLastPortDWatched & _BV(PD4)) != 0;
    const bool isHigh = (currD & _BV(PD4)) != 0;
    if (wasHigh && !isHigh) {
      onSensorFallingISR(1, irqUs);
    }
  }

  if (changed & _BV(PD7)) {
    const bool wasHigh = (gLastPortDWatched & _BV(PD7)) != 0;
    const bool isHigh = (currD & _BV(PD7)) != 0;
    if (wasHigh && !isHigh) {
      onSensorFallingISR(2, irqUs);
    }
  }

  gLastPortDWatched = currD;
}

ISR(PCINT0_vect) {
  const uint8_t currB = PINB & PORTB_WATCH_MASK;
  const unsigned long irqUs = micros();

  const uint8_t changed = (currB ^ gLastPortBWatched) & PORTB_WATCH_MASK;

  if (changed & _BV(PB0)) {
    const bool wasHigh = (gLastPortBWatched & _BV(PB0)) != 0;
    const bool isHigh = (currB & _BV(PB0)) != 0;
    if (wasHigh && !isHigh) {
      onSensorFallingISR(3, irqUs);
    }
  }

  gLastPortBWatched = currB;
}

// -----------------------------------------------------------------------------
// Arduino entry points
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  for (uint8_t i = 0; i < NUM_LANES; i++) {
    pinMode(SOL_PINS[i], OUTPUT);
    digitalWrite(SOL_PINS[i], LOW);
  }

  for (uint8_t i = 0; i < NUM_LANES; i++) {
    pinMode(SENSOR_PINS[i], INPUT_PULLUP);
  }

  // Snapshot initial pin states before enabling PCINT.
  gLastPortDWatched = PIND & PORTD_WATCH_MASK;
  gLastPortBWatched = PINB & PORTB_WATCH_MASK;

  // Enable Pin Change Interrupt groups for Port D and Port B.
  PCICR |= _BV(PCIE2);
  PCICR |= _BV(PCIE0);

  // Enable specific sensor pins: D2, D4, D7, D8.
  PCMSK2 |= _BV(PD2);
  PCMSK2 |= _BV(PD4);
  PCMSK2 |= _BV(PD7);
  PCMSK0 |= _BV(PB0);

  resetSharedFlagsAndState();

  Serial.println();
  Serial.println(F("=== Phys124 Computer Pianist Unified Sketch ==="));
  Serial.println(F("Pins: Sensors 2,4,7,8 (INPUT_PULLUP), Solenoids 3,5,6,9"));
  printHelp();
  printStatus();

  enterMode(MODE_FINAL);
}

void loop() {
  handleSerialInput();
  runCurrentMode(millis());
}
