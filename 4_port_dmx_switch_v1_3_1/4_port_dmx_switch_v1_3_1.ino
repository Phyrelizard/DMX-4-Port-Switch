#include <DMXSerial.h>

// 4-Port DMX Switch Controller
// version 1.3.1
//
// PCB/perfboard-friendly pinout:
//
// - D3 through D11 are used for the 9 DMX address DIP switches
// - DIP 10 is reserved for hardware RS485 receiver enable/disable
// - Relay outputs moved to A0 through A3
// - Button A moved to A4
// - Button B moved to A5
// - External demo/status LED remains on D2
// - New DMX present LED on D12
// - MAX485 RO still connects to Nano D0/RX
//
// DIP 10 hardware wiring recommendation:
//   MAX485 /RE -> 10k pullup -> 5V
//   MAX485 /RE -> DIP switch 10 -> GND
//   MAX485 DE  -> GND
//
// DIP 10 ON  = RS485 receiver enabled
// DIP 10 OFF = RS485 receiver disabled

// -----------------------------
// Relay pin assignments
// -----------------------------
// A0 through A3 are used as digital output pins.
const uint8_t relayPins[4] = {
  A0, // Relay 1
  A1, // Relay 2
  A2, // Relay 3
  A3  // Relay 4
};

const uint8_t RELAY_COUNT = 4;

// Many 4-relay boards are active LOW.
// If your relays work backwards, swap these two values.
const uint8_t RELAY_ON  = LOW;
const uint8_t RELAY_OFF = HIGH;

// -----------------------------
// Demo/status LED
// Wire as:
//   D2 -> series resistor -> LED anode
//   LED cathode -> GND
//
// Active HIGH:
//   HIGH = LED on
//   LOW  = LED off
// -----------------------------
const uint8_t STATUS_LED_PIN = 2;

// -----------------------------
// DMX present LED
// Wire as:
//   D12 -> series resistor -> LED anode
//   LED cathode -> GND
//
// Active HIGH:
//   HIGH = valid DMX stream present
//   LOW  = no DMX stream
// -----------------------------
const uint8_t DMX_PRESENT_LED_PIN = 12;

// DMX present LED flicker timing.
//10 Hz full blink rate => toggle every ~16.7 ms.
const unsigned long DMX_LED_HALF_PERIOD_US = 50000;
bool dmxLedState = false;
unsigned long lastDmxLedToggleUs = 0;

// -----------------------------
// DIP switch pins
//
// DIP1 = 1
// DIP2 = 2
// DIP3 = 4
// DIP4 = 8
// DIP5 = 16
// DIP6 = 32
// DIP7 = 64
// DIP8 = 128
// DIP9 = 256
//
// DIP switch ON = pin grounded = LOW
//
// DIP 10 is not read by this sketch.
// Use DIP 10 as a hardware enable/disable for the MAX485 /RE line.
// -----------------------------
const uint8_t addrPins[9] = {
  3,  // DIP 1 = address bit 1
  4,  // DIP 2 = address bit 2
  5,  // DIP 3 = address bit 4
  6,  // DIP 4 = address bit 8
  7,  // DIP 5 = address bit 16
  8,  // DIP 6 = address bit 32
  9,  // DIP 7 = address bit 64
  10, // DIP 8 = address bit 128
  11  // DIP 9 = address bit 256
};

const uint8_t ADDR_PIN_COUNT = 9;

// -----------------------------
// Demo mode buttons
//
// Active LOW because both use INPUT_PULLUP.
//
// Button A:
//   A4 - long press enters/exits demo mode
//   A4 - short press selects effect while in demo mode
//
// Button B:
//   A5 - short press selects demo speed while in demo mode
// -----------------------------
const uint8_t BUTTON_A_PIN = A4;
const uint8_t BUTTON_B_PIN = A5;

const unsigned long DEBOUNCE_MS = 40;
const unsigned long BUTTON_A_LONG_PRESS_MS = 1000;

// -----------------------------
// Demo mode timing and effects
// -----------------------------
const unsigned long demoSpeedsMs[6] = {
  1000,
  500,
  400,
  300,
  200,
  100
};

const uint8_t DEMO_SPEED_COUNT = 6;

enum DemoEffect : uint8_t {
  DEMO_CYCLE = 0,
  DEMO_SEQUENCE_LEFT_RIGHT,
  DEMO_SEQUENCE_RIGHT_LEFT,
  DEMO_PING_PONG,
  DEMO_RANDOM,
  DEMO_SOLID_ON,
  DEMO_EFFECT_COUNT
};

struct ButtonState {
  uint8_t pin;
  bool stableState;
  bool lastRawState;
  unsigned long lastRawChangeMs;
  unsigned long pressedAtMs;
  bool longPressFired;
  bool shortPressEvent;
  bool longPressEvent;
};

ButtonState buttonA = {
  BUTTON_A_PIN,
  HIGH,
  HIGH,
  0,
  0,
  false,
  false,
  false
};

ButtonState buttonB = {
  BUTTON_B_PIN,
  HIGH,
  HIGH,
  0,
  0,
  false,
  false,
  false
};

uint16_t startAddress = 1;

bool inDemoMode = false;
DemoEffect currentDemoEffect = DEMO_CYCLE;
uint8_t currentDemoSpeedIndex = 0;

unsigned long lastDemoStepMs = 0;
uint8_t demoStep = 0;
int8_t pingPongDirection = 1;
bool cycleStateOn = true;

// -----------------------------
// Read DIP switch address
//
// Switch ON = pin grounded = LOW
// Switch OFF = internal pullup = HIGH
// -----------------------------
uint16_t readDipAddress()
{
  uint16_t addr = 0;

  for (uint8_t i = 0; i < ADDR_PIN_COUNT; i++) {
    if (digitalRead(addrPins[i]) == LOW) {
      addr |= (1U << i);
    }
  }

  // Valid DMX start address for 4 channels is 1..509.
  if (addr < 1) {
    addr = 1;
  }

  if (addr > 509) {
    addr = 509;
  }

  return addr;
}

// -----------------------------
void allRelaysOff()
{
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    digitalWrite(relayPins[i], RELAY_OFF);
  }
}

// -----------------------------
void allRelaysOn()
{
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    digitalWrite(relayPins[i], RELAY_ON);
  }
}

// -----------------------------
void setOnlyOneRelayOn(uint8_t relayIndex)
{
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    if (i == relayIndex) {
      digitalWrite(relayPins[i], RELAY_ON);
    } else {
      digitalWrite(relayPins[i], RELAY_OFF);
    }
  }
}

// -----------------------------
void setRelayMask(uint8_t mask)
{
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    if (mask & (1 << i)) {
      digitalWrite(relayPins[i], RELAY_ON);
    } else {
      digitalWrite(relayPins[i], RELAY_OFF);
    }
  }
}

// -----------------------------
void updateButton(ButtonState &button, unsigned long longPressMs)
{
  button.shortPressEvent = false;
  button.longPressEvent = false;

  const unsigned long now = millis();
  const bool rawState = digitalRead(button.pin);

  if (rawState != button.lastRawState) {
    button.lastRawState = rawState;
    button.lastRawChangeMs = now;
  }

  if ((now - button.lastRawChangeMs) >= DEBOUNCE_MS && rawState != button.stableState) {
    button.stableState = rawState;

    if (button.stableState == LOW) {
      // Button pressed
      button.pressedAtMs = now;
      button.longPressFired = false;
    } else {
      // Button released
      if (!button.longPressFired) {
        button.shortPressEvent = true;
      }
    }
  }

  if (longPressMs > 0 && button.stableState == LOW && !button.longPressFired) {
    if ((now - button.pressedAtMs) >= longPressMs) {
      button.longPressFired = true;
      button.longPressEvent = true;
    }
  }
}

// -----------------------------
void resetDemoPatternState()
{
  lastDemoStepMs = millis();
  demoStep = 0;
  pingPongDirection = 1;
  cycleStateOn = true;

  switch (currentDemoEffect) {
    case DEMO_CYCLE:
      allRelaysOn();
      break;

    case DEMO_SEQUENCE_LEFT_RIGHT:
      setOnlyOneRelayOn(0);
      break;

    case DEMO_SEQUENCE_RIGHT_LEFT:
      setOnlyOneRelayOn(RELAY_COUNT - 1);
      break;

    case DEMO_PING_PONG:
      setOnlyOneRelayOn(0);
      break;

    case DEMO_RANDOM:
      setRelayMask(random(1, 16)); // random non-zero 4-bit pattern
      break;

    case DEMO_SOLID_ON:
      allRelaysOn();
      break;

    default:
      allRelaysOff();
      break;
  }
}

// -----------------------------
void enterDemoMode()
{
  inDemoMode = true;
  currentDemoEffect = DEMO_CYCLE;
  currentDemoSpeedIndex = 0;
  resetDemoPatternState();

  // DMX present LED is off during demo mode.
  digitalWrite(DMX_PRESENT_LED_PIN, LOW);
}

// -----------------------------
void exitDemoMode()
{
  inDemoMode = false;
  allRelaysOff();
  digitalWrite(STATUS_LED_PIN, LOW);
  digitalWrite(DMX_PRESENT_LED_PIN, LOW);
}

// -----------------------------
void nextDemoEffect()
{
  currentDemoEffect = (DemoEffect)((currentDemoEffect + 1) % DEMO_EFFECT_COUNT);
  resetDemoPatternState();
}

// -----------------------------
void nextDemoSpeed()
{
  currentDemoSpeedIndex++;

  if (currentDemoSpeedIndex >= DEMO_SPEED_COUNT) {
    currentDemoSpeedIndex = 0;
  }

  // Restart the current visual step timing immediately so the new speed feels responsive.
  lastDemoStepMs = millis();
}

// -----------------------------
void handleDemoMode()
{
  // Demo mode ignores DMX input for output control.

  // Fast blink built-in LED to show demo mode is active.
  digitalWrite(LED_BUILTIN, (millis() / 100) % 2);

  // External demo/status LED blinks while demo mode is active.
  // About 600 ms on / 600 ms off.
  digitalWrite(STATUS_LED_PIN, (millis() / 600) % 2);

  // DMX present LED is off during demo mode.
  digitalWrite(DMX_PRESENT_LED_PIN, LOW);

  if (buttonA.shortPressEvent) {
    nextDemoEffect();
  }

  if (buttonB.shortPressEvent) {
    nextDemoSpeed();
  }

  const unsigned long now = millis();
  const unsigned long stepMs = demoSpeedsMs[currentDemoSpeedIndex];

  if (currentDemoEffect == DEMO_SOLID_ON) {
    allRelaysOn();
    return;
  }

  if ((now - lastDemoStepMs) < stepMs) {
    return;
  }

  lastDemoStepMs = now;

  switch (currentDemoEffect) {
    case DEMO_CYCLE:
      cycleStateOn = !cycleStateOn;

      if (cycleStateOn) {
        allRelaysOn();
      } else {
        allRelaysOff();
      }
      break;

    case DEMO_SEQUENCE_LEFT_RIGHT:
      demoStep++;

      if (demoStep >= RELAY_COUNT) {
        demoStep = 0;
      }

      setOnlyOneRelayOn(demoStep);
      break;

    case DEMO_SEQUENCE_RIGHT_LEFT:
      demoStep++;

      if (demoStep >= RELAY_COUNT) {
        demoStep = 0;
      }

      setOnlyOneRelayOn((RELAY_COUNT - 1) - demoStep);
      break;

    case DEMO_PING_PONG:
      if (demoStep == 0) {
        pingPongDirection = 1;
      } else if (demoStep >= (RELAY_COUNT - 1)) {
        pingPongDirection = -1;
      }

      demoStep = demoStep + pingPongDirection;
      setOnlyOneRelayOn(demoStep);
      break;

    case DEMO_RANDOM:
      setRelayMask(random(1, 16)); // random non-zero 4-bit pattern
      break;

    case DEMO_SOLID_ON:
    default:
      allRelaysOn();
      break;
  }
}

// -----------------------------
void updateDmxPresentLed(bool dmxPresent)
{
  if (!dmxPresent) {
    dmxLedState = false;
    digitalWrite(DMX_PRESENT_LED_PIN, LOW);
    return;
  }

  unsigned long nowUs = micros();

  if ((unsigned long)(nowUs - lastDmxLedToggleUs) >= DMX_LED_HALF_PERIOD_US) {
    lastDmxLedToggleUs = nowUs;
    dmxLedState = !dmxLedState;
    digitalWrite(DMX_PRESENT_LED_PIN, dmxLedState ? HIGH : LOW);
  }
}

// -----------------------------
void handleDmxMode()
{
  const bool dmxPresent = (DMXSerial.noDataSince() <= 1000);

  // DMX present LED = flicker whenever valid DMX stream is present.
  updateDmxPresentLed(dmxPresent);

  // Demo/status LED is only used for demo mode.
  digitalWrite(STATUS_LED_PIN, LOW);

  // If DMX has been missing for over 1000 ms,
  // fail safe: turn all relays off.
  if (!dmxPresent) {
    allRelaysOff();

    // Blink built-in LED to show no DMX.
    digitalWrite(LED_BUILTIN, (millis() / 250) % 2);
    return;
  }

  // DMX is present.
  digitalWrite(LED_BUILTIN, HIGH);

  // Read 4 consecutive channels starting at startAddress.
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    uint8_t dmxValue = DMXSerial.read(startAddress + i);

    if (dmxValue >= 128) {
      digitalWrite(relayPins[i], RELAY_ON);
    } else {
      digitalWrite(relayPins[i], RELAY_OFF);
    }
  }
}

// -----------------------------
void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  pinMode(DMX_PRESENT_LED_PIN, OUTPUT);
  digitalWrite(DMX_PRESENT_LED_PIN, LOW);

  // Relay outputs
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i], RELAY_OFF);
  }

  // DIP switch inputs
  for (uint8_t i = 0; i < ADDR_PIN_COUNT; i++) {
    pinMode(addrPins[i], INPUT_PULLUP);
  }

  // Demo button inputs
  pinMode(BUTTON_A_PIN, INPUT_PULLUP);
  pinMode(BUTTON_B_PIN, INPUT_PULLUP);

  delay(20); // Let inputs settle.

  startAddress = readDipAddress();

  // Initialize DMX receiver.
  //
  // MAX485 RO connects to Nano D0/RX.
  // MAX485 /RE should be handled by DIP 10 hardware switch.
  // MAX485 DE should be tied to GND for receive-only operation.
  DMXSerial.init(DMXReceiver);

  randomSeed(analogRead(A7));

  allRelaysOff();
}

// -----------------------------
void loop()
{
  updateButton(buttonA, BUTTON_A_LONG_PRESS_MS);
  updateButton(buttonB, 0); // Button B uses short press only.

  if (buttonA.longPressEvent) {
    if (inDemoMode) {
      exitDemoMode();
    } else {
      enterDemoMode();
    }
  }

  if (inDemoMode) {
    handleDemoMode();
  } else {
    handleDmxMode();
  }
}
