#include <Adafruit_MCP23X17.h>
#include <Encoder.h>
#include <XInput.h>

Adafruit_MCP23X17 mcp;
Encoder jogWheel(2, 3);

//Pins
const int BOARD_LED = 13;
const int ENC_SW = 4;
const int POT_PIN = A0;
const int STOP_PLAY_LED = 7;
const int ENC_LED = 6;

//State Variabels
long oldPosition = -999;
int lastPotValue = -1;
unsigned long lastPotMillis = 0;
bool isShuttleMode = false;
bool isPlaying = false;
unsigned long lastBlinkMillis = 0;
bool ledState = false;

//MCP Buttons
const int numButtons = 11;
int btnPins[numButtons] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
bool lastBtnState[numButtons];

//Fader Calibration
int rawMin = 1023;
int rawMax = 0;

void setup() {
  pinMode(BOARD_LED, OUTPUT);
  pinMode(STOP_PLAY_LED, OUTPUT);
  pinMode(ENC_LED, OUTPUT);
  pinMode(ENC_SW, INPUT_PULLUP);

  if (!mcp.begin_I2C(0x27)) {
    while(1) {
      digitalWrite(BOARD_LED, !digitalRead(BOARD_LED));
      delay(100);
    }
  }

  for (int i = 0; i < numButtons; i++) {
    mcp.pinMode(btnPins[i], INPUT_PULLUP);
    lastBtnState[i] = HIGH;
  }

  XInput.setJoystickRange(0, 1023); 

  XInput.setTriggerRange(0, 255);

  XInput.begin();
}

void loop() {
  handleButtons();
  handleEncoder();
  handlePot();
  handleLEDs();
}

void handleButtons() {
  for (int i = 0; i < numButtons; i++) {
    bool currentState = mcp.digitalRead(btnPins[i]);
    if (currentState != lastBtnState[i]) {
      delay(15); 
      bool pressed = (currentState == LOW);

      switch(i) {
        case 0: XInput.setButton(BUTTON_A, pressed); break;
        case 1: XInput.setButton(BUTTON_B, pressed); break;
        case 2: XInput.setButton(BUTTON_X, pressed); break;
        case 3: XInput.setButton(BUTTON_Y, pressed); break;
        case 4: XInput.setButton(BUTTON_LB, pressed); break;
        case 5: XInput.setButton(BUTTON_RB, pressed); break;
        case 6: XInput.setDpad(pressed, false, false, false); break; // Up
        case 7: XInput.setDpad(false, pressed, false, false); break; // Down
        case 8: 
          XInput.setButton(BUTTON_START, pressed); 
          if (pressed) isPlaying = !isPlaying;
          break;
        case 9: XInput.setButton(BUTTON_BACK, pressed); break;
        case 10: XInput.setButton(BUTTON_L3, pressed); break;
      }
      lastBtnState[i] = currentState;
    }
  }

  static bool lastSwState = HIGH;
  bool swState = digitalRead(ENC_SW);
  if (swState != lastSwState) {
    delay(15);
    XInput.setButton(BUTTON_R3, (swState == LOW));
    if (swState == LOW) isShuttleMode = !isShuttleMode;
    lastSwState = swState;
  }
}

void handleEncoder() {
  long newPos = jogWheel.read() / 4;
  if (newPos != oldPosition) {
    if (newPos > oldPosition) {
      XInput.setJoystick(JOY_LEFT, 1023, 512); // Pulse Right
    } else {
      XInput.setJoystick(JOY_LEFT, 0, 512);    // Pulse Left
    }
    oldPosition = newPos;
    delay(2); // Small delay to help Windows "see" the flick
  } else {
    XInput.setJoystick(JOY_LEFT, 512, 512);   // Center
  }
}

void handlePot() {
  if (millis() - lastPotMillis > 30) {
    int raw = analogRead(POT_PIN);
    
    if (raw < rawMin && raw > 5) rawMin = raw;
    if (raw > rawMax) rawMax = raw;

    // Declaring triggerVal inside the scope before use
    int triggerVal = map(raw, rawMin, rawMax, 0, 255);
    triggerVal = constrain(triggerVal, 0, 255);

    if (abs(triggerVal - lastPotValue) > 1) {
      XInput.setTrigger(TRIGGER_LEFT, triggerVal);
      lastPotValue = triggerVal;
      lastPotMillis = millis();
    }
  }
}

void handleLEDs() {
  unsigned long currentMillis = millis();
  if (isPlaying) {
    if (currentMillis - lastBlinkMillis >= 500) {
      lastBlinkMillis = currentMillis;
      ledState = !ledState;
      digitalWrite(STOP_PLAY_LED, ledState);
    }
  } else {
    digitalWrite(STOP_PLAY_LED, HIGH);
  }
  digitalWrite(ENC_LED, isShuttleMode ? ((currentMillis / 250) % 2 == 0) : HIGH);
}
