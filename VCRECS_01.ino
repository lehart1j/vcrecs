#include <Wire.h>
#include <Adafruit_MCP23X17.h>

// MIDI CONFIG
static const uint8_t MIDI_CH = 1;
static const uint8_t BTN_NOTE_BASE = 60;
static const uint8_t PLAY_STATE_CC = 30;
static const uint8_t MODE_CC = 31;
static const uint8_t JOG_REL_CC = 21;
static const uint8_t SHUTTLE_CC = 20;
static const uint8_t SLIDER_CC = 10;

//Heartbeat 
static const uint8_t HEARTBEAT_CC = 119;
static const uint32_t HEARTBEAT_MS = 1000;

uint32_t lastHeartbeat = 0;
bool heartbeatFlip = false;

// PINS
static const uint8_t ENC_A_PIN = 2;
static const uint8_t ENC_B_PIN = 3;
static const uint8_t ENC_SW_PIN = 4;
static const uint8_t SLIDER_PIN = A0;

// MCP23017
static const uint8_t MCP_ADDR = 0x20;
static const uint8_t NUM_BTNS = 11;

static const uint8_t mcpPins[NUM_BTNS] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };

Adafruit_MCP23X17 mcp;

// TIMING / FILTERING
static const uint32_t DEBOUNCE_MS = 20;
static const uint32_t MCP_POLL_MS = 2;
static const uint32_t SLIDER_POLL_MS = 5;
static const uint8_t SLIDER_DEADBAND = 1;

// ENCODER
volatile int32_t encDelta = 0;
volatile uint8_t encLast = 0;

//Shuttle stepped speeds (axis CC values), 64=Center Stop
static const uint8_t shuttleSteps[] = { 0, 16, 32, 48, 56, 64, 72, 80, 96, 112, 127 };
static const int8_t SHUTTLE_CENTER_INDEX = 5;
static const int8_t SHUTTLE_MIN_INDEX = 0;
static const int8_t SHUTTLE_MAX_INDEX = (int8_t)(sizeof(shuttleSteps) - 1);

// State
bool playing = false;
bool shuttleMode = false;

//Debounce for encoder switch
bool lastEncSw = true;
uint32_t lastEncSwChange = 0;

//Debounce for MCP Buttons
uint16_t lastBtnBits = 0xFFFF;
uint32_t lastBtnChangeMs[NUM_BTNS] = {0};

//Slider State
uint8_t lastSliderMidi = 255;

//Shuttle State
int8_t shuttleIndex = SHUTTLE_CENTER_INDEX;
uint8_t lastShuttleValue = 64;

// Poll Timers
uint32_t lastMcpPoll = 0;
uint32_t lastSliderPoll = 0;

static inline uint8_t clampMidi(int v) {
  if (v < 0) return 0;
  if (v > 127) return 127;
  return (uint8_t)v;
}

void sendPlayState();
void sendModeState();
void sendSliderCC(uint8_t v);
void sendShuttleCC(uint8_t v);

//Heartbeat Helpers
uint8_t readSliderMidiNow() {
  uint16_t raw = analogRead(SLIDER_PIN);
  return clampMidi((int)((raw * 127UL) / 1023UL));
}

void sendHeartbeat(uint32_t now) {
  if (now - lastHeartbeat >= HEARTBEAT_MS) {
    lastHeartbeat = now;
    heartbeatFlip = !heartbeatFlip;
    usbMIDI.sendControlChange(HEARTBEAT_CC, heartbeatFlip ? 127 : 0, MIDI_CH);
  }
}

void sendStartupDump() {
  //Send core states
  sendPlayState();
  sendModeState();
  sendShuttleCC(64);
  uint8_t s = readSliderMidiNow();
  lastSliderMidi = s;
  sendSliderCC(s);
  usbMIDI.sendControlChange(HEARTBEAT_CC, 127, MIDI_CH);
}

//Midi Helpers
void sendButtonNote(uint8_t btnIndex, bool pressed) {
  uint8_t note = BTN_NOTE_BASE + btnIndex;
  if (pressed) usbMIDI.sendNoteOn(note, 100, MIDI_CH);
  else         usbMIDI.sendNoteOff(note, 0, MIDI_CH);
}

void sendPlayState() {
  usbMIDI.sendControlChange(PLAY_STATE_CC, playing ? 127: 0, MIDI_CH);
}

void sendModeState() {
  usbMIDI.sendControlChange(MODE_CC, shuttleMode ? 127 : 0, MIDI_CH);
}

void sendSliderCC(uint8_t v) {
  usbMIDI.sendControlChange(SLIDER_CC, v, MIDI_CH);
}

void sendShuttleCC(uint8_t v) {
  usbMIDI.sendControlChange(SHUTTLE_CC, v, MIDI_CH);
}

void sendJogRelativeStep(int8_t step) {
  if (step > 0) usbMIDI.sendControlChange(JOG_REL_CC, 1, MIDI_CH);
  else          usbMIDI.sendControlChange(JOG_REL_CC, 127, MIDI_CH);
}

// Encoder ISR
void encoderISR() {
  uint8_t a = digitalReadFast(ENC_A_PIN);
  uint8_t b = digitalReadFast(ENC_B_PIN);
  uint8_t s = (a << 1) | b;

  int8_t delta = 0;
  uint8_t last = encLast;

  if (last == 0b00) {
    if (s == 0b01) delta = +1;
    else if (s == 0b10) delta = -1;
  } else if (last == 0b01) {
    if (s == 0b11) delta = +1;
    else if (s == 0b00) delta = -1;
  } else if (last == 0b11) {
    if (s == 0b10) delta = +1;
    else if (s == 0b01) delta = -1;
  } else if (last == 0b10) {
    if (s == 0b00) delta = +1;
    else if (s == 0b11) delta = -1;
  }

  encLast = s;
  encDelta += delta;
}

// SETUP
void setup() {
  pinMode(ENC_A_PIN, INPUT_PULLUP);
  pinMode(ENC_B_PIN, INPUT_PULLUP);
  pinMode(ENC_SW_PIN, INPUT_PULLUP);

  analogReadResolution(10);

  encLast = ((digitalReadFast(ENC_A_PIN) << 1) | digitalReadFast(ENC_B_PIN));

  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_PIN), encoderISR, CHANGE);

  Wire.begin();
  Wire.setClock(400000);


  if (!mcp.begin_I2C(MCP_ADDR)) {
    while (1) delay(50);
  }

  //MCP button pins as inputs with pullips
  for (uint8_t i = 0; i < NUM_BTNS; i++) {
    mcp.pinMode(mcpPins[i], INPUT_PULLUP);
  }
  lastBtnBits = 0;
  for (uint8_t i = 0; i < NUM_BTNS; i++) {
    if (mcp.digitalRead(mcpPins[i])) lastBtnBits |= (1u << i);
  }

  //Inital state outputs
  sendStartupDump();
  lastHeartbeat = millis();
  heartbeatFlip = true;
}

//Loop
void loop() {
  //Keep USB MIDI Happy
  while (usbMIDI.read()) {}

  const uint32_t now = millis();
  sendHeartbeat(now);

  // Encoder Switch Toggle
  bool sw = digitalReadFast(ENC_SW_PIN);
  if (sw != lastEncSw && (now - lastEncSwChange) > DEBOUNCE_MS) {
    lastEncSwChange = now;
    lastEncSw = sw;

    if (sw == false){
      shuttleMode = !shuttleMode;
      sendModeState();

      if (!shuttleMode) {
        shuttleIndex = SHUTTLE_CENTER_INDEX;
        lastShuttleValue = 64;
        sendShuttleCC(64);
      } else {
        sendShuttleCC(lastShuttleValue);
      }
    }
  }

  // Encoder Rotation - Jog (Relative) / Shuttle (Axis)
  int32_t d;
  noInterrupts();
  d = encDelta;
  encDelta = 0;
  interrupts();

  if (d > 50) d = 50;
  if (d < -50) d = -50;

  if (d != 0) {
    if (!shuttleMode) {
      //Jog: relative CC per tick
      if (d > 0) {
        for (int32_t i = 0; i < d; i++) sendJogRelativeStep(+1);
      } else {
        for (int32_t i = 0; i < (-d); i++) sendJogRelativeStep(-1);
      }
    } else {
      //Shuttle: stepped axis
      int8_t step = (d > 0) ? 1 : -1;
      int next = shuttleIndex + step;
      if (next < SHUTTLE_MIN_INDEX) next = SHUTTLE_MIN_INDEX;
      if (next > SHUTTLE_MAX_INDEX) next = SHUTTLE_MAX_INDEX;
      shuttleIndex = (int8_t)next;

      uint8_t v = shuttleSteps[shuttleIndex];
      if (v != lastShuttleValue) {
        lastShuttleValue = v;
        sendShuttleCC(v);
      }
    }
  }

  // Slider CC
  if (now - lastSliderPoll >= SLIDER_POLL_MS) {
    lastSliderPoll = now;

    uint16_t raw = analogRead(SLIDER_PIN);
    uint8_t midi = clampMidi((int)((raw * 127UL) / 1023UL));

    if (lastSliderMidi == 255 ||
      midi > (uint8_t)(lastSliderMidi + SLIDER_DEADBAND) ||
      midi + SLIDER_DEADBAND < lastSliderMidi) {
        lastSliderMidi = midi;
        sendSliderCC(midi);
      }
  }

  // MCP23017
  if (now - lastMcpPoll >= MCP_POLL_MS) {
    lastMcpPoll = now;

    for (uint8_t i = 0; i < NUM_BTNS; i++) {
      bool level = mcp.digitalRead(mcpPins[i]);
      bool prevReleased = ((lastBtnBits & (1u << i)) !=0);

      if (level != prevReleased) {
        if (now - lastBtnChangeMs[i] >= DEBOUNCE_MS) {
          lastBtnChangeMs[i] = now;

          // Update stored release bit
          if (level) lastBtnBits |= (1u << i);
          else       lastBtnBits &= ~(1u << i);

          bool pressed = (level == LOW);

          if (i == 8) {
            if (pressed) {
              usbMIDI.sendNoteOn(90, 100, MIDI_CH);
              usbMIDI.sendNoteOff(90, 0, MIDI_CH);
              playing = !playing;
              sendPlayState();
            }
          } else {
            sendButtonNote(i, pressed);
          }
        }
      }
    }
  }
  usbMIDI.send_now();
}
