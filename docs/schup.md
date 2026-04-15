
//// Schup demo

#include <Servo.h>


Servo lid;


// ---------------- PINS ----------------
const int LID_PIN   = 9;
const int LED_PIN   = 3;
const int BLINK_BTN = 2;
const int HALF_BTN  = 4;
const int LED_BTN   = 5;


// ---------------- LID POSITIONS ----------------
const int OPEN   = 80;
const int HALF   = 143;
const int CLOSED = 172;


// ---------------- BLINK TIMING ----------------
const int BLINK_CLOSE_SPEED = 1;    // lower = faster
const int BLINK_OPEN_SPEED  = 1;    // lower = faster
const int BLINK_PAUSE       = 180;  // milliseconds at bottom


// ---------------- BUTTON MEMORY ----------------
bool lastBlinkState = HIGH;
bool lastHalfState  = HIGH;
bool lastLedState   = HIGH;


// ---------------- LID STATE ----------------
bool halfMode = false;
int currentPos = OPEN;


// ---------------- LED STATE ----------------
bool pulseEnabled = false;
bool fadeOutActive = false;


float currentBrightness = 0.0;
float targetBrightness  = 0.0;


unsigned long lastLedUpdate = 0;
unsigned long lastTargetChange = 0;


// Faster than before
const int LED_UPDATE_INTERVAL = 8;      // was slower, now faster updates
unsigned long targetHoldTime = 220;     // new target more often


const int MIN_BRIGHTNESS = 35;          // never fully dark while active
const int MAX_BRIGHTNESS = 180;


float smoothing = 0.18;                 // faster movement toward target
float fadeOutAmount = 8.0;              // amount to subtract each update


void setup() {
  lid.attach(LID_PIN);


  pinMode(BLINK_BTN, INPUT_PULLUP);
  pinMode(HALF_BTN, INPUT_PULLUP);
  pinMode(LED_BTN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);


  lid.write(OPEN);
  currentPos = OPEN;


  analogWrite(LED_PIN, 0);


  randomSeed(analogRead(A0));
  targetBrightness = random(MIN_BRIGHTNESS, MAX_BRIGHTNESS + 1);
}


void loop() {
  handleHalfToggle();
  handleBlink();
  handleLedToggle();
  updateOrganicLED();
}


// ---------------- HALF TOGGLE ----------------
void handleHalfToggle() {
  bool currentHalfState = digitalRead(HALF_BTN);


  if (lastHalfState == HIGH && currentHalfState == LOW) {
    halfMode = !halfMode;


    if (halfMode) {
      lid.write(HALF);
      currentPos = HALF;
    } else {
      lid.write(OPEN);
      currentPos = OPEN;
    }
  }


  lastHalfState = currentHalfState;
}


// ---------------- FULL BLINK ----------------
void handleBlink() {
  bool currentBlinkState = digitalRead(BLINK_BTN);


  if (lastBlinkState == HIGH && currentBlinkState == LOW) {
    doBlink();


    // full blink always returns to neutral/open
    halfMode = false;
    lid.write(OPEN);
    currentPos = OPEN;
  }


  lastBlinkState = currentBlinkState;
}


void doBlink() {
  // start from wherever the lid currently is
  for (int pos = currentPos; pos <= CLOSED; pos++) {
    lid.write(pos);
    currentPos = pos;
    delay(BLINK_CLOSE_SPEED);
  }


  delay(BLINK_PAUSE);


  for (int pos = CLOSED; pos >= OPEN; pos--) {
    lid.write(pos);
    currentPos = pos;
    delay(BLINK_OPEN_SPEED);
  }
}


// ---------------- LED TOGGLE ----------------
void handleLedToggle() {
  bool currentLedState = digitalRead(LED_BTN);


  if (lastLedState == HIGH && currentLedState == LOW) {
    if (!pulseEnabled && !fadeOutActive) {
      // turn pulse ON
      pulseEnabled = true;
      fadeOutActive = false;


      if (currentBrightness < MIN_BRIGHTNESS) {
        currentBrightness = MIN_BRIGHTNESS;
      }


      targetBrightness = random(MIN_BRIGHTNESS, MAX_BRIGHTNESS + 1);
      targetHoldTime = random(140, 320);
      lastTargetChange = millis();
    }
    else if (pulseEnabled) {
      // turn pulse OFF with fade
      pulseEnabled = false;
      fadeOutActive = true;
    }
  }


  lastLedState = currentLedState;
}


// ---------------- ORGANIC LED ----------------
void updateOrganicLED() {
  unsigned long now = millis();


  if (pulseEnabled) {
    // pick a new target brightness more often
    if (now - lastTargetChange >= targetHoldTime) {
      targetBrightness = random(MIN_BRIGHTNESS, MAX_BRIGHTNESS + 1);
      targetHoldTime = random(140, 320);   // about 2x faster than before
      lastTargetChange = now;
    }


    if (now - lastLedUpdate >= LED_UPDATE_INTERVAL) {
      lastLedUpdate = now;


      currentBrightness += (targetBrightness - currentBrightness) * smoothing;


      if (currentBrightness < MIN_BRIGHTNESS) currentBrightness = MIN_BRIGHTNESS;
      if (currentBrightness > MAX_BRIGHTNESS) currentBrightness = MAX_BRIGHTNESS;


      analogWrite(LED_PIN, (int)currentBrightness);
    }
  }
  else if (fadeOutActive) {
    if (now - lastLedUpdate >= LED_UPDATE_INTERVAL) {
      lastLedUpdate = now;


      currentBrightness -= fadeOutAmount;


      if (currentBrightness <= 0) {
        currentBrightness = 0;
        fadeOutActive = false;
      }


      analogWrite(LED_PIN, (int)currentBrightness);
    }
  }
}
