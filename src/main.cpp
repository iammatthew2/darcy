#include <Arduino.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_sleep.h>

#include <cstring>

// Optional sender MAC filter. Keep all zeros to accept packets from any sender.
static uint8_t DARYL_MAC[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// C3-safe choices for XIAO ESP32-C3 test wiring.
static constexpr uint8_t PAN_SERVO_PIN = 4;
static constexpr uint8_t EYELID_SERVO_PIN = 5;
static constexpr uint8_t SERVO_POWER_KILL_PIN =
    6;  // HIGH cuts servo power (Adafruit #1400 KILL pin)
static constexpr uint8_t SERVO_POWER_ON_PIN =
    7;  // Pulse HIGH to restore servo power (simulate button press via S9013)
static constexpr uint32_t SERVO_POWER_ON_PULSE_MS = 250;

static constexpr int SERVO_MIN_DEG = 0;
static constexpr int SERVO_MAX_DEG = 180;
static constexpr int PAN_SERVO_DEFAULT_DEG = 90;
static constexpr int PAN_SERVO_ENCODER_FAST_DEG_PER_STEP = 13;
static constexpr int PAN_SERVO_ENCODER_SLOW_DEG_PER_STEP = 5;
static constexpr int EYELID_OPEN_DEG = 0;
static constexpr int EYELID_HALF_DEG = 45;
static constexpr int EYELID_CLOSED_DEG = 90;

enum EyelidState { EYELID_OPEN = 0, EYELID_CLOSED = 1, EYELID_HALF = 2 };
static constexpr int SLEEP_POSE_PAN_STEP_MS = 10;
static constexpr int SLEEP_POSE_EYELID_STEP_MS = 6;
static constexpr int BLINK_CLOSE_STEP_MS = 1;
static constexpr int BLINK_OPEN_STEP_MS = 1;
static constexpr uint32_t BLINK_PAUSE_MS = 180;
static constexpr uint32_t FAST_MODE_REBOUND_PAUSE_MS = 350;
static constexpr int FAST_MODE_REBOUND_DEG = 18;
static constexpr uint32_t LINK_TIMEOUT_MS = 4000;
static constexpr uint8_t WAKE_PIN = 3;  // GPIO3: LOW wakes from deep sleep
static constexpr uint32_t BOARD_SLEEP_DELAY_MS =
    300000;  // Deep sleep 5min after servo power kill
static constexpr uint8_t BUTTON_IDX_BLINK = 1;
static constexpr uint8_t BUTTON_IDX_GAIN_TOGGLE = 4;
static constexpr uint8_t BUTTON_IDX_SLEEP = 5;
static constexpr uint8_t VALID_BUTTON_BITS = 0b00111111;  // Bits 0-5 only

struct __attribute__((packed)) RemotePacket {
  uint32_t seq;
  uint32_t uptimeMs;
  int32_t encoderPosition;
  int16_t encoderDelta;
  uint8_t buttonsMask;
  uint8_t encoderPressed;
};

Servo gPanServo;
Servo gEyelidServo;
volatile bool gPacketReady = false;
volatile RemotePacket gLatestPacket = {};
volatile uint8_t gSourceMac[6] = {0};

int gPanServoAngle = PAN_SERVO_DEFAULT_DEG;
int gEyelidServoAngle = EYELID_CLOSED_DEG;
uint32_t gLastRxMs = 0;
bool gHasLastEncoderPosition = false;
int32_t gLastEncoderPosition = 0;
uint8_t gPrevButtonsMask = 0;
bool gUseSlowEncoderGain = false;
EyelidState gEyelidState = EYELID_CLOSED;
int gLastEncoderDir = 0;
uint32_t gLastEncoderFastMoveMs = 0;
bool gReboundApplied = false;
volatile bool gDemoAbort = false;
bool gServoPowerKilled = false;
uint32_t gBoardSleepAt =
    0;  // millis() target for deep sleep; 0 = not scheduled

static bool isZeroMac(const uint8_t* mac) {
  for (size_t i = 0; i < 6; ++i) {
    if (mac[i] != 0) {
      return false;
    }
  }
  return true;
}

static bool macEquals(const uint8_t* a, const uint8_t* b) {
  return memcmp(a, b, 6) == 0;
}

static void printMac(const uint8_t* mac) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X", mac[0],
           mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.print(macStr);
}

static void setPanServoAngle(int angle) {
  gPanServoAngle = constrain(angle, SERVO_MIN_DEG, SERVO_MAX_DEG);
  gPanServo.write(gPanServoAngle);
}

static void setEyelidServoAngle(int angle) {
  gEyelidServoAngle = constrain(angle, SERVO_MIN_DEG, SERVO_MAX_DEG);
  gEyelidServo.write(gEyelidServoAngle);
}

static void movePanSmooth(int target, int stepMs) {
  int step = (target > gPanServoAngle) ? 1 : -1;
  while (gPanServoAngle != target) {
    setPanServoAngle(gPanServoAngle + step);
    delay(stepMs);
  }
}

static void moveEyelidSmooth(int target, int stepMs) {
  int step = (target > gEyelidServoAngle) ? 1 : -1;
  while (gEyelidServoAngle != target) {
    setEyelidServoAngle(gEyelidServoAngle + step);
    delay(stepMs);
  }
}

// Interruptible variants — return false if a packet arrived mid-move.
static bool movePanSmoothI(int target, int stepMs) {
  int step = (target > gPanServoAngle) ? 1 : -1;
  while (gPanServoAngle != target) {
    if (gDemoAbort) return false;
    setPanServoAngle(gPanServoAngle + step);
    delay(stepMs);
  }
  return true;
}

static bool moveEyelidSmoothI(int target, int stepMs) {
  int step = (target > gEyelidServoAngle) ? 1 : -1;
  while (gEyelidServoAngle != target) {
    if (gDemoAbort) return false;
    setEyelidServoAngle(gEyelidServoAngle + step);
    delay(stepMs);
  }
  return true;
}

static bool demoHold(uint32_t ms) {
  uint32_t start = millis();
  while (millis() - start < ms) {
    if (gDemoAbort) return false;
    delay(1);
  }
  return true;
}

// Abort the demo and reset the link timer so the incoming packet is handled cleanly.
#define DEMO_STEP(expr) do { if (!(expr)) { gLastRxMs = millis(); return; } } while (0)

static void demoSequence() {
  Serial.println("[DEMO] start");
  gDemoAbort = false;

  // --- Phase 1: Wake up ---
  if (gEyelidState != EYELID_OPEN) {
    DEMO_STEP(moveEyelidSmoothI(EYELID_OPEN_DEG, 12));
    gEyelidState = EYELID_OPEN;
  }
  DEMO_STEP(demoHold(800));

  // --- Phase 2: Lazy look around ---
  DEMO_STEP(movePanSmoothI(65, 10));
  DEMO_STEP(demoHold(600));
  DEMO_STEP(movePanSmoothI(115, 10));
  DEMO_STEP(demoHold(400));
  DEMO_STEP(movePanSmoothI(PAN_SERVO_DEFAULT_DEG, 10));
  DEMO_STEP(demoHold(500));

  // --- Phase 3: Something catches attention (snap right, linger, drift back) ---
  DEMO_STEP(movePanSmoothI(145, 3));
  DEMO_STEP(demoHold(300));
  DEMO_STEP(movePanSmoothI(105, 12));
  DEMO_STEP(demoHold(200));
  DEMO_STEP(movePanSmoothI(PAN_SERVO_DEFAULT_DEG, 8));

  // --- Phase 4: Blink ---
  for (int pos = gEyelidServoAngle; pos <= EYELID_CLOSED_DEG; pos++) {
    if (gDemoAbort) { gLastRxMs = millis(); return; }
    setEyelidServoAngle(pos);
    delay(BLINK_CLOSE_STEP_MS);
  }
  DEMO_STEP(demoHold(BLINK_PAUSE_MS));
  for (int pos = EYELID_CLOSED_DEG; pos >= EYELID_OPEN_DEG; pos--) {
    if (gDemoAbort) { gLastRxMs = millis(); return; }
    setEyelidServoAngle(pos);
    delay(BLINK_OPEN_STEP_MS);
  }
  gEyelidState = EYELID_OPEN;
  DEMO_STEP(demoHold(400));

  // --- Phase 5: Drowsy — half-lid, slow wander ---
  DEMO_STEP(moveEyelidSmoothI(EYELID_HALF_DEG, 4));
  gEyelidState = EYELID_HALF;
  DEMO_STEP(movePanSmoothI(70, 15));
  DEMO_STEP(demoHold(700));
  DEMO_STEP(movePanSmoothI(112, 15));
  DEMO_STEP(demoHold(500));
  DEMO_STEP(movePanSmoothI(PAN_SERVO_DEFAULT_DEG, 12));
  DEMO_STEP(demoHold(800));

  // --- Phase 6: Snap to attention ---
  DEMO_STEP(moveEyelidSmoothI(EYELID_OPEN_DEG, 4));
  gEyelidState = EYELID_OPEN;
  DEMO_STEP(movePanSmoothI(30, 3));
  DEMO_STEP(demoHold(150));
  DEMO_STEP(movePanSmoothI(150, 3));
  DEMO_STEP(demoHold(150));
  DEMO_STEP(movePanSmoothI(PAN_SERVO_DEFAULT_DEG, 5));
  DEMO_STEP(demoHold(300));

  // --- Phase 7: Settle ---
  DEMO_STEP(movePanSmoothI(80, 12));
  DEMO_STEP(demoHold(400));
  DEMO_STEP(movePanSmoothI(95, 12));
  DEMO_STEP(demoHold(600));
  DEMO_STEP(movePanSmoothI(PAN_SERVO_DEFAULT_DEG, 8));

  Serial.println("[DEMO] end");
}

static void doBlink() {
  // Close from wherever the lid currently is
  for (int pos = gEyelidServoAngle; pos <= EYELID_CLOSED_DEG; pos++) {
    setEyelidServoAngle(pos);
    delay(BLINK_CLOSE_STEP_MS);
  }
  delay(BLINK_PAUSE_MS);
  // Reopen fully
  for (int pos = EYELID_CLOSED_DEG; pos >= EYELID_OPEN_DEG; pos--) {
    setEyelidServoAngle(pos);
    delay(BLINK_OPEN_STEP_MS);
  }
  gEyelidState = EYELID_OPEN;
}

static void toggleEyelid() {
  if (gEyelidState == EYELID_OPEN) {
    moveEyelidSmooth(EYELID_CLOSED_DEG, BLINK_CLOSE_STEP_MS);
    gEyelidState = EYELID_CLOSED;
  } else if (gEyelidState == EYELID_CLOSED) {
    moveEyelidSmooth(EYELID_HALF_DEG, BLINK_OPEN_STEP_MS);
    gEyelidState = EYELID_HALF;
  } else {
    moveEyelidSmooth(EYELID_OPEN_DEG, BLINK_OPEN_STEP_MS);
    gEyelidState = EYELID_OPEN;
  }
}


static void applySleepPose() {
  movePanSmooth(PAN_SERVO_DEFAULT_DEG, SLEEP_POSE_PAN_STEP_MS);
  moveEyelidSmooth(EYELID_CLOSED_DEG, SLEEP_POSE_EYELID_STEP_MS);
  gEyelidState = EYELID_CLOSED;
}

static void killServoPower() {
  delay(300);  // Allow servos to reach sleep pose before cutting power
  digitalWrite(SERVO_POWER_KILL_PIN, HIGH);
  gServoPowerKilled = true;
  gBoardSleepAt = millis() + BOARD_SLEEP_DELAY_MS;
  Serial.print("[POWER] Servo power killed — deep sleep in ");
  Serial.print(BOARD_SLEEP_DELAY_MS / 1000);
  Serial.println("s");
}

static void restoreServoPower() {
  gBoardSleepAt = 0;
  digitalWrite(SERVO_POWER_KILL_PIN, LOW);
  delay(200);  // Let KILL de-assert before simulating button press
  digitalWrite(SERVO_POWER_ON_PIN, HIGH);
  delay(SERVO_POWER_ON_PULSE_MS);
  digitalWrite(SERVO_POWER_ON_PIN, LOW);
  gServoPowerKilled = false;
  Serial.println("[POWER] Servo power restored");
}

static void enterDeepSleep() {
  Serial.println("[SLEEP] Entering deep sleep — GPIO3 LOW to wake");
  Serial.flush();
  esp_deep_sleep_enable_gpio_wakeup(1ULL << WAKE_PIN, ESP_GPIO_WAKEUP_GPIO_LOW);
  esp_deep_sleep_start();
}

static void onSleepSignal() {
  Serial.println("[SLEEP] Daryl entering deep sleep");
  applySleepPose();
  killServoPower();
}

static void onDataRecv(const esp_now_recv_info_t* info, const uint8_t* data,
                       int len) {
  if (len != static_cast<int>(sizeof(RemotePacket))) {
    return;
  }

  if (!isZeroMac(DARYL_MAC) && !macEquals(info->src_addr, DARYL_MAC)) {
    return;
  }

  memcpy((void*)&gLatestPacket, data, sizeof(RemotePacket));
  memcpy((void*)gSourceMac, info->src_addr, sizeof(gSourceMac));
  gPacketReady = true;
  if (gLatestPacket.buttonsMask != 0 || gLatestPacket.encoderDelta != 0 ||
      gLatestPacket.encoderPressed != 0) {
    gDemoAbort = true;
  }
}

static bool initEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return false;
  }

  esp_now_register_recv_cb(onDataRecv);

  Serial.print("Darcy MAC: ");
  Serial.println(WiFi.macAddress());
  if (!isZeroMac(DARYL_MAC)) {
    Serial.print("Accepting only Daryl MAC: ");
    printMac(DARYL_MAC);
    Serial.println();
  } else {
    Serial.println("Accepting sender from any MAC (filter disabled)");
  }
  return true;
}

static void initServos() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  gPanServo.setPeriodHertz(50);
  gPanServo.attach(PAN_SERVO_PIN, 1000, 2000);
  setPanServoAngle(PAN_SERVO_DEFAULT_DEG);

  gEyelidServo.setPeriodHertz(50);
  gEyelidServo.attach(EYELID_SERVO_PIN, 1000, 2000);
  setEyelidServoAngle(EYELID_CLOSED_DEG);

  Serial.print("Pan servo attached to GPIO");
  Serial.println(PAN_SERVO_PIN);
  Serial.print("Eyelid servo attached to GPIO");
  Serial.println(EYELID_SERVO_PIN);
}

void setup() {
  Serial.begin(115200);
  delay(800);

  Serial.println();
  Serial.println("Darcy ESP-NOW receiver booting...");

  // Ensure servo power is on at boot
  pinMode(SERVO_POWER_KILL_PIN, OUTPUT);
  digitalWrite(SERVO_POWER_KILL_PIN, LOW);
  pinMode(SERVO_POWER_ON_PIN, OUTPUT);
  digitalWrite(SERVO_POWER_ON_PIN, LOW);
  delay(200);
  digitalWrite(SERVO_POWER_ON_PIN, HIGH);
  delay(SERVO_POWER_ON_PULSE_MS);
  digitalWrite(SERVO_POWER_ON_PIN, LOW);
  Serial.println("[POWER] Boot power-on pulse sent");

  initServos();
  if (!initEspNow()) {
    Serial.println("Setup failed. Rebooting in 3 seconds...");
    delay(3000);
    ESP.restart();
  }

  Serial.print("Encoder gain mode: FAST (");
  Serial.print(PAN_SERVO_ENCODER_FAST_DEG_PER_STEP);
  Serial.println(" deg/step)");
}

void loop() {
  if (gPacketReady) {
    noInterrupts();
    RemotePacket packet = {};
    memcpy(&packet, (const void*)&gLatestPacket, sizeof(packet));
    uint8_t srcMac[6];
    memcpy(srcMac, (const void*)gSourceMac, sizeof(srcMac));
    gPacketReady = false;
    interrupts();

    gLastRxMs = millis();

    if (gServoPowerKilled) {
      restoreServoPower();
    }

    // Filter out-of-band signals: anything with bits 6-7 set
    if ((packet.buttonsMask & ~VALID_BUTTON_BITS) != 0) {
      onSleepSignal();
      return;
    }

    int32_t movementSteps = packet.encoderDelta;
    if (gHasLastEncoderPosition) {
      movementSteps = packet.encoderPosition - gLastEncoderPosition;
    }
    gLastEncoderPosition = packet.encoderPosition;
    gHasLastEncoderPosition = true;

    uint8_t prevButtonsMask = gPrevButtonsMask;

    bool demoNowPressed = (packet.buttonsMask & (1u << 0)) != 0;
    bool demoWasPressed = (prevButtonsMask & (1u << 0)) != 0;
    if (demoNowPressed && !demoWasPressed) {
      demoSequence();
      gLastRxMs = millis();
    }

    bool blinkAnimNowPressed = (packet.buttonsMask & (1u << 3)) != 0;
    bool blinkAnimWasPressed = (prevButtonsMask & (1u << 3)) != 0;
    if (blinkAnimNowPressed && !blinkAnimWasPressed) {
      doBlink();
    }

    bool blinkNowPressed = (packet.buttonsMask & (1u << BUTTON_IDX_BLINK)) != 0;
    bool blinkWasPressed = (prevButtonsMask & (1u << BUTTON_IDX_BLINK)) != 0;
    if (blinkNowPressed && !blinkWasPressed) {
      applySleepPose();
    }

    bool eyelidToggleNowPressed =
        (packet.buttonsMask & (1u << BUTTON_IDX_GAIN_TOGGLE)) != 0;
    bool eyelidToggleWasPressed =
        (prevButtonsMask & (1u << BUTTON_IDX_GAIN_TOGGLE)) != 0;
    if (eyelidToggleNowPressed && !eyelidToggleWasPressed) {
      toggleEyelid();
    }

    bool gainToggleNowPressed = (packet.buttonsMask & (1u << 2)) != 0;
    bool gainToggleWasPressed = (prevButtonsMask & (1u << 2)) != 0;
    if (gainToggleNowPressed && !gainToggleWasPressed) {
      gUseSlowEncoderGain = !gUseSlowEncoderGain;
      Serial.print("Encoder gain mode -> ");
      if (gUseSlowEncoderGain) {
        Serial.print("SLOW (");
        Serial.print(PAN_SERVO_ENCODER_SLOW_DEG_PER_STEP);
      } else {
        Serial.print("FAST (");
        Serial.print(PAN_SERVO_ENCODER_FAST_DEG_PER_STEP);
      }
      Serial.println(" deg/step)");
    }
    gPrevButtonsMask = packet.buttonsMask;

    if (movementSteps != 0) {
      int gain = gUseSlowEncoderGain ? PAN_SERVO_ENCODER_SLOW_DEG_PER_STEP
                                     : PAN_SERVO_ENCODER_FAST_DEG_PER_STEP;
      setPanServoAngle(gPanServoAngle + static_cast<int>(movementSteps) * gain);
      if (!gUseSlowEncoderGain) {
        gLastEncoderDir = (movementSteps > 0) ? 1 : -1;
        gLastEncoderFastMoveMs = millis();
        gReboundApplied = false;
      }
    }

    if (packet.encoderPressed) {
      setPanServoAngle(PAN_SERVO_DEFAULT_DEG);
    }

    Serial.print("rx seq=");
    Serial.print(packet.seq);
    Serial.print(" from=");
    printMac(srcMac);
    Serial.print(" buttons=0b");
    Serial.print(packet.buttonsMask, BIN);
    Serial.print(" encDelta=");
    Serial.print(packet.encoderDelta);
    Serial.print(" gainMode=");
    Serial.print(gUseSlowEncoderGain ? "slow" : "fast");
    Serial.print(" servo=");
    Serial.print(gPanServoAngle);
    Serial.print(" eyelid=");
    Serial.println(gEyelidServoAngle);
  }

  if (!gUseSlowEncoderGain && !gReboundApplied && gLastEncoderFastMoveMs != 0 &&
      (millis() - gLastEncoderFastMoveMs) > FAST_MODE_REBOUND_PAUSE_MS) {
    int reboundTarget = gPanServoAngle - gLastEncoderDir * FAST_MODE_REBOUND_DEG;
    Serial.print("[REBOUND] dir=");
    Serial.print(gLastEncoderDir);
    Serial.print(" from=");
    Serial.print(gPanServoAngle);
    Serial.print(" to=");
    Serial.println(reboundTarget);
    movePanSmooth(reboundTarget, 8);
    gReboundApplied = true;
  }

  if (gLastRxMs != 0 && (millis() - gLastRxMs) > LINK_TIMEOUT_MS) {
    gLastRxMs = 0;
    applySleepPose();
    killServoPower();
    Serial.println("link timeout -> sleep pose (pan centered, eyelid closed)");
  }

  if (gBoardSleepAt != 0 && millis() >= gBoardSleepAt) {
    enterDeepSleep();
  }

  delay(1);
}