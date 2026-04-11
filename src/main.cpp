#include <Arduino.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <esp_now.h>

#include <cstring>

// Optional sender MAC filter. Keep all zeros to accept packets from any sender.
static uint8_t DARYL_MAC[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// C3-safe choices for XIAO ESP32-C3 test wiring.
static constexpr uint8_t PAN_SERVO_PIN = 4;
static constexpr uint8_t EYELID_SERVO_PIN = 5;

static constexpr int SERVO_MIN_DEG = 0;
static constexpr int SERVO_MAX_DEG = 180;
static constexpr int PAN_SERVO_DEFAULT_DEG = 90;
static constexpr int PAN_SERVO_ENCODER_FAST_DEG_PER_STEP = 13;
static constexpr int PAN_SERVO_ENCODER_SLOW_DEG_PER_STEP = 5;
static constexpr int EYELID_OPEN_DEG = 0;
static constexpr int EYELID_CLOSED_DEG = 90;
static constexpr uint32_t LINK_TIMEOUT_MS = 4000;
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
int gEyelidServoAngle = EYELID_OPEN_DEG;
uint32_t gLastRxMs = 0;
bool gHasLastEncoderPosition = false;
int32_t gLastEncoderPosition = 0;
uint8_t gPrevButtonsMask = 0;
bool gUseSlowEncoderGain = false;
bool gEyelidClosed = false;

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

static void toggleEyelid() {
  if (gEyelidClosed) {
    setEyelidServoAngle(EYELID_OPEN_DEG);
    gEyelidClosed = false;
  } else {
    setEyelidServoAngle(EYELID_CLOSED_DEG);
    gEyelidClosed = true;
  }
}

static void applyButtonsToPanServo(uint8_t buttonsMask) {
  if (buttonsMask & (1u << 0)) {
    setPanServoAngle(0);
  } else if (buttonsMask & (1u << 2)) {
    setPanServoAngle(90);
  } else if (buttonsMask & (1u << 3)) {
    setPanServoAngle(135);
  }
}

static void applySleepPose() {
  // Sleep pose: center pan and close eyelid
  setPanServoAngle(PAN_SERVO_DEFAULT_DEG);
  setEyelidServoAngle(EYELID_CLOSED_DEG);
  gEyelidClosed = true;
}

static void onSleepSignal() {
  Serial.println("[SLEEP] Daryl entering deep sleep");
  applySleepPose();
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
  setEyelidServoAngle(EYELID_OPEN_DEG);

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

    bool blinkNowPressed = (packet.buttonsMask & (1u << BUTTON_IDX_BLINK)) != 0;
    bool blinkWasPressed = (prevButtonsMask & (1u << BUTTON_IDX_BLINK)) != 0;
    if (blinkNowPressed && !blinkWasPressed) {
      toggleEyelid();
    }

    bool toggleNowPressed =
        (packet.buttonsMask & (1u << BUTTON_IDX_GAIN_TOGGLE)) != 0;
    bool toggleWasPressed =
        (prevButtonsMask & (1u << BUTTON_IDX_GAIN_TOGGLE)) != 0;
    if (toggleNowPressed && !toggleWasPressed) {
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
    }

    applyButtonsToPanServo(packet.buttonsMask);

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

  if (gLastRxMs != 0 && (millis() - gLastRxMs) > LINK_TIMEOUT_MS) {
    gLastRxMs = 0;
    applySleepPose();
    Serial.println("link timeout -> sleep pose (pan centered, eyelid closed)");
  }

  delay(1);
}