#include <FastLED.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLESecurity.h>
#include <BLE2902.h>
#include <driver/ledc.h>

// BLE UUIDs (shared across props)
#define SERVICE_UUID "09d2abe8-30ec-4519-86ff-ba0cbaf79160"
#define CHARACTERISTIC_UUID "102d8bfe-dc7b-44d2-8cfe-0e09f2ee6107"

// LED configuration
#define DATA_PIN 5
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
#define NUM_LEDS 9
#define BRIGHTNESS 255
#define FRAMES_PER_SECOND 120
CRGB leds[NUM_LEDS];

// MOSFET configuration
#define COIL_PIN 18
#define MOTOR_PIN 19
#define COIL_LEDC_CHANNEL 0
#define MOTOR_LEDC_CHANNEL 1
#define LEDC_FREQ 1000
#define LEDC_RESOLUTION 10

// Timing configuration
#define RAMP_DURATION_MS 1000
#define AUTO_SHUTOFF_MS 30000

// BLE configuration
#define PASSKEY 123456

// State machine
enum FlamerState {
  STATE_BOOT = 0,
  STATE_IDLE,
  STATE_RAMP_UP,
  STATE_FLAMING,
  STATE_RAMP_DOWN
};

// Global state
static FlamerState gState = STATE_BOOT;
static FlamerState gPrevState = STATE_BOOT;

// BLE
static BLEServer *pServer = nullptr;
static BLECharacteristic *pCharacteristic = nullptr;
static bool deviceConnected = false;
static bool oldDeviceConnected = false;
static uint8_t bleCommand = 0;
static bool bleCommandPending = false;

// Ramp state
static unsigned long rampStartTime = 0;
static uint8_t rampValue = 0;

// Auto-shutoff
static unsigned long flamingStartTime = 0;

// Fire effect
static uint16_t noiseOffset = 0;

// FastLED fire palette
static CRGBPalette16 firePalette = HeatColors_p;

// Function prototypes
static void notifyStateChange();
static void bleSecuritySetup();
static void setupLedc();
static void setMosfetDuty(uint8_t value);
static void fireEffect();
static void bootAnimation();

// BLE write callback — receives commands from armDisplay
class FlamerWriteCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChr) {
    if (pChr->getLength() >= 1) {
      bleCommand = pChr->getData()[0];
      bleCommandPending = true;
      Serial.print("BLE command received: ");
      Serial.println(bleCommand);
    }
  }
};

class SecurityCallback : public BLESecurityCallbacks {
  uint32_t onPassKeyRequest() {
    return PASSKEY;
  }
  void onPassKeyNotify(uint32_t pass_key) {}
  bool onConfirmPIN(uint32_t pass_key) {
    return true;
  }
  bool onSecurityRequest() {
    return true;
  }
  void onAuthenticationComplete(esp_ble_auth_cmpl_t cmpl) {
    if (cmpl.success) {
      Serial.println("  - BLE auth success");
      deviceConnected = true;
      BLEDevice::startAdvertising();
    } else {
      Serial.println("  - BLE auth failure");
      if (pServer) {
        pServer->removePeerDevice(pServer->getConnId(), true);
      }
    }
  }
};

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pSrv) {
    deviceConnected = true;
    Serial.println("BLE device connected");
  }
  void onDisconnect(BLEServer* pSrv) {
    deviceConnected = false;
    Serial.println("BLE device disconnected");
  }
};

static void notifyStateChange() {
  if (gPrevState != gState) {
    uint8_t stateVal = (uint8_t)gState;
    Serial.print("State: ");
    switch (gState) {
      case STATE_BOOT: Serial.println("BOOT"); break;
      case STATE_IDLE: Serial.println("IDLE"); break;
      case STATE_RAMP_UP: Serial.println("RAMP_UP"); break;
      case STATE_FLAMING: Serial.println("FLAMING"); break;
      case STATE_RAMP_DOWN: Serial.println("RAMP_DOWN"); break;
    }
    pCharacteristic->setValue(&stateVal, 1);
    pCharacteristic->notify();
    gPrevState = gState;
  }
}

static void setupLedc() {
  ledcSetup(COIL_LEDC_CHANNEL, LEDC_FREQ, LEDC_RESOLUTION);
  ledcSetup(MOTOR_LEDC_CHANNEL, LEDC_FREQ, LEDC_RESOLUTION);
  ledcAttachPin(COIL_PIN, COIL_LEDC_CHANNEL);
  ledcAttachPin(MOTOR_PIN, MOTOR_LEDC_CHANNEL);
  ledcWrite(COIL_LEDC_CHANNEL, 0);
  ledcWrite(MOTOR_LEDC_CHANNEL, 0);
}

static void setMosfetDuty(uint8_t value) {
  uint32_t duty = (uint32_t)value * 1024 / 255;
  ledcWrite(COIL_LEDC_CHANNEL, duty);
  ledcWrite(MOTOR_LEDC_CHANNEL, duty);
}

static void fireEffect() {
  noiseOffset += 20;
  for (int i = 0; i < NUM_LEDS; i++) {
    uint16_t x = (i * 30) + noiseOffset;
    uint8_t val = inoise8(x, noiseOffset / 2);
    leds[i] = ColorFromPalette(firePalette, val, 255, LINEARBLEND);
  }
}

static void bootAnimation() {
  for (int i = NUM_LEDS - 1; i >= 0; i--) {
    leds[i] = CRGB::Orange;
    FastLED.show();
    delay(150);
  }
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB::Black;
    FastLED.show();
    delay(50);
  }
}

static void bleSetup() {
  BLEDevice::init("TechPriest_Flamer");
  BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);
  BLEDevice::setSecurityCallbacks(new SecurityCallback());

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE
  );

  pCharacteristic->setAccessPermissions(ESP_GATT_PERM_READ_ENCRYPTED | ESP_GATT_PERM_WRITE_ENCRYPTED);
  pCharacteristic->setCallbacks(new FlamerWriteCallback());

  BLEDescriptor *pDescr = new BLEDescriptor((uint16_t)0x2901);
  pDescr->setValue("Flamer_Control");
  pCharacteristic->addDescriptor(pDescr);

  BLE2902 *pBLE2902 = new BLE2902();
  pBLE2902->setNotifications(true);
  pCharacteristic->addDescriptor(pBLE2902);

  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);
  BLEDevice::startAdvertising();

  bleSecuritySetup();

  Serial.println("BLE advertising as TechPriest_Flamer");
}

static void bleSecuritySetup() {
  esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND;
  esp_ble_io_cap_t iocap = ESP_IO_CAP_OUT;
  uint8_t key_size = 16;
  uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  uint32_t passkey = PASSKEY;
  uint8_t auth_option = ESP_BLE_ONLY_ACCEPT_SPECIFIED_AUTH_DISABLE;
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_STATIC_PASSKEY, &passkey, sizeof(uint32_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_ONLY_ACCEPT_SPECIFIED_SEC_AUTH, &auth_option, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));
}

void setup() {
  Serial.begin(115200);
  esp_log_level_set("*", ESP_LOG_DEBUG);

  setupLedc();
  bleSetup();

  FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS)
    .setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();

  notifyStateChange();
  bootAnimation();

  gState = STATE_IDLE;
  notifyStateChange();
}

void loop() {
  unsigned long now = millis();

  // BLE re-advertising after disconnect
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    pServer->startAdvertising();
    Serial.println("BLE re-advertising");
    oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }

  // Process BLE command
  if (bleCommandPending) {
    bleCommandPending = false;
    if (bleCommand == 1 && (gState == STATE_IDLE || gState == STATE_BOOT)) {
      gState = STATE_RAMP_UP;
      rampStartTime = now;
      rampValue = 0;
    } else if (bleCommand == 0 && (gState == STATE_RAMP_UP || gState == STATE_FLAMING)) {
      gState = STATE_RAMP_DOWN;
      rampStartTime = now;
      rampValue = 255;
    }
    notifyStateChange();
  }

  // State machine
  switch (gState) {
    case STATE_BOOT:
      break;

    case STATE_IDLE:
      fill_solid(leds, NUM_LEDS, CRGB::Black);
      setMosfetDuty(0);
      break;

    case STATE_RAMP_UP: {
      unsigned long elapsed = now - rampStartTime;
      uint8_t progress = (elapsed < RAMP_DURATION_MS) ? (uint8_t)(elapsed * 255 / RAMP_DURATION_MS) : 255;
      setMosfetDuty(progress);
      FastLED.setBrightness(progress);
      fireEffect();
      FastLED.show();
      if (elapsed >= RAMP_DURATION_MS) {
        gState = STATE_FLAMING;
        flamingStartTime = now;
        FastLED.setBrightness(BRIGHTNESS);
        notifyStateChange();
      }
      break;
    }

    case STATE_FLAMING:
      fireEffect();
      setMosfetDuty(255);
      FastLED.show();
      if (now - flamingStartTime >= AUTO_SHUTOFF_MS) {
        gState = STATE_RAMP_DOWN;
        rampStartTime = now;
        rampValue = 255;
        Serial.println("Auto-shutoff triggered (30s timeout)");
        notifyStateChange();
      }
      break;

    case STATE_RAMP_DOWN: {
      unsigned long elapsed = now - rampStartTime;
      uint8_t progress = (elapsed < RAMP_DURATION_MS) ? (uint8_t)(255 - elapsed * 255 / RAMP_DURATION_MS) : 0;
      setMosfetDuty(progress);
      FastLED.setBrightness(progress);
      fireEffect();
      FastLED.show();
      if (elapsed >= RAMP_DURATION_MS) {
        gState = STATE_IDLE;
        fill_solid(leds, NUM_LEDS, CRGB::Black);
        FastLED.setBrightness(BRIGHTNESS);
        setMosfetDuty(0);
        FastLED.show();
        notifyStateChange();
      }
      break;
    }
  }
}
