#include <FastLED.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLESecurity.h>
#include <BLE2902.h>

// UUIDs for BLE service and characteristic
#define SERVICE_UUID "09d2abe8-30ec-4519-86ff-ba0cbaf79160"
#define CHARACTERISTIC_UUID "102d8bfe-dc7b-44d2-8cfe-0e09f2ee6107"

// LED configuration
#define DATA_PIN 5
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
#define NUM_LEDS 9
#define BRIGHTNESS 120
#define FRAMES_PER_SECOND 120
CRGB leds[NUM_LEDS];
uint8_t breathBrightness = 100;

// Button configuration
#define BUTTON_PIN 6
bool lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;
unsigned long buttonPressTime = 0;

BLEServer *pServer;
BLECharacteristic *pCharacteristic;
BLEDescriptor *pDescr;
BLE2902 *pBLE2902;
volatile bool deviceConnected = false;
volatile bool oldDeviceConnected = false;

volatile uint8_t gCurrentPatternNumber = 0;
volatile uint8_t prev_gCurrentPatternNumber = 255; // Initialized to a different value
uint8_t overchargingTransitionStep = 0;
int shootingStep = 0;
unsigned long shootingLastTime = 0;
const unsigned long shootingInterval = 20;

typedef void (*SimplePatternList[])();

unsigned long previousMillis = 0;
const long interval = 1000 / FRAMES_PER_SECOND;

// PIN for BLE pairing - unique per device from MAC address
uint32_t getDevicePasskey() {
  uint8_t base_mac[6];
  esp_read_mac(base_mac, ESP_MAC_WIFI_STA);
  return (base_mac[3] << 16) | (base_mac[4] << 8) | base_mac[5];
}

// Function prototypes
void idle();
void charging();
void overcharging();
void shooting();
void addGlitter(fract8 chanceOfGlitter);
void startup();
void intToPrint(uint8_t i);
void notifyPatternChange();
void bleSecuritySetup();


// LED patterns (Idle, Charging, Overcharging, Shooting)
void idle() {
  static int8_t breathDirection = 1;
  breathBrightness += breathDirection * 1;
  if (breathBrightness <= 100 || breathBrightness >= 254) {
    breathDirection *= -1;
  }
  fill_solid(leds, NUM_LEDS, CHSV(160, 255, breathBrightness));
}

void charging() {
  fill_solid(leds, NUM_LEDS, CRGB::Blue);
  addGlitter(80);
}

void overcharging() {
  for (int i = 0; i < NUM_LEDS; i++) {
    uint8_t blendAmount = scale8(overchargingTransitionStep, 255);
    leds[i] = blend(CRGB::Blue, CRGB::Red, blendAmount);
  }
  overchargingTransitionStep = qadd8(overchargingTransitionStep, 1);
  addGlitter(80);
  if (overchargingTransitionStep >= 255) {
    overchargingTransitionStep = 255;
  }
}

void shooting() {
  unsigned long now = millis();
  if (now - shootingLastTime >= shootingInterval) {
    shootingLastTime = now;
    if (shootingStep >= 0 && shootingStep < NUM_LEDS) {
      leds[NUM_LEDS - 1 - shootingStep] = CRGB::Black;
      FastLED.show();
      shootingStep++;
    }
  }
}

void addGlitter(fract8 chanceOfGlitter) {
  if (random8() < chanceOfGlitter) {
    leds[random16(NUM_LEDS)] += CRGB::White;
  }
}

void startup() {
  for (int i = NUM_LEDS - 1; i >= 0; i--) {
    leds[i] = CRGB::Blue;
    FastLED.show();
    delay(200);  // Adjust this delay to control the startup speed
  }
}

SimplePatternList gPatterns = {idle, charging, overcharging, shooting};


class SecurityCallback : public BLESecurityCallbacks {
  uint32_t onPassKeyRequest(){
    return getDevicePasskey();
  }

  void onPassKeyNotify(uint32_t pass_key){}

  bool onConfirmPIN(uint32_t pass_key){
    return true;
  }

  bool onSecurityRequest(){
    return true;
  }

  void onAuthenticationComplete(esp_ble_auth_cmpl_t cmpl){
    if(cmpl.success){
      Serial.println("   - SecurityCallback - Authentication Success");
      deviceConnected = true;
      BLEDevice::startAdvertising();
    } else {
      Serial.println("   - SecurityCallback - Authentication Failure*");
      if (pServer) {
        pServer->removePeerDevice(pServer->getConnId(), true);
      }
    }
  }
};

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pSrv) {
    deviceConnected = true;
    Serial.println("Device connected");
  }

  void onDisconnect(BLEServer* pSrv) {
    deviceConnected = false;
    Serial.println("Device disconnected");
  }
};

// BLE write callback — receives commands from armDisplay
class PlasmaWriteCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChr) {
    if (pChr->getLength() >= 1) {
      uint8_t cmd = pChr->getData()[0];
      if (cmd >= 0 && cmd <= 3) {
        gCurrentPatternNumber = cmd;
        if (cmd == 2) {
          overchargingTransitionStep = 0;
        }
        if (cmd == 3) {
          shootingStep = 0;
          shootingLastTime = millis();
        }
        Serial.print("BLE command received - Pattern: ");
        Serial.println(cmd);
      }
    }
  }
};

void notifyPatternChange() {
  if (prev_gCurrentPatternNumber != gCurrentPatternNumber) {
    /*
    Serial.print("Pattern changed from ");
    Serial.print(prev_gCurrentPatternNumber);
    Serial.print(" to ");
    Serial.println(gCurrentPatternNumber);
    */
    intToPrint(gCurrentPatternNumber);
    uint8_t val = gCurrentPatternNumber;
    pCharacteristic->setValue(&val, 1);
    pCharacteristic->notify();
    prev_gCurrentPatternNumber = gCurrentPatternNumber;
  }
}

void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.begin(115200);

  // Set logging level
  esp_log_level_set("*", ESP_LOG_DEBUG);

  // Initialize BLE
  BLEDevice::init("Plasma_Pistol");
  BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);
  BLEDevice::setSecurityCallbacks(new SecurityCallback());

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create a BLE service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE characteristic (bidirectional: notify + write)
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE
  );

  pCharacteristic->setAccessPermissions(ESP_GATT_PERM_READ_ENCRYPTED | ESP_GATT_PERM_WRITE_ENCRYPTED);
  pCharacteristic->setCallbacks(new PlasmaWriteCallback());

  // Create descriptors for the characteristic
  pDescr = new BLEDescriptor((uint16_t)0x2901);
  pDescr->setValue("Plasma_Gun_State");
  pCharacteristic->addDescriptor(pDescr);

  pBLE2902 = new BLE2902();
  pBLE2902->setNotifications(true);
  pCharacteristic->addDescriptor(pBLE2902);

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();

  // Setup BLE Security
  bleSecuritySetup();

  Serial.println("Waiting for a client connection to notify...");
  delay(100); // brief delay for BLE recovery

  // Initialize FastLED
  FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);
  fill_solid(leds, NUM_LEDS, CRGB::Black); // start up
  notifyPatternChange();
  startup();
}

void loop() {
  int reading = !digitalRead(BUTTON_PIN);

  // Button press detection with debouncing
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // Only process if the button state has actually changed
    if (reading != lastButtonState) {
      lastButtonState = reading;

      if (reading == HIGH) { // Button pressed
        // Record when button was pressed
        buttonPressTime = millis();

        // Start charging when button is pressed
        gCurrentPatternNumber = 1; // Charging mode
        Serial.println("Button pressed - Charging");
      } else { // Button released
        // Calculate how long the button was held
        unsigned long buttonHoldTime = millis() - buttonPressTime;
        Serial.print("Button released after ");
        Serial.print(buttonHoldTime);
        Serial.println(" ms");

        if (buttonHoldTime > 2000) {
          // Long press - go to overcharging
          gCurrentPatternNumber = 2; // Overcharging mode
          overchargingTransitionStep = 0;
          Serial.println("Long press - Overcharging");
        } else {
          // Short press - go to shooting
          gCurrentPatternNumber = 3; // Shooting mode
          shootingStep = 0;
          shootingLastTime = millis();
          Serial.println("Short press - Shooting");
        }
        notifyPatternChange();
      }
    }
  }

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    // Call the appropriate pattern function based on current state
    gPatterns[gCurrentPatternNumber]();
    FastLED.show();

    if (gCurrentPatternNumber == 3 && shootingStep >= NUM_LEDS) {
      fill_solid(leds, NUM_LEDS, CRGB::Black);
      FastLED.show();
      gCurrentPatternNumber = 0;
      Serial.println("Shooting complete - Returning to Idle");
    }
  }

  // Re-advertising after disconnection
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    pServer->startAdvertising();  // restart advertising
    Serial.println("start advertising");
    oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }
}



void intToPrint(uint8_t i){
  Serial.print("Pattern: ");
  switch (i){
    case 0:
      Serial.println("idle");
      break;
    case 1:
      Serial.println("charging");
      break;
    case 2:
      Serial.println("overcharging");
      break;
    case 3:
      Serial.println("shooting");
      break;
  }
}

void bleSecuritySetup(){
  esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND;
  esp_ble_io_cap_t iocap = ESP_IO_CAP_OUT;
  uint8_t key_size = 16;
  uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  uint32_t passkey = getDevicePasskey();
  uint8_t auth_option = ESP_BLE_ONLY_ACCEPT_SPECIFIED_AUTH_DISABLE;
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_STATIC_PASSKEY, &passkey, sizeof(uint32_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_ONLY_ACCEPT_SPECIFIED_SEC_AUTH, &auth_option, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));
}

// This line must be at the end, after all functions are defined
