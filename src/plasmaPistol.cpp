#include <FastLED.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLESecurity.h>
#include <BLE2902.h>
#include "esp_log.h"

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
bool buttonPressed = false;
bool chargingComplete = false;

BLEServer *pServer;
BLECharacteristic *pCharacteristic;
BLEDescriptor *pDescr;
BLE2902 *pBLE2902;
bool deviceConnected = false;
bool oldDeviceConnected = false;

uint8_t gCurrentPatternNumber = 0;
uint8_t prev_gCurrentPatternNumber = 255; // Initialized to a different value

typedef void (*SimplePatternList[])();

unsigned long previousMillis = 0;
const long interval = 1000; // 1 second

// PIN for BLE pairing
#define PASSKEY 123456 // 6-digit PIN

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
  static uint8_t transitionStep = 0;
  for (int i = 0; i < NUM_LEDS; i++) {
    uint8_t blendAmount = scale8(transitionStep, 255);
    leds[i] = blend(CRGB::Blue, CRGB::Red, blendAmount);
  }
  transitionStep = qadd8(transitionStep, 1);
  addGlitter(80);
  if (transitionStep >= 255) {
    transitionStep = 255;
  }
}

void shooting() {
  // Execute shooting animation (reverse turn off LEDs)
  for (int i = NUM_LEDS - 1; i >= 0; i--) {
    leds[i] = CRGB::Black;
    FastLED.show();
    delay(20);
  }
  // Ensure all LEDs are off
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
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
    return 000000;
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
      pServer->removePeerDevice(pServer->getConnId(), true);
    }
  }
};

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("Device connected");
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("Device disconnected");
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
    pCharacteristic->setValue(&gCurrentPatternNumber, 1);
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

  // Create a BLE characteristic
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );

  pCharacteristic->setAccessPermissions(ESP_GATT_PERM_READ_ENCRYPTED | ESP_GATT_PERM_WRITE_ENCRYPTED);

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
  delay(100); // 3 second delay for recovery

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
        buttonPressed = true;
        chargingComplete = false;

        // Start charging when button is pressed
        gCurrentPatternNumber = 1; // Charging mode
        Serial.println("Button pressed - Charging");
      } else { // Button released
        // Calculate how long the button was held
        unsigned long buttonHoldTime = millis() - buttonPressTime;
        Serial.print("Button released after ");
        Serial.print(buttonHoldTime);
        Serial.println(" ms");

        // Reset button state tracking
        buttonPressed = false;

        if (buttonHoldTime > 2000) {
          // Long press - go to overcharging
          gCurrentPatternNumber = 2; // Overcharging mode
          Serial.println("Long press - Overcharging");
        } else {
          // Short press - go to shooting
          gCurrentPatternNumber = 3; // Shooting mode
          Serial.println("Short press - Shooting");
        }
        notifyPatternChange();

        // Handle shooting state (non-blocking)
        if (gCurrentPatternNumber == 3) {
          // Shooting state - execute shooting animation
          shooting();
          // Return to idle after shooting
          gCurrentPatternNumber = 0; // Idle mode
          Serial.println("Shooting complete - Returning to Idle");
        }
      }
      notifyPatternChange();
    }
  }

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    // Call the appropriate pattern function based on current state
    gPatterns[gCurrentPatternNumber]();
    FastLED.show();
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

// This line must be at the end, after all functions are defined
