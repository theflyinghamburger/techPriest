// Shooting to idle condition is not being triggered
// Overcharge is not being triggered

#include <FastLED.h>
FASTLED_USING_NAMESPACE

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

// Button and blinker configuration
#define BUTTON_PIN 6


bool lastButtonState = HIGH;
unsigned long buttonPressStartTime = 0;  // Track when the button is first pressed
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 20;
unsigned long shooting_time = 0;
unsigned long blink_time = 0;
unsigned long shootingStartTime = 0;         // Track when shooting mode starts
const unsigned long shootingDuration = 500;  // Duration for shooting mode in milliseconds
unsigned long pressDuration = 0;

BLEServer *pServer;
BLECharacteristic *pCharacteristic;
BLEDescriptor *pDescr;
BLE2902 *pBLE2902;
bool buttonPressed = false;
bool deviceConnected = false;
bool oldDeviceConnected = false;
bool button_base = false;

uint8_t breathBrightness = 100;
uint8_t gCurrentPatternNumber = 0;         //Startup
uint8_t prev_gCurrentPatternNumber = 255;  // Initialized to a different value
uint8_t status = 0;
volatile bool patternChangeFlag = false;  // Flag to indicate a pattern change
volatile bool blinkFlag = false;          // Flag to handle blinking LED

// Timer variables
hw_timer_t *Timer0_Cfg = NULL;
volatile uint16_t milSecCount = 0;
volatile uint32_t secCount = 0;
volatile uint32_t prevSecCount = 0;
const unsigned long interval = 8;  // 8 miliseconds to show
unsigned long previousMillis = 0;

typedef void (*SimplePatternList[])();

// PIN for BLE pairing
#define PASSKEY 123456  // 6-digit PIN

class SecurityCallback : public BLESecurityCallbacks {
  uint32_t onPassKeyRequest() override {
    return PASSKEY;
  }

  void onPassKeyNotify(uint32_t pass_key) override {}

  bool onConfirmPIN(uint32_t pass_key) override {
    return true;
  }

  bool onSecurityRequest() override {
    return true;
  }

  void onAuthenticationComplete(esp_ble_auth_cmpl_t cmpl) override {
    if (cmpl.success) {
      Serial.println("Authentication Success");
    } else {
      Serial.println("Authentication Failure");
      pServer->removePeerDevice(pServer->getConnId(), true);
    }
    BLEDevice::startAdvertising();
  }
};

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) override {
    deviceConnected = true;
    Serial.println("Device connected");
  }

  void onDisconnect(BLEServer *pServer) override {
    deviceConnected = false;
    Serial.println("Device disconnected");
  }
};

void notifyPatternChange() {
  if (gCurrentPatternNumber != prev_gCurrentPatternNumber) {
    intToPrint(gCurrentPatternNumber);
    pCharacteristic->setValue(&gCurrentPatternNumber, 1);
    pCharacteristic->notify();
    prev_gCurrentPatternNumber = gCurrentPatternNumber;
  }
}

void intToPrint(uint8_t i) {
  Serial.print("Pattern: ");
  switch (i) {
    case 0:
      Serial.println("startup");
      break;
    case 1:
      Serial.println("idle");
      break;
    case 2:
      Serial.println("charging");
      break;
    case 3:
      Serial.println("overcharging");
      break;
    case 4:
      Serial.println("shooting");
      break;
  }
}

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
  for (int i = NUM_LEDS - 1; i >= 0; i--) {
    leds[i] = CRGB::Black;
    FastLED.show();
    delay(20);
  }
}

void addGlitter(fract8 chanceOfGlitter) {
  if (random8() < chanceOfGlitter) {
    leds[random16(NUM_LEDS)] += CRGB::White;
  }
}

void startup() {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB::Blue;
    FastLED.show();
    delay(200);  // Adjust this delay to control the startup speed
  }
  gCurrentPatternNumber = 1;  //switch to idle pattern
  patternChangeFlag = true;
}

void buttonReader() {
  bool currentButtonState = digitalRead(BUTTON_PIN) != button_base;

  // Check for debounce: Only change state if the button has been stable for debounceDelay ms
  if (currentButtonState != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // Update button pressed state only if the stable state is different
    if (currentButtonState != buttonPressed) {
      buttonPressed = currentButtonState;
    }
      if (buttonPressed) {
         buttonPressStartTime = millis();  // Record the time when the button was pressed
      }
    lastButtonState = currentButtonState;
  }
  if (buttonPressed) {

    if (gCurrentPatternNumber != 3) {
      gCurrentPatternNumber = 2;  // Charging mode
      patternChangeFlag = true;
    }
    pressDuration = millis() - buttonPressStartTime;
    if (pressDuration >= 2000) {
      gCurrentPatternNumber = 3;  // Overcharging mode
      patternChangeFlag = true;
    }
  } else {
    if ((gCurrentPatternNumber == 2) || (gCurrentPatternNumber == 3)) {
      gCurrentPatternNumber = 4;     // Shooting mode
      shootingStartTime = millis();  // Record the start time of shooting mode
      patternChangeFlag = true;
    }
  }
}

void buttonInfo(){
  Serial.print("Button pressed: ");
  Serial.println(buttonPressed);
  Serial.print("Press duration: ");
  Serial.println(pressDuration);
  intToPrint(gCurrentPatternNumber);
}

void blinker() {
  u_int32_t newMilis = millis();
  if (newMilis - blink_time > 500) {
    blink_time = newMilis;
    blinkFlag = !blinkFlag;
    digitalWrite(BUILTIN_LED, blinkFlag ? HIGH : LOW);  // Toggle the built-in LED
  }
}

void IRAM_ATTR Timer0_ISR() {
  milSecCount += 2;
  status++;
  if (status >= 8) {
    status = 0;
  }
  if (milSecCount >= 1000) {
    secCount++;
    milSecCount = 0;
  }

  switch (status) {
    case 0:
      blinker();
      break;
    case 1:
      buttonReader();
      break;
    default:
      break;
  }
}

void setupBLE() {
  BLEDevice::init("Plasma_Pistol");
  BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);
  BLEDevice::setSecurityCallbacks(new SecurityCallback());

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_NOTIFY);

  pCharacteristic->setAccessPermissions(ESP_GATT_PERM_READ_ENCRYPTED | ESP_GATT_PERM_WRITE_ENCRYPTED);
  pDescr = new BLEDescriptor((uint16_t)0x2901);
  pDescr->setValue("Plasma_Gun_State");
  pCharacteristic->addDescriptor(pDescr);

  pBLE2902 = new BLE2902();
  pBLE2902->setNotifications(true);
  pCharacteristic->addDescriptor(pBLE2902);

  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);
  BLEDevice::startAdvertising();

  Serial.println("Waiting for a client connection...");
}

void setupTimer() {
  Timer0_Cfg = timerBegin(1000000);
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR);
  timerAlarm(Timer0_Cfg, 2000, true, 0);  // 2 ms interval
}



void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUILTIN_LED, OUTPUT);  // Set the built-in LED pin as an output
  button_base = digitalRead(BUTTON_PIN);
  setupBLE();
  setupTimer();

  FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);
  fill_solid(leds, NUM_LEDS, CRGB::Black);  // Turn off LEDs after startup sequence
  notifyPatternChange();
  startup();  // Call the startup function
}

SimplePatternList gPatterns = { startup, idle, charging, overcharging, shooting };

void loop() {
  // Check for pattern changes outside of the ISR
  if (patternChangeFlag) {
    notifyPatternChange();
    patternChangeFlag = false;  // Reset the flag
  }

  // Handle timing for shooting mode
  if (gCurrentPatternNumber == 4 && (millis() - shootingStartTime) >= shootingDuration) {
    gCurrentPatternNumber = 1;  // Switch back to Idle mode
    patternChangeFlag = true;
  }
  // Handle FastLED updates in the loop
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    // Perform your action here
    gPatterns[gCurrentPatternNumber]();
    FastLED.show();
  }



  // Re-advertise if disconnected
  if (secCount > prevSecCount) {
    buttonInfo();
    if (!deviceConnected && oldDeviceConnected) {
      pServer->startAdvertising();
      oldDeviceConnected = deviceConnected;
    }
    prevSecCount = secCount;
  }
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }
}
