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
#define SHOOT_INTERVAL_TIMING 500
CRGB leds[NUM_LEDS];
uint8_t breathBrightness = 100;

// Button configuration
#define BUTTON_PIN 6
bool lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 20;
unsigned long shooting_time = 0;

BLEServer *pServer;
BLECharacteristic *pCharacteristic;
BLEDescriptor *pDescr;
BLE2902 *pBLE2902;
bool buttonPressed = false;
bool deviceConnected = false;
bool oldDeviceConnected = false;

uint8_t gCurrentPatternNumber = 0;
uint8_t prev_gCurrentPatternNumber = 255; // Initialized to a different value
u_int8_t status = 0;
volatile bool reading_base = false;

typedef void (*SimplePatternList[])();

unsigned long previousMillis = 0;
const long interval = 8; // 8 miliseconds to show

// Declare variables to store the button state and press time
int buttonState = LOW;
volatile bool ledState = false;
u_int32_t prevMilis[8]; //prev mil for each status
unsigned long pressTime = 0;
unsigned long lastLoopTime = 0;

#define SHORT_PRESS_DURATION 2000
//#define LONG_PRESS_DURATION 1000
#define DEBOUNCE_TIMING 20

u_int16_t milSecCount = 0;
u_int32_t secCount = 0;
hw_timer_t *Timer0_Cfg = NULL;

// PIN for BLE pairing
#define PASSKEY 123456 // 6-digit PIN

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
    } else {
      Serial.println("   - SecurityCallback - Authentication Failure*");
      pServer->removePeerDevice(pServer->getConnId(), true);
    }
    BLEDevice::startAdvertising();
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

void notifyPatternChange() {
  if (prev_gCurrentPatternNumber != gCurrentPatternNumber) {

    intToPrint(gCurrentPatternNumber);
    pCharacteristic->setValue(&gCurrentPatternNumber, 1);
    pCharacteristic->notify();
    prev_gCurrentPatternNumber = gCurrentPatternNumber;
  }
}

void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  timer_interupt_setup();
  // Set logging level
  esp_log_level_set("*", ESP_LOG_DEBUG);

  BLE_setup();

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
  reading_base = !digitalRead(BUTTON_PIN);
}

SimplePatternList gPatterns = {idle, charging, overcharging, shooting};

void loop() {
  



  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    // Perform your action here
    notifyPatternChange();
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
  for (int i = NUM_LEDS - 1; i >= 0; i--) {
    leds[i] = CRGB::Blue;
    FastLED.show();
    delay(200);  // Adjust this delay to control the startup speed
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

void buttonReader()
{
    // Get the current time
  unsigned long currentTime = millis();

  // Calculate the elapsed time since the last loop iteration
  unsigned long elapsedTime = currentTime - lastLoopTime;
  unsigned long elapsedShootingTime = currentTime - shooting_time;
  int currentState = digitalRead(BUTTON_PIN);

  // Debounce the button
  if ((currentState != buttonState) && (elapsedTime >= DEBOUNCE_TIMING) && (elapsedShootingTime >= SHOOT_INTERVAL_TIMING ) ) {
    buttonState = currentState;
      // Update the last loop time
  lastLoopTime = currentTime;
  }


  // Detect a short press
  if (buttonState != reading_base && pressTime == 0) {
    pressTime = millis();
    gCurrentPatternNumber = 2; //charging
  }

  // Detect a long press
  if ((buttonState != reading_base) && ((millis() - pressTime) >= SHORT_PRESS_DURATION) && (gCurrentPatternNumber == 2)) {

    // Reset the press time
    pressTime = 0;
    gCurrentPatternNumber = 3; //overcharge
  }

  // Detect a short press release
  if ((buttonState == reading_base && pressTime > 0) && ((gCurrentPatternNumber == 2) || (gCurrentPatternNumber == 3))) {
    gCurrentPatternNumber = 4; //shooting
    // Reset the press time
    shooting_time = millis();
    
  }
}

void blinker()
{
    u_int32_t newMilis = millis();
    if (newMilis - prevMilis[1] > 500)
    {
      prevMilis[1] = newMilis;
      ledState = !ledState;  
      digitalWrite(LED_BUILTIN, ledState);
    }
}

// Define a callback function that will be called when the timer expires
void IRAM_ATTR Timer0_ISR() 
{
  milSecCount+= 2;
  status++;
  if (status >= 8)
  {
    status = 0;
  }
  if (milSecCount >= 1000)
  {
    secCount++;
    milSecCount = 0;
  }
  
   switch (status) //2ms per status
  {
    case 0:
      blinker();
      break;
    case 1:
      buttonReader();
      break;
    case 2:
      break;
    case 3:
       break;
    case 4:
      break;
    case 5:
      break;
    case 6:
      break;
    case 7: 
      break;
    default:
      break;
  }
    
}

void timer_interupt_setup(){
  Timer0_Cfg = timerBegin(1000000);
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR);
  timerAlarm(Timer0_Cfg, 2000, true, 0);
}

void BLE_setup(){
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
}