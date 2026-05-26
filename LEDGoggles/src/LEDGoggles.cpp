#include <ESP32Servo.h>
#include <NeoPixelBus.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLESecurity.h>
#include <BLE2902.h>
#include <esp_task_wdt.h>

#define PIXEL_COUNT 44  // Number of NeoPixels

// Define the LED pin


// Define the filter order
#define FILTER_ORDER 16
#define xADC 12
#define yADC 13
#define PIXEL_PIN    15  // Digital IO pin connected to the NeoPixels.

int servoPin = 4;




Servo myservo;  // create servo object to control a servo
// 16 servo objects can be created on the ESP32

// Declare our NeoPixel strip object:
// Demonstrating the use of the first four channels, but the method used allows for eight
//NeoPixelBus<NeoBgrFeature, NeoEsp32I2s1X8Ws2811Method> strip1(10, 15); // note: older WS2811 and longer strip
NeoPixelBus<NeoGrbFeature, NeoEsp32I2s1X8Ws2812xMethod> strip1(PIXEL_COUNT, PIXEL_PIN); // note: modern WS2812 with letter like WS2812b

uint8_t     mode     = 4;    // Currently-active animation mode, 0-5 (default: spinning wheels red)

// BLE configuration (unique per prop)
#define SERVICE_UUID "09d2abea-30ec-4519-86ff-ba0cbaf79160"
#define CHARACTERISTIC_UUID "102d8bf0-dc7b-44d2-8cfe-0e09f2ee6107"
#define PASSKEY 123456

BLEServer *pServer = nullptr;
BLECharacteristic *pCharacteristic = nullptr;
volatile bool deviceConnected = false;
volatile bool oldDeviceConnected = false;
volatile uint8_t bleCommand = 0;
volatile bool bleCommandPending = false;
uint8_t prevMode = 255;

// BLE write callback — receives mode commands from armDisplay
class GogglesWriteCallback : public BLECharacteristicCallbacks {
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
    } else {
      Serial.println("  - BLE auth failure");
      if (pServer) {
        uint16_t connId = pServer->getConnId();
        if (connId != 0) {
          pServer->removePeerDevice(connId, true);
        }
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

// Create an array to store the sensor readings
u_int16_t xReadings[FILTER_ORDER];
u_int16_t yReadings[FILTER_ORDER];
u_int8_t filterIndex[2]; //Increase for number of ADCs

u_int32_t prevMilis[2];
u_int32_t printMilis = 0;
u_int32_t lightMilis = 0;

u_int32_t servoMilis = 0;
u_int32_t posTimePrev = 0;
u_int32_t joystickMilis = 0;
// Initialize the filter output
u_int16_t xfilteredOutput = 0;
u_int16_t yfilteredOutput = 0;

u_int16_t xSum = 0;
u_int16_t ySum = 0;
u_int16_t prevLEDCount = 0;
u_int8_t ledState = 0;
u_int8_t ledBounce = 0;
u_int8_t offset = 0;
u_int8_t pos = 0;




hw_timer_t *Timer0_Cfg = nullptr; // Intentionally always-on: drives blinker + servo arm via ISR
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// Forward declarations
void colorWipe(u_int8_t red, u_int8_t green, u_int8_t blue, uint32_t wait, bool bounce);
void spinningWheelsLED(u_int8_t red, u_int8_t green, u_int8_t blue, uint32_t wait);
void clearLED();

void avg_flt (u_int8_t channel, u_int8_t *index, u_int16_t *sensorReadings, u_int16_t *filteredOutput, u_int16_t *sum )
{
  u_int16_t newReading = 0;
  // Get a new sensor reading
  
  newReading = analogRead(channel);
  
  // Add the new reading to the array
  *sum = *sum - sensorReadings[*index]; //remove oldest value from total
  sensorReadings[*index] = newReading;
  *sum += newReading;
  //Increment index and reset if buffer size is met
  (*index)++;
  if ((*index) > (FILTER_ORDER - 1))
    {
      *index = 0;
    }
  *filteredOutput = (*sum) >> 4; //divide by 16

}

void updateJoystick()
{
    avg_flt (xADC, &(filterIndex[0]), xReadings, &xfilteredOutput, &xSum);
    avg_flt (yADC, &(filterIndex[1]), yReadings, &yfilteredOutput, &ySum);
}

void blinker()
{
    u_int32_t newMilis = millis();
    if (newMilis - prevMilis[1] > 500)
    {
      prevMilis[1] = newMilis;
      ledState = !ledState;  
      digitalWrite(BUILTIN_LED, ledState);
    }
}

void printSensorInfo()
{
  Serial.print("XPOS:");
  Serial.println(pos);
}
void servoMove()
{   u_int32_t servoNowTime = millis();
  if ((servoNowTime - servoMilis) >= 15)
  {
    servoMilis = servoNowTime;
    portENTER_CRITICAL(&timerMux);
    uint8_t cp = pos;
    portEXIT_CRITICAL(&timerMux);
    myservo.write(cp);
  }
}

uint8_t prevLightingMode = 255;

void lighting()
{
  if (prevLightingMode != mode) {
    prevLEDCount = 0;
    ledBounce = 0;
    offset = 0;
    lightMilis = millis();
    prevLightingMode = mode;
  }

   switch(mode) {           // Start the new animation...
    case 0:
      clearLED();
      break;
    case 1:
      colorWipe(100,   0,   0, 50, true);    // Red
      break;
    case 2:
      colorWipe(0, 100,   0, 50, true);    // Green
      break;
    case 3:
      colorWipe( 0,   0, 100, 50, true);    // Blue
      break;
    case 4:
      spinningWheelsLED(150, 0, 0, 50); // 
      break;
    case 5:
      spinningWheelsLED(0, 100, 0, 50); // 
      break;
      }
}

void clearLED()
{
  for (int i = 0; i < strip1.PixelCount(); i++)
      {
        strip1.SetPixelColor(i, RgbColor (0, 0, 0));
      }
      strip1.Show();

}

// Fill strip pixels one after another with a color. Strip is NOT cleared
// first; anything there will be covered pixel by pixel. Pass in color
// (as a single 'packed' 32-bit value, which you can get by calling
// strip.Color(red, green, blue) as shown in the loop() function above),
// and a delay time (in milliseconds) between pixels.
void colorWipe(u_int8_t red, u_int8_t green, u_int8_t blue, uint32_t wait, bool bounce) {
  uint32_t newCall =  millis();
  if ((newCall - lightMilis) > wait) 
  {

      if (!bounce){
        if (strip1.PixelCount() > prevLEDCount)
        {
          strip1.SetPixelColor(prevLEDCount, RgbColor (red, green, blue));         //  Set pixel's color (in RAM)
          strip1.Show();                       //  Update strip to match
          prevLEDCount++;
          }
      }
      else if (bounce){
        if (!ledBounce && prevLEDCount < strip1.PixelCount()){
          strip1.SetPixelColor(prevLEDCount, RgbColor (red, green, blue));
          strip1.Show();
          prevLEDCount++;
        }
        else if (ledBounce && prevLEDCount > 0){
          strip1.SetPixelColor(prevLEDCount - 1, RgbColor (0, 0, 0));
          strip1.Show();
          prevLEDCount--;
        }
        else if (!ledBounce && prevLEDCount >= strip1.PixelCount()){
          ledBounce = true;
        }
        else if (ledBounce && prevLEDCount == 0){
          ledBounce = false;
        }
     }

      lightMilis = newCall;
  }
}


void spinningWheelsLED(u_int8_t red, u_int8_t green, u_int8_t blue, uint32_t wait)
{
  uint32_t newCall =  millis();
  if ((newCall - lightMilis) > wait) 
  {
    for (int z = 0; z < PIXEL_COUNT / 2; z++) {
      if (((offset + z) & 7) < 2) {
        strip1.SetPixelColor(z, RgbColor(red, green, blue));
        strip1.SetPixelColor(PIXEL_COUNT - 1 - z, RgbColor(red, green, blue));
      }
      else {
        strip1.SetPixelColor(z, RgbColor(0, 0, 0));
        strip1.SetPixelColor(PIXEL_COUNT - 1 - z, RgbColor(0, 0, 0));
      }
    }
    strip1.Show();
    offset++;
    if (offset > 7) {
      offset = 0;
    }
    lightMilis = newCall;
  }
}

void notifyModeChange()
{
  if (prevMode != mode && pCharacteristic) {
    Serial.print("Mode: ");
    Serial.println(mode);
    pCharacteristic->setValue(&mode, 1);
    pCharacteristic->notify();
    prevMode = mode;
  }
}

void bleSecuritySetup()
{
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

void bleSetup()
{
  BLEDevice::init("LEDGoggles");
  BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);
  BLEDevice::setSecurityCallbacks(new SecurityCallback());

  bleSecuritySetup();

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE
  );

  pCharacteristic->setAccessPermissions(ESP_GATT_PERM_READ_ENCRYPTED | ESP_GATT_PERM_WRITE_ENCRYPTED);
  pCharacteristic->setCallbacks(new GogglesWriteCallback());

  BLEDescriptor *pDescr = new BLEDescriptor((uint16_t)0x2901);
  pDescr->setValue("Goggles_Control");
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

  Serial.println("BLE advertising as LEDGoggles");
}



void armPos(u_int16_t * filteredADC, u_int32_t wait)
{
  u_int32_t posTimeNow = millis();
  if ((*filteredADC > 3500 || *filteredADC < 500 ) && (posTimeNow - posTimePrev >= wait))
  {
    posTimePrev = posTimeNow;
    if (*filteredADC > 3500)
    {
      pos++;
      if (pos > 180)
        pos = 180;
    }
    else {
        pos--;
        if (pos <= 0)
          pos = 0;
    }
        
  }
}
// Define a callback function that will be called when the timer expires
static uint32_t isrTick = 0;
void IRAM_ATTR Timer0_ISR()
{
   portENTER_CRITICAL_ISR(&timerMux);
   isrTick++;
   if ((isrTick & 1) == 1) {
     blinker();
   }
   if ((isrTick % 25) == 0) {
     armPos(&xfilteredOutput, 50);
   }
   portEXIT_CRITICAL_ISR(&timerMux);
}

void setup() {
  Serial.begin(115200);
  esp_log_level_set("*", ESP_LOG_WARN);

  bleSetup();

  // Set up the LED pin as an output
  pinMode(BUILTIN_LED, OUTPUT);

  Timer0_Cfg = timerBegin(0, 80, true);
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, false);
  timerAlarmWrite(Timer0_Cfg, 2000, true);
  timerAlarmEnable(Timer0_Cfg);

  strip1.Begin();
  strip1.Show();

  myservo.setPeriodHertz(50);
  myservo.attach(servoPin, 500, 3500);

  for (int i = 0; i < FILTER_ORDER; i++) {
    xReadings[i] = 0;
    yReadings[i] = 0;
  }
  esp_task_wdt_init(30, true);
  esp_task_wdt_add(nullptr);
}



void loop() {
  // BLE re-advertising after disconnect
  if (!deviceConnected && oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
    pServer->startAdvertising();
    Serial.println("BLE re-advertising");
  }
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }

  // Process BLE command
  if (bleCommandPending) {
    portENTER_CRITICAL(&timerMux);
    uint8_t cmd = bleCommand;
    bleCommandPending = false;
    portEXIT_CRITICAL(&timerMux);
    if (cmd <= 5) {
      mode = cmd;
      notifyModeChange();
    }
  }

  u_int32_t newMilisPrint = millis();
  if (newMilisPrint - printMilis > 100)
  {
    printMilis = newMilisPrint;
    printSensorInfo();
  }
  lighting();
  servoMove();

  u_int32_t now = millis();
  if (now - joystickMilis >= 16) {
    joystickMilis = now;
    updateJoystick();
  }
  
  esp_task_wdt_reset();
}







  

