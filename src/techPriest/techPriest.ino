#include <ESP32Servo.h>
#include <NeoPixelBus.h>

// Digital IO pin connected to the button. This will be driven with a
// pull-up resistor so the switch pulls the pin to ground momentarily.
// On a high -> low transition the button press logic will execute.

#define PIXEL_COUNT 44  // Number of NeoPixels
#define SHORT_PRESS_DURATION 250
#define LONG_PRESS_DURATION 1000
#define DEBOUNCE_TIMING 20

// Declare variables to store the button state and press time
int buttonState = LOW;
unsigned long pressTime = 0;
unsigned long lastLoopTime = 0;

// Define the LED pin
#define LED_PIN 2

// Define the filter order
#define FILTER_ORDER 16
#define xADC 12
#define yADC 13
#define swADC 14
#define PIXEL_PIN    15  // Digital IO pin connected to the NeoPixels.

int servoPin = 4;




Servo myservo;  // create servo object to control a servo
// 16 servo objects can be created on the ESP32

// Declare our NeoPixel strip object:
// Demonstrating the use of the first four channels, but the method used allows for eight
//NeoPixelBus<NeoBgrFeature, NeoEsp32I2s1X8Ws2811Method> strip1(10, 15); // note: older WS2811 and longer strip
NeoPixelBus<NeoGrbFeature, NeoEsp32I2s1X8Ws2812xMethod> strip1(PIXEL_COUNT, PIXEL_PIN); // note: modern WS2812 with letter like WS2812b

boolean oldState = HIGH;
uint8_t     mode     = 0;    // Currently-active animation mode, 0-9

 

// Create an array to store the sensor readings
u_int16_t xReadings[FILTER_ORDER];
u_int16_t yReadings[FILTER_ORDER];
u_int8_t filterIndex[2]; //Increase for number of ADCs

u_int32_t secCount = 0;
u_int32_t prevMilis[8]; //prev mil for each status
u_int32_t printMilis = 0;
u_int32_t lightMilis = 0;
u_int32_t lightCasePrint = 0;
u_int32_t servoMilis = 0;
u_int32_t posTimePrev = 0;
// Initialize the filter output
u_int16_t xfilteredOutput = 0;
u_int16_t yfilteredOutput = 0;
u_int16_t swfilteredOutput = 0;
u_int16_t xSum = 0;
u_int16_t ySum = 0;
u_int16_t prevLEDCount = 0;
u_int16_t milSecCount = 0;

u_int8_t ledState = 0;
u_int8_t status = 0;
u_int8_t ledBounce = 0;
u_int8_t servoBounce = 0;
u_int8_t offset = 0;
u_int8_t pos = 10;

bool ledSwitch = false;
bool servoSwitch = false;


// Create a timer object
TimerHandle_t timer;
#define LED 21
 
hw_timer_t *Timer0_Cfg = NULL;

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
    //avg_flt (swADC, &(filterIndex[2]), swReadings, &swfilteredOutput, &swSum);
}

void blinker()
{
    u_int32_t newMilis = millis();
    if (newMilis - prevMilis[1] > 500)
    {
      prevMilis[1] = newMilis;
      ledState = !ledState;  
      digitalWrite(LED_PIN, ledState);
    }
}

void printSensorInfo()
{
  /*
  Serial.print("X:");
  Serial.println(xfilteredOutput);
  Serial.print("Y:");
  Serial.println(yfilteredOutput);
*/
  Serial.print("XPOS:");
  Serial.println(pos);
}
void servoSweep()
{
  
    if (pos < 110 && servoBounce == false)
      {
        pos++;
        if (pos == 110)
          servoBounce = true;
      }
    else if (pos > 0 && servoBounce == true)
    {
        pos--;
        if (pos == 20)
          servoBounce = false;
      }
}
void servoMove()
{   u_int32_t servoNowTime = millis();
  if ((servoNowTime - servoMilis) >= 15)
  {
    servoMilis = servoNowTime;
    //servoSweep();

		myservo.write(pos);    // tell servo to go to position in variable 'pos'
		
  }
}

void lighting()
{

  // Check if state changed from high to low (button press).
  if(ledSwitch) {
      if(++mode > 5) mode = 0; 
      clearLED();
      ledSwitch = false;
      prevLEDCount = 0;
      }       
      mode = 4;              // Advance to next mode, wrap around after #8
  switch(mode) {           // Start the new animation...
    case 0:
      colorWipe( 0,   0,   0, 50, true);    // Black/off
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

      if (bounce == false){
        if (strip1.PixelCount() > prevLEDCount)
        {
          strip1.SetPixelColor(prevLEDCount, RgbColor (red, green, blue));         //  Set pixel's color (in RAM)
          strip1.Show();                       //  Update strip to match
          prevLEDCount++;
          }
      }
      else if (bounce == true){
        if ((strip1.PixelCount() >= prevLEDCount) && (ledBounce == false)){
          strip1.SetPixelColor(prevLEDCount, RgbColor (red, green, blue));         //  Set pixel's color (in RAM)
          strip1.Show();                       //  Update strip to match
          prevLEDCount++;
        }
        else if ((prevLEDCount > 0) && (ledBounce == true)){
          strip1.SetPixelColor(prevLEDCount, RgbColor (0, 0, 0));         //  Set pixel's color (in RAM)
          strip1.Show();                       //  Update strip to match
          prevLEDCount--;
        }
        else{
          if (ledBounce == 1)
            strip1.SetPixelColor(prevLEDCount, RgbColor (0, 0, 0)); 
            strip1.Show(); 
          ledBounce = !ledBounce;
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
    for (int z = 0; z < 44; z++) {
      uint32_t c = 0;
      if (((offset + z) & 7) < 2) {// 4 pixels on...
        strip1.SetPixelColor(   z, RgbColor (red, green, blue)); // First eye
        strip1.SetPixelColor(23 - z, RgbColor (red, green, blue)); // Second eye (flipped)
      }
      else{
        strip1.SetPixelColor(   z, RgbColor (0, 0, 0));
        strip1.SetPixelColor(23 - z, RgbColor (0, 0, 0));
      }
    }
    strip1.Show();
    offset++;
    if (offset > 44)
    {
      offset = 0;
    }
    lightMilis = newCall;
  }
}

void buttonReader()
{
    // Get the current time
  unsigned long currentTime = millis();

  // Calculate the elapsed time since the last loop iteration
  unsigned long elapsedTime = currentTime - lastLoopTime;



  int currentState = digitalRead(swADC);

  // Debounce the button
  if (currentState != buttonState && elapsedTime >= DEBOUNCE_TIMING) {
    buttonState = currentState;
      // Update the last loop time
  lastLoopTime = currentTime;
  }


  // Detect a short press
  if (buttonState == LOW && pressTime == 0) {
    pressTime = millis();
  }

  // Detect a long press
  if ((buttonState == LOW) && ((millis() - pressTime) >= LONG_PRESS_DURATION)) {

    // Reset the press time
    pressTime = 0;
    ledSwitch = false;
    servoSwitch = true;
  }

  // Detect a short press release
  if ((buttonState == HIGH && pressTime > 0) && ((millis() - pressTime) < SHORT_PRESS_DURATION)) {

    // Reset the press time
    pressTime = 0;
    ledSwitch = true;
    servoSwitch = false;
  }
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
        if (pos <= 10)
          pos = 10;
    }
        
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
      updateJoystick();
      break;
    case 1:
      blinker();
      break;
    case 2:
      buttonReader();
      break;
    case 3:
      armPos(&xfilteredOutput, 50);
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

void setup() {
  Serial.begin(115200);
     // Set up the LED pin as an output
  pinMode(LED_PIN, OUTPUT);

  Timer0_Cfg = timerBegin(1000000);
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR);
  timerAlarm(Timer0_Cfg, 2000, true, 0); //2ms

  strip1.Begin();
  strip1.Show();  // Initialize all pixels to 'off'

  // Allow allocation of all timers
	//ESP32PWM::allocateTimer(0);
	myservo.setPeriodHertz(50);    // standard 50 hz servo
	myservo.attach(servoPin, 500, 3500); // attaches the servo on pin 18 to the servo object


  // Initialize the sensor readings array
  for (int i = 0; i < FILTER_ORDER; i++) {
    xReadings[i] = 0;
    yReadings[i] = 0;
    
  }
  for (int j = 0; j < 8; j++){
    prevMilis[j] = 0;
  }
  for (int k = 0; k < 2; k++){
    filterIndex[k] = 0;
  }
}



void loop() {

 
  u_int32_t newMilisPrint = millis();
  if (newMilisPrint - printMilis > 100)
  {
    printMilis = newMilisPrint;
    printSensorInfo();
  }
  lighting();
  servoMove();
  
}







  

