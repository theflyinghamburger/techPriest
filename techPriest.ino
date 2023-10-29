#include <analogWrite.h>
#include <ESP32PWM.h>
#include <ESP32Servo.h>
#include <Arduino.h>
#include <NeoPixelBus.h>

// Digital IO pin connected to the button. This will be driven with a
// pull-up resistor so the switch pulls the pin to ground momentarily.
// On a high -> low transition the button press logic will execute.
#define PIXEL_PIN    6  // Digital IO pin connected to the NeoPixels.
#define PIXEL_COUNT 16  // Number of NeoPixels

// Define the LED pin
#define LED_PIN 2

// Define the filter order
#define FILTER_ORDER 16
#define xADC 12
#define yADC 13
#define swADC 14

// Declare our NeoPixel strip object:
// Demonstrating the use of the first four channels, but the method used allows for eight
//NeoPixelBus<NeoBgrFeature, NeoEsp32I2s1X8Ws2811Method> strip1(10, 15); // note: older WS2811 and longer strip
NeoPixelBus<NeoGrbFeature, NeoEsp32I2s1X8Ws2812xMethod> strip1(100, 15); // note: modern WS2812 with letter like WS2812b

boolean oldState = HIGH;
int     mode     = 0;    // Currently-active animation mode, 0-9
 
Servo myservo;  // create servo object to control a servo
// 16 servo objects can be created on the ESP32
 

// Create an array to store the sensor readings
u_int16_t xReadings[FILTER_ORDER];
u_int16_t yReadings[FILTER_ORDER];
u_int16_t swReadings[FILTER_ORDER];
u_int8_t filterIndex[3]; //Increase for number of ADCs

u_int32_t secCount = 0;
u_int32_t prevMilis[8]; //prev mil for each status
u_int32_t printMilis = 0;
u_int32_t lightMilis = 0;
// Initialize the filter output
u_int16_t xfilteredOutput = 0;
u_int16_t yfilteredOutput = 0;
u_int16_t swfilteredOutput = 0;
u_int16_t xSum = 0;
u_int16_t ySum = 0;
u_int16_t swSum = 0;

u_int16_t milSecCount = 0;

u_int8_t ledState = 0;
u_int8_t status = 0;

// Create a timer object
TimerHandle_t timer;
#define LED 21
 
hw_timer_t *Timer0_Cfg = NULL;



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
      servoMove();
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
    avg_flt (swADC, &(filterIndex[2]), swReadings, &swfilteredOutput, &swSum);
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
  Serial.print("X = ");
  Serial.print(xfilteredOutput);
  Serial.print("\t Y = ");
  Serial.print(yfilteredOutput);
  Serial.print("\t SW= ");
  Serial.println(swfilteredOutput);
}

void servoMove()
{

}

void lighting()
{
  /*

  // Get current button state.
  u_int16_t newState = swfilteredOutput;

  // Check if state changed from high to low (button press).
  if((newState < 10) && (oldState > 350)) {

      if(++mode > 8) mode = 0; // Advance to next mode, wrap around after #8
      switch(mode) {           // Start the new animation...
        case 0:
          colorWipe(strip.Color(  0,   0,   0), 50);    // Black/off
          break;
        case 1:
          colorWipe(strip.Color(255,   0,   0), 50);    // Red
          break;
        case 2:
          colorWipe(strip.Color(  0, 255,   0), 50);    // Green
          break;
        case 3:
          colorWipe(strip.Color(  0,   0, 255), 50);    // Blue
          break;
        case 4:
          theaterChase(strip.Color(127, 127, 127), 50); // White
          break;
        case 5:
          theaterChase(strip.Color(127,   0,   0), 50); // Red
          break;
        case 6:
          theaterChase(strip.Color(  0,   0, 127), 50); // Blue
          break;
        case 7:
          rainbow(10);
          break;
        case 8:
          theaterChaseRainbow(50);
          break;
      }
    
  }

  // Set the last-read button state to the old state.
  oldState = newState;
  */
        // draw on the strips
    
    for (int i = 0; i < 16; i++){
      for (int j = 0; j < 100; j++ ){
      strip1.SetPixelColor(i, RgbColor(j, 0, j));      // red
      strip1.Show();
      }
    }

    for (int i = 16; i >= 0; i--){
      for (int j = 100; j >= 0; j-- ){
      strip1.SetPixelColor(i, RgbColor(j, 0, j));      // red
      strip1.Show();
      }
    }
    //strip2.Show();
}

/*
// Fill strip pixels one after another with a color. Strip is NOT cleared
// first; anything there will be covered pixel by pixel. Pass in color
// (as a single 'packed' 32-bit value, which you can get by calling
// strip.Color(red, green, blue) as shown in the loop() function above),
// and a delay time (in milliseconds) between pixels.
void colorWipe(uint32_t color, int wait) {
  for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
    strip.show();                          //  Update strip to match
    //delay(wait);                           //  Pause for a moment
  }
}

// Theater-marquee-style chasing lights. Pass in a color (32-bit value,
// a la strip.Color(r,g,b) as mentioned above), and a delay time (in ms)
// between frames.
void theaterChase(uint32_t color, int wait) {
  for(int a=0; a<10; a++) {  // Repeat 10 times...
    for(int b=0; b<3; b++) { //  'b' counts from 0 to 2...
      strip.clear();         //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in steps of 3...
      for(int c=b; c<strip.numPixels(); c += 3) {
        strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
      }
      strip.show(); // Update strip with new contents
      //delay(wait);  // Pause for a moment
    }
  }
}

// Rainbow cycle along whole strip. Pass delay time (in ms) between frames.
void rainbow(int wait) {
  // Hue of first pixel runs 3 complete loops through the color wheel.
  // Color wheel has a range of 65536 but it's OK if we roll over, so
  // just count from 0 to 3*65536. Adding 256 to firstPixelHue each time
  // means we'll make 3*65536/256 = 768 passes through this outer loop:
  for(long firstPixelHue = 0; firstPixelHue < 3*65536; firstPixelHue += 256) {
    for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
      // Offset pixel hue by an amount to make one full revolution of the
      // color wheel (range of 65536) along the length of the strip
      // (strip.numPixels() steps):
      int pixelHue = firstPixelHue + (i * 65536L / strip.numPixels());
      // strip.ColorHSV() can take 1 or 3 arguments: a hue (0 to 65535) or
      // optionally add saturation and value (brightness) (each 0 to 255).
      // Here we're using just the single-argument hue variant. The result
      // is passed through strip.gamma32() to provide 'truer' colors
      // before assigning to each pixel:
      strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(pixelHue)));
    }
    strip.show(); // Update strip with new contents
    //delay(wait);  // Pause for a moment
  }
}

// Rainbow-enhanced theater marquee. Pass delay time (in ms) between frames.
void theaterChaseRainbow(int wait) {
  int firstPixelHue = 0;     // First pixel starts at red (hue 0)
  for(int a=0; a<30; a++) {  // Repeat 30 times...
    for(int b=0; b<3; b++) { //  'b' counts from 0 to 2...
      strip.clear();         //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in increments of 3...
      for(int c=b; c<strip.numPixels(); c += 3) {
        // hue of pixel 'c' is offset by an amount to make one full
        // revolution of the color wheel (range 65536) along the length
        // of the strip (strip.numPixels() steps):
        int      hue   = firstPixelHue + c * 65536L / strip.numPixels();
        uint32_t color = strip.gamma32(strip.ColorHSV(hue)); // hue -> RGB
        strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
      }
      strip.show();                // Update strip with new contents
      //delay(wait);                 // Pause for a moment
      firstPixelHue += 65536 / 90; // One cycle of color wheel over 90 frames
    }
  }
}
*/

void setup() {
  Serial.begin(115200);
     // Set up the LED pin as an output
  pinMode(LED_PIN, OUTPUT);

  Timer0_Cfg = timerBegin(0, 80, true);
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
  timerAlarmWrite(Timer0_Cfg, 2000, true); //2ms
  timerAlarmEnable(Timer0_Cfg);

  strip1.Begin();
  strip1.Show();  // Initialize all pixels to 'off'

  // Initialize the sensor readings array
  for (int i = 0; i < FILTER_ORDER; i++) {
    xReadings[i] = 0;
    yReadings[i] = 0;
    swReadings[i] = 0;
  }
  for (int j = 0; j < 8; j++){
    prevMilis[j] = 0;
  }
  for (int k = 0; k < 3; k++){
    filterIndex[k] = 0;
  }
}



void loop() {

 
  u_int32_t newMilisPrint = millis();
  if (newMilisPrint - printMilis > 500)
  {
    printMilis = newMilisPrint;
    printSensorInfo();
  }
  newMilisPrint = millis();
  if (newMilisPrint - lightMilis > 50)
  { 
    lightMilis = newMilisPrint;
    lighting();
  }
  
}







  

