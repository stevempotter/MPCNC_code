
// Steve M. Potter 10 Dec 2021   steve@stevempotter.tech
// To monitor the Portable MPCNC I built. This uses the TTGO LilyGO V1.1 ESP32 board
//  which has a small TFT full-colour screen 135x240 pixels. I bought them from Tinytronics for about 8 euros each.
// I glued DS18B20 digital temperature sensors to the TB6600 board stepper drivers, and to the VFD heatsink. I had to remove the VFD
//  sensor because of all the interference the VFD makes. I had to use all shielded cables for these sensors. There
// is also an ambient air temp sensor for the inside of the electronics case, just in front of the outflow fan. 
// Still left to do:  
//  -Use interrupts on the ESP32 pushbuttons to allow adjusting the alarm temperature (or other things.)
//     Left button is GPIO 0 and right is GPIO 35
//  -Set up push notification alarms with Pushbullet
// 

#include <OneWire.h>
#include <DallasTemperature.h>
#include <Tone32.h>
#include <Preferences.h> // place on the ESP32 to store non-volatile variables.
#include "WiFi.h"
#include <SPI.h>
#include <TFT_eSPI.h>       // Hardware-specific library
TFT_eSPI tft = TFT_eSPI(); // TTGO LilyGO V1.1 screen is 240w X 135h

// Fonts I may use. 
#include "NotoSansBold15.h"
#include "NotoSansBold36.h"
#define AA_FONT_SMALL NotoSansBold15
#define AA_FONT_LARGE NotoSansBold36
#define LOAD_FONT6

#define ONE_WIRE_BUS 2 // This wire is pulled up to 3.3V with a 4k7 resistor.
#define SPEAKERPIN 32  // TTGO LilyGO board pins that don't work as digital IO: 36,37,38,39,21,22
// 21, 22 are I2C. The others are for small signal devices like thermocouple or onboard Hall sensor.
// UNLESS YOU REMOVE TWO TINY CAPS ON THE UNDERSIDE OF THE BOARD.
#define SPEAKERCHANNEL 1 // PWM timer channel for sound.
#define TEMPERATURE_PRECISION 10
#define MAXTEMP 50 // If any sensor gets hotter than this, Alarm is sounded.
#define FAN_PWM_PIN 27 // to control speed of both box fans
#define FAN_PWM_CHANNEL 3
#define FANSPEED_LOW 120 // Range: 51-255
#define LEFTFANTACHPIN 13  // fan tachometers give 50% duty cycle, 4 half-periods per rev (one rev is 4x the output of pulseIn() )
#define RIGHTFANTACHPIN 15 // They are pulled up to 3.3V with 1.8k and filtered with low-pass filter of
#define BOTTOMFANTACHPIN 17 // 220nf and 680ohm (1kHz)
const float PULSEIN_CORRECTION_FACTOR = 1.0;  // Used for calibrating fan RPM with PulseIn. I used an optical tachmometer to calibrate the fans. They were right on.

OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire device (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass our oneWire reference to Dallas Temperature.
int Sensor_count;
unsigned long FanTachPulseLength;
unsigned long MaxFanTachPulseuS = 100000; // 100ms 
float HotTemp = -1.0; 
float CoolTemp = -1.0;
float TempDiff = 0.0;
bool AlarmCondition = false;
bool OverTemperature = false; 
bool LftFanStalled = false;
bool RtFanStalled = false;
bool BotFanStalled = false;
int ToneLength_ms = 40;
int Hertz = 262; // start at NOTE_C4
int ToneSpacing = 200; // in Hz
int FanRPM;

// arrays to hold DS18B20 device addresses

 /*  These sensors are the ones glued to the DRV8825s on the RAMPS board                              
DeviceAddress X1_temp_addr =   {0x28, 0xE6, 0xAB, 0x07, 0xD6, 0x01, 0x3C, 0x51};   //
DeviceAddress X2_temp_addr =   {0x28, 0x01, 0xF6, 0x07, 0xD6, 0x01, 0x3C, 0x84};   //
DeviceAddress Y1_temp_addr =   {0x28, 0x9E, 0xA6, 0x07, 0xD6, 0x01, 0x3C, 0x84};   // 
DeviceAddress Y2_temp_addr =   {0x28, 0x44, 0x5D, 0x07, 0xD6, 0x01, 0x3C, 0x59};   // 
DeviceAddress Z_temp_addr  =   {0x28, 0xE8, 0xDA, 0x07, 0xD6, 0x01, 0x3C, 0xE2};   // 
*/

// You will need to find the addresses of the temp sensors you have. 
// There is an example sketch in the DallasTemperature library examples for this purpose called oneWireSearch.
DeviceAddress VFDtempAddr =    {0x28, 0x15, 0x00, 0x07, 0xD6, 0x01, 0x3C, 0x39};   // No longer used due to too much electrical interference from VFD
DeviceAddress BoxAirTempAddr = {0x28, 0x89, 0xF0, 0x07, 0xD6, 0x01, 0x3C, 0x53};   //
// Following are the sensors glued to the TB6600 stepper drivers, Identity of each determined with a soldering iron on the heatsink.
DeviceAddress Y2_temp_addr =   {0x28, 0x46, 0xD0, 0x07, 0xD6, 0x01, 0x3C, 0x10};   //
DeviceAddress Z_temp_addr =   {0x28, 0x8E, 0x69, 0x07, 0xD6, 0x01, 0x3C, 0x92};   //
DeviceAddress X2_temp_addr =   {0x28, 0x45, 0x8B, 0x07, 0xD6, 0x01, 0x3C, 0xAC};   // 
DeviceAddress Y1_temp_addr =   {0x28, 0x35, 0xAF, 0x07, 0xD6, 0x01, 0x3C, 0xD3};   // 
DeviceAddress X1_temp_addr  =   {0x28, 0x5D, 0x4E, 0x07, 0xD6, 0x01, 0x3C, 0xA6};   // 

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  Serial.print("{0x");
  for (uint8_t i = 0; i < 8; i++)
  {
    if (i != 0) Serial.print(", 0x");
    if (deviceAddress[i] < 16) Serial.print("0");   // zero pad the address if necessary
    Serial.print(deviceAddress[i], HEX);
    if (i == 7) Serial.print("}");
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(void)
{
  // start serial port
  Serial.begin(115200);
  ledcDetachPin(FAN_PWM_PIN); // I got weird problems of PWM not working sometimes without this.
  ledcSetup(FAN_PWM_CHANNEL, 25000, 8);  // Set up fan speed control PWM. PWM channel, freq, resolution (range: 51-255 or 20-100%)
  ledcAttachPin(FAN_PWM_PIN, FAN_PWM_CHANNEL);  // Use PWM Channel 0 on GPIO pin FAN_PWM_PIN
  ledcWrite(FAN_PWM_CHANNEL, 250); // Turn on the box fans almost full blast to overcome stiction and test out.
  pinMode(LEFTFANTACHPIN, INPUT_PULLUP);
  pinMode(RIGHTFANTACHPIN, INPUT_PULLUP);
  pinMode(BOTTOMFANTACHPIN, INPUT_PULLUP);
 
  AlarmSound(); // make the alarm noise to test the speaker and train people what it sounds like.
    
    // Initialise the TFT display registers
  tft.init(); // equivalent to tft.begin();
  tft.setRotation(0); // 0 is Portrait mode. TTGO LilyGO V1.1 is 240x135. Origin is the upper left.
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_MAGENTA, TFT_BLACK); // Set the font colour AND the background colour
  tft.setTextFont(6);
  tft.setCursor(0,0,4); // sets cursor location and font number to use for next tft.println()
  tft.println(" MPCNC");
  tft.println(" Temps");
  tft.setTextColor(TFT_YELLOW, TFT_BLACK); // Set the font colour AND the background colour    
  Serial.println("MPCNC Monitor with DS18B20 sensors and fan control. ");
  sensors.begin(); // Start up the temp sensors library
  delay(500); // can be removed if using Wifi, since that causes a delay that the sensors need to initialize.

/* 
    // Connect to Wi-Fi. I will eventually use this for pushing alerts to my phone.
    WiFi.mode(WIFI_STA);
    WiFi.begin("SSID", "PWD");
    Serial.print("Connecting to WiFi");
    long StartMillis = millis();
    bool NoWifi = false;
    while ((WiFi.status() != WL_CONNECTED) && (NoWifi == false)) {
      Serial.print('.');
      delay(1000);
      long ElapsedTime = millis() - StartMillis;
      if (ElapsedTime > 10000) {
        NoWifi = true;
        Serial.println(" Could not connect to Wifi in 10 seconds.");
       }
    }
    if (WiFi.status() == WL_CONNECTED){
    Serial.print("Connected! ");
    Serial.println(WiFi.localIP());  
   }
   */
  
  // locate onewire devices on the bus
  Serial.print("Locating temp sensors...");
  Serial.print("Found ");
  Sensor_count = sensors.getDeviceCount();
  Serial.print(Sensor_count, DEC);
  Serial.println(" DS18B20 sensors.");

  // show the addresses we found on the bus or programmed in explicitly
  Serial.print("VFDtempAddr: ");
  printAddress(VFDtempAddr);
  Serial.println();
  Serial.print("BoxAirTempAddr: ");
  printAddress(BoxAirTempAddr);
  Serial.println();

  Serial.print("X1_temp_addr: ");
  printAddress(X1_temp_addr);
  Serial.println();
  Serial.print("X2_temp_addr: ");
  printAddress(X2_temp_addr);
  Serial.println();
  Serial.print("Y1_temp_addr: ");
  printAddress(Y1_temp_addr);
  Serial.println();
  Serial.print("Y2_temp_addr: ");
  printAddress(Y2_temp_addr);
  Serial.println();
  Serial.print("Z_temp_addr: ");
  printAddress(Z_temp_addr);
  Serial.println();

  // Set the resolution to 9 - 12 bits. You really only need to do this once. It gets saved in non-volatile memory of the sensor.
  // 9 bits precision is about half a degree Celsius. 10 bits is 0.25 degree C precision. The accuracy across devices is only about that good.
  // Within one device, accuracy is very good, so use more bits if tiny changes in temperature are important.
  // Use fewer bits for faster/more frequent readings. Resolution can be 9,10,11, or 12 bits.
  // 12 bits: 750ms nom. conversion time, 0.06 C precision.
  // It seems that the conversions all happen in parallel, so adding more devices does not add much conversion time.

  sensors.setResolution(VFDtempAddr, TEMPERATURE_PRECISION);
  sensors.setResolution(BoxAirTempAddr, TEMPERATURE_PRECISION);
  sensors.setResolution(X1_temp_addr, TEMPERATURE_PRECISION);
  sensors.setResolution(X2_temp_addr, TEMPERATURE_PRECISION);
  sensors.setResolution(Y1_temp_addr, TEMPERATURE_PRECISION);
  sensors.setResolution(Y2_temp_addr, TEMPERATURE_PRECISION);
  sensors.setResolution(Z_temp_addr, TEMPERATURE_PRECISION);
  
  delay(500);

  ledcWrite(FAN_PWM_CHANNEL, FANSPEED_LOW); // Turn down fan speed
} //end setup

/////////////////////////////////
float printTemperature(DeviceAddress deviceAddress) // Function to print the temperature for a DS18B20 device
{
  float tempC = sensors.getTempC(deviceAddress);
  //printAddress(deviceAddress);
  //Serial.print(" Temp C: ");
  //Serial.println(tempC);
  if ((tempC > 126) || (tempC < 0)) {tempC = 0.0 ;} // Bad sensor or wiring, not an overtemperature.
  return tempC;
}

////////////////////////////////////////
void printResolution(DeviceAddress deviceAddress) // Function to print a DS18B20 device's resolution
{
  Serial.print("Resolution: ");
  Serial.print(sensors.getResolution(deviceAddress));
  Serial.println();
}

///////////////////////////////////////////
void printData(DeviceAddress deviceAddress) // Function to print information about a DS18B20 device
{
  Serial.print("DS18B20 Address: ");
  printAddress(deviceAddress);
  Serial.print(" ");
  printTemperature(deviceAddress);
  Serial.println();
}


/////////////////////////////////////////////////////////////////////
void AlarmSound()  // make a noise that is hard to ignore. TO DO: need an amplifier circuit to be heard when cutting wood.
{
 for (int i = 1; i <= 20; i++) 
  {  // Make a ramping up sound.
      tone(SPEAKERPIN, Hertz, ToneLength_ms, SPEAKERCHANNEL); // line, freq (Hz), duration (ms), PWM channel to use
      noTone(SPEAKERPIN, SPEAKERCHANNEL); // frees up the PWM channel for next use, and sets pin low, I think.    
      Serial.print(Hertz);
      Serial.println(" ALERT!");
      Hertz = Hertz + ToneSpacing;
  }
  for (int i = 1; i <= 20; i++) 
  {  // Make a ramping down sound.
      tone(SPEAKERPIN, Hertz, ToneLength_ms, SPEAKERCHANNEL); // line, freq (Hz), duration (ms), PWM channel to use
      noTone(SPEAKERPIN, SPEAKERCHANNEL); // frees up the PWM channel for next use, and sets pin low, I think.    
      Serial.print(Hertz);
      Serial.println(" ALERT!");
      Hertz = Hertz - ToneSpacing;
  }
} // end AlarmSound()

/////////////////////////////////////////////////////////////////
void ProcessAlarms()  // Process various alarm conditions
{
  if (OverTemperature)
  {
    ledcWrite(FAN_PWM_CHANNEL, 255); // Turn the box fans on full speed.
                                     // At some point, I may use a proportional control of fan speed, but 
                                     //  this is good because you hear them coming on as an indicator that the 
                                     //  CNC is working very hard or something is wrong.
    AlarmCondition = true;
    Serial.println("OverTemperature!!");
    AlarmSound();
  }
  
  if (RtFanStalled || LftFanStalled || BotFanStalled ) // The pulsein() function reached its timeout or fan is very slow.
  {
    AlarmCondition = true;
    Serial.println(" Alarm! A fan is stalled! ");
    AlarmSound();
  }
  // else if [other alarm conditions...]
  if (AlarmCondition) // put a message on the screen and change background colour
  {
   tft.fillScreen(TFT_YELLOW);
   tft.setTextColor(TFT_BLACK, TFT_YELLOW); // Set the font colour AND the background colour
   tft.setCursor(0,0,4); // 
   tft.println("ALERT!!");
   if (OverTemperature)
   {
    tft.println("OVERTEMP!!");
   }
   if (RtFanStalled) 
   {
    tft.println("Right");
    tft.println("fan");
    tft.println("STALLED!");
   }
   if (LftFanStalled) 
   {
    tft.println("Left");
    tft.println("fan");
    tft.println("STALLED!");
   }
   if (BotFanStalled) 
   {
    tft.println("Bottom");
    tft.println("fan");
    tft.println("STALLED!");
   }
   delay(1000);
  } 
 
  if (!RtFanStalled && !LftFanStalled && !BotFanStalled && !OverTemperature)
  {  // put things back to normal.
   AlarmCondition = false;     
   ledcWrite(FAN_PWM_CHANNEL, FANSPEED_LOW); // Turn down the box fans
   tft.fillScreen(TFT_BLACK);
   tft.setTextColor(TFT_MAGENTA, TFT_BLACK); // Set the font colour AND the background colour
   tft.setCursor(0,0,4); // 
   tft.println(" MPCNC");
   tft.println(" Temps");
   tft.setTextColor(TFT_YELLOW, TFT_BLACK); // Set the font colour AND the background colour        
  }
} // end ProcessAlarms()

///////////////////////////////////////////////////////
int ReadFanSpeed(int TachPin) // returns RPM
// Used 4-wire fans that include both the PWM speed control and a tachometer line. I am using Akasa CPU fans.
 /* The readings were full of noise when I just used pinmode(tachpin, INPUT_PULLUP);
  *  I needed to pull them up more stiffly, using a 1.8k pullup resistor to Vcc.
  *  I also added a 220nF capacitor soldered next to the microprocessor, to short the HF noise to ground.
  *  That was not enough filtering, so I added 680ohm resistors in series with the Tach signals. That should
  *  make a low-pass filter with cutoff frequency of about 1kHz.
  *  And since a tach pulse from the fan goes from Vcc (3.3V) to ground and back, pulseIn() should be set to LOW.
  */
  /* pulseIn() Syntax:
  pulseIn(pin, value)
  pulseIn(pin, value, timeout)
  Returns the length of the pulse in microseconds.
  pin: the number of the pin on which you want to read the pulse. (int)
  value: type of pulse to read: either LOW (HIGH->LOW first) or HIGH (LOW->HIGH first). 
  e.g., value == HIGH: if the pin is already high when the function is called, it will 
  wait for the pin to go LOW and then HIGH before it starts counting. 
  (int) timeout (optional): the number of microseconds to wait for the pulse to be completed: 
  NOTE: the function returns 0 if no complete pulse was received within the timeout. 
  Default is one second (unsigned long). I set it to 100ms with MaxFanTachPulseuS.
  */
 {
  //Serial.print(" Reading fan tachometer...");
  int LocalFanRPM;
  int loopCounter = 0;
  float FanPulseSum = 0.0;
  int PulsesToAverage = 10;
  int LeftFanStallErrors = 0; // This is the outflow fan on my controller box
  int RightFanStallErrors = 0; // Inflow fan. Filtered with a thin layer of gauze.
  int BottomFanStallErrors = 0; // Gauze-filtered inflow just beneath the stepper drivers, which are arranged in a pentagon to make a cooling tower.
  for (uint8_t i = 1; i <= (PulsesToAverage); i++)  // collect PulsesToAverage tachometer pulse lengths to average. Two pulses per revolution.                                     
  {
    loopCounter++;
    Serial.print(i);
    Serial.print(". Tach pin #");
    Serial.print(TachPin);
    FanTachPulseLength = (PULSEIN_CORRECTION_FACTOR * pulseIn(TachPin, LOW, MaxFanTachPulseuS));  // times out in (MaxFanTachPulseuS/1000) ms = 5 Hz if less than 150 rpm and returns zero. 
    if (FanTachPulseLength == 0) // Because fan is stalled
    {     
     if (TachPin == LEFTFANTACHPIN) {LeftFanStallErrors++ ;}   // Fan not moving.
     if (TachPin == RIGHTFANTACHPIN) {RightFanStallErrors++ ;} // Increment the error counts.
     if (TachPin == BOTTOMFANTACHPIN) {BottomFanStallErrors++ ;} // Tach lines have a lot of noise, so we are looking for a number of these.
     FanTachPulseLength = MaxFanTachPulseuS; 
    }
    
    Serial.print(" Tach sample: " );
    Serial.println(FanTachPulseLength);
    if (FanTachPulseLength > 3000.0)  FanPulseSum += FanTachPulseLength; // (3ms) Only add the Tachometer measurement to the average if it is reasonable, 5-200 ms
    else   i-- ; // if number was out of range, sample again.   
    if (loopCounter > (PulsesToAverage + 3)) i = PulsesToAverage; // don't get stuck here.
  }
  if (LeftFanStallErrors >= (PulsesToAverage - 2)) {LftFanStalled = true;} // Set or reset any needed alarm flags. 
  else LftFanStalled = false;  // reasonable fan speed
  if (RightFanStallErrors >= (PulsesToAverage - 2)) {RtFanStalled = true;} // Error must persist for long enough to rule out tach line noise.
  else RtFanStalled = false;
  if (BottomFanStallErrors >= (PulsesToAverage - 2)) {BotFanStalled = true;}
  else BotFanStalled = false;  
  ProcessAlarms();
  float FanPulseAvg = (FanPulseSum / float(PulsesToAverage)); // calculate the average.
  Serial.print(" Avg fan pulse duration (uS): ");
  Serial.print(FanPulseAvg); // I am doing this in steps because mixing floats and ints in one big equation kept giving me weird errors.
   float uSperRev = FanPulseAvg*4 ; //EG FanPulseAvg = 9000 uS: uSperRev = 36,000
   float RevPerUs =  1/ uSperRev  ; // 2.77e-5 RevPerUs
   float RevPerSec = RevPerUs * 1000000; // 27.7 RevPerSec
   LocalFanRPM = RevPerSec * 60; // 1,667 RPM
  // 
  // pulseIn() measures half a period (from HIGH to LOW transition (or from LOW to HIGH))
                  // If there are two pulses per rev, that's 4 periods.
                  // e.g., 20,000 usec = 20ms pulse: 80ms/rev. (1,000,000*60) us/min so 15,000,000 us/quarter-rev.                                           
  Serial.print("  Fan RPM: ");
  Serial.println(LocalFanRPM);
  return LocalFanRPM;  // TO FIX: this is not returning anything. Why?
 }  // end ReadFanSpeed()

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop(void)  // MAIN LOOP
{
  long startmillis = millis();
  //Serial.print("Requesting temperatures...");
  //DS18B20 NEEDS THE FOLLOWING LINE: 
  sensors.requestTemperatures();   // temperature request to all devices on the bus
  int elapsedmillis = millis() - startmillis;
  //Serial.println("DONE. ");
  //Serial.print(elapsedmillis);  // seems to be 503ms for precision of 12
  //Serial.println(" ms conversion time");
  
  float Hottest_temp = 0.0;
   
    // Reading the fan tach lines caused a lot of spurious alarms when VFD was RUN at 0 spindle RPM.
  FanRPM = ReadFanSpeed(LEFTFANTACHPIN);
  Serial.print(" L fan speed: ");
  Serial.println(FanRPM); 
  FanRPM = ReadFanSpeed(RIGHTFANTACHPIN);
  Serial.print(" R fan speed: ");
  Serial.println(FanRPM);
  FanRPM = ReadFanSpeed(BOTTOMFANTACHPIN);
  Serial.print(" Bot. fan speed: ");
  Serial.println(FanRPM);

  tft.setCursor(0,55,4); // sets cursor location and font number to use for next tft.println()
  /*  The VFD temp sensor wire was causing interference that reset the ESP32.
  float VFDtemp = printTemperature(VFDtempAddr);
  Hottest_temp = max(Hottest_temp, VFDtemp);
  tft.print("VFD: ");
  tft.print(abs((int)VFDtemp));
  tft.println("   ");
  Serial.print("VFD: ");
  Serial.println(VFDtemp);
*/
  float BoxAirTemp = printTemperature(BoxAirTempAddr);
  Hottest_temp = max(Hottest_temp, BoxAirTemp);
  tft.print("Box: ");
  tft.print(abs((int)BoxAirTemp));
  tft.println("   ");
  Serial.print("Box: ");
  Serial.print(BoxAirTemp);
  
  float X1_temp = printTemperature(X1_temp_addr);
  Hottest_temp = max(Hottest_temp, X1_temp);
  tft.print("X1:    ");
  tft.print(abs((int)X1_temp));
  tft.println("   ");
  Serial.print(" X1:  ");
  Serial.print(X1_temp);
  
  float X2_temp = printTemperature(X2_temp_addr);
  Hottest_temp = max(Hottest_temp, X2_temp);
  tft.print("X2:    ");
  tft.print(abs((int)X2_temp));
  tft.println("   ");
  Serial.print(" X2: ");
  Serial.print(X2_temp);
  
  float Y1_temp = printTemperature(Y1_temp_addr);
  Hottest_temp = max(Hottest_temp, Y1_temp);
  tft.print("Y1:    ");
  tft.print(abs((int)Y1_temp));
  tft.println("   ");
  Serial.print(" Y1: ");
  Serial.print(Y1_temp);
  
  float Y2_temp = printTemperature(Y2_temp_addr);
  Hottest_temp = max(Hottest_temp, Y2_temp);
  tft.print("Y2:    ");
  tft.print(abs((int)Y2_temp));
  tft.println("   ");
  Serial.print(" Y2:");
  Serial.print(Y2_temp);
    
  float Z_temp = printTemperature(Z_temp_addr);
  Hottest_temp = max(Hottest_temp, Z_temp);
  tft.print("Z:       ");
  tft.print(abs((int)Z_temp));
  tft.println("   ");
  Serial.print(" Z:");
  Serial.print(Z_temp);
  tft.print("Fan ");
  tft.println(FanRPM);

  Serial.print(" Hottest temp: ");
  Serial.println(Hottest_temp);
  if (Hottest_temp > MAXTEMP) OverTemperature = true;
  else OverTemperature = false;
    
 // tft.unloadFont(); // Remove the font to recover memory used
  delay(1500); // so it will keep the temps on screen for a while before blanking.
  ProcessAlarms(); // also blanks screen if no alarms.
  Serial.print("Loop timestamp: ");
  Serial.println(millis());
}  // END main loop
