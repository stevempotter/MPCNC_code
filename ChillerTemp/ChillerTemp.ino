
// Steve M. Potter 5 April 2021   steve@stevempotter.tech
// To monitor the spindle liquid cooling for the Portable MPCNC I built. This uses the TTGO LilyGO V1.1 ESP32 board
//  which as a small TFT full-colour screen 135x240 pixels. I bought them from Tinytronics for about 8 euros each.
// I glued a pair of DS18B20 digital temperature sensors to small pieces of copper tubing on the inflow and
// out flow of the radiator (CPU cooler). The temperatures of the pre- and Post-cooling sensors are indicated
// with a bar chart, and the difference between them with a number (of degrees C) at the top of the display.
// Still left to do:  
//  -Set up push notification alarms with Pushbullet
//  -Flow sensor and no-flow alarm

#include <OneWire.h>
#include <DallasTemperature.h>
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

#define ONE_WIRE_BUS 2 // This wire is pulled up to 3.3V with a 4k7 resistor.
#define BEEPER 32  // TTGO LilyGO board pins that don't work as digital IO: 36,37,38,39,21,22
#define TEMPERATURE_PRECISION 12
#define MAXTEMP 40 // If cooling fluid gets hotter than this, Alarm is sounded.

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);
int Sensor_count;
float HotTemp = -1.0; 
float CoolTemp = -1.0;
float TempDiff = 0.0;
bool Alarm = false; 

// arrays to hold DS18B20 device addresses. EDIT for the devices you are using.
DeviceAddress CoolTempAddr = {0x28, 0x20, 0x9B, 0x07, 0xD6, 0x01, 0x3C, 0xF8};
DeviceAddress HotTempAddr = {0x28, 0xAD, 0x1C, 0x07, 0xD6, 0x01, 0x3C, 0xC2};

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

void setup(void)
{
  // start serial port
  Serial.begin(115200);
  pinMode(BEEPER, OUTPUT);
  tft.begin();
    // Initialise the TFT registers
  tft.init(); // Do I need both of these?
  tft.setRotation(0); // 0 is Portrait mode
  Serial.println("Chiller Monitor with two DS18B20 sensors! ");
  
  // Start up the temp sensors library
  sensors.begin();
  //delay(1000);

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
  Serial.print("CoolTemp Address: ");
  printAddress(CoolTempAddr);
  Serial.println();
  Serial.print("HotTemp Address: ");
  printAddress(HotTempAddr);
  Serial.println();

  // Set the resolution to 9 - 12 bits. You really only need to do this once. It gets saved in non-volatile memory of the sensor.
  // 9 bits precision is about half a degree Celcius. The accuracy across devices is only about that good.
  // Within one device, accuracy is very good, so use more bits if tiny changes in temperature are important.
  // Use fewer bits for faster/more frequent readings. Resolution can be 9,10,11, or 12 bits.
  // 12 bits: 750ms nom. conversion time, 0.06 C precision.
  // It seems that the conversions all happen in parallel, so adding more devices does not add much conversion time.

  sensors.setResolution(HotTempAddr, TEMPERATURE_PRECISION);
  sensors.setResolution(CoolTempAddr, TEMPERATURE_PRECISION);

  Serial.print("HotTemp bits: ");
  Serial.print(sensors.getResolution(HotTempAddr), DEC);
  Serial.println();
  Serial.print("CoolTemp bits: ");
  Serial.print(sensors.getResolution(CoolTempAddr), DEC);
  Serial.println();

  //Draw unchanging stuff:
  //Background:
        tft.fillScreen(TFT_BLACK);
// Words: 
        tft.setTextColor(TFT_RED, TFT_BLACK); // Set the font colour AND the background colour
        tft.drawString("IN", 10, 5, 4);
        tft.setTextColor(TFT_SKYBLUE, TFT_BLACK);
        tft.drawString("OUT", 85, 5, 4);
        tft.setTextColor(TFT_YELLOW, TFT_BLACK);
        tft.drawString("diff", 40, 10, 4);
  //Scale:
        tft.setTextColor(TFT_RED, TFT_BLACK);
        tft.drawString("50", 52, 70, 4);
        tft.setTextColor(TFT_ORANGE, TFT_BLACK);
        tft.drawString("40", 52, 100, 4);
        tft.drawString("30", 52, 130, 4);
        tft.setTextColor(TFT_YELLOW, TFT_BLACK);
        tft.drawString("20", 52, 160, 4);
        tft.drawString("10", 49, 190, 4);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.drawString("0 C", 50, 220, 4);


   tft.loadFont(AA_FONT_LARGE); // Load another different font to be used for changing values
 
  digitalWrite(BEEPER, HIGH); // just to be sure the beeper and this code are working.
  delay(200);
  digitalWrite(BEEPER, LOW);
} //end setup


// Function to print the temperature for a DS18B20 device
float printTemperature(DeviceAddress deviceAddress)
{
  float tempC = sensors.getTempC(deviceAddress);
  printAddress(deviceAddress);
  Serial.print(" Temp C: ");
  Serial.println(tempC);
  return tempC;
 // Serial.print(" Temp F: ");
//  Serial.print(DallasTemperature::toFahrenheit(tempC));
}

// Function to print a DS18B20 device's resolution
void printResolution(DeviceAddress deviceAddress)
{
  Serial.print("Resolution: ");
  Serial.print(sensors.getResolution(deviceAddress));
  Serial.println();
}

// Function to print information about a DS18B20 device
void printData(DeviceAddress deviceAddress)
{
  Serial.print("DS18B20 Address: ");
  printAddress(deviceAddress);
  Serial.print(" ");
  printTemperature(deviceAddress);
  Serial.println();
}


void loop(void)  // MAIN LOOP
{
  long startmillis = millis();
  Serial.print("Requesting temperatures...");
  sensors.requestTemperatures();   // temperature request to all devices on the bus
  int elapsedmillis = millis() - startmillis;
  Serial.print("DONE. ");
  Serial.print(elapsedmillis);  // seems to be 503ms for precision of 12
  Serial.println(" ms conversion time");

  HotTemp = printTemperature(HotTempAddr);
  if (HotTemp > MAXTEMP) {
    Alarm = true; // to be used later for pushing alarms to phone.
    for (int i = 1; i <= 10; i++) {  // Make a trill hard to ignore.
      digitalWrite(BEEPER, HIGH);
      delay(100);
      digitalWrite(BEEPER, LOW);
      delay(100);
      } 
    }
    else {  // Turn off alarm if it was on and the temperature has gone down.
      Alarm = false;
      digitalWrite(BEEPER, LOW);
    }
  
  Serial.print("Hot:");
  Serial.print(HotTemp);
  Serial.print(" Cool:");
  CoolTemp = printTemperature(CoolTempAddr);
  Serial.print(CoolTemp);
  TempDiff = HotTemp - CoolTemp;
  Serial.print(" Diff: ");
  Serial.print(TempDiff);
  Serial.println(" degrees C");
  int TempDiffInt = abs(round(TempDiff)); // Out temp will always be less than In.
 
 // Display hot and cool temps as vertical bars.
 // Cover previous bars:
  tft.fillRect(10, 50, 30, 190, TFT_BLACK); // (ULX, ULY, w, h, TFT_color)
  tft.fillRect(95, 50, 30, 190, TFT_BLACK); // (ULX, ULY, w, h, TFT_color)
 // Convert from degrees C to pixels:
  int HotHeight =  map(round(HotTemp), 0, 60, 230, 50); // 30 pixels per 10 degrees. Bar is 180 pixels high at 60C.
  tft.fillRect(10, HotHeight, 30, (230 - HotHeight), TFT_RED); // (ULX, ULY, w, h, TFT_color)

  int CoolHeight = map(round(CoolTemp), 0, 60, 230, 50);
  tft.fillRect(95, CoolHeight, 30, (230 - CoolHeight), TFT_SKYBLUE); // (ULX, ULY, w, h, TFT_color)

// Indicate the temperature difference:
  tft.fillRect (46, 33, 40, 37, TFT_BLACK); // Overprint with a filled rectangle
  tft.setTextColor(TFT_GREENYELLOW, TFT_BLACK);
  tft.drawString(String(TempDiffInt),46, 33, 4);// or drawNumber(long intNumber, int32_t x, int32_t y)

  
 // tft.unloadFont(); // Remove the font to recover memory used

  delay(1497); // so it will be about 2 sec per loop.

}  // END main loop
