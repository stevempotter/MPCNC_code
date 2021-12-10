
// Steve M. Potter 15 May 2021   steve@stevempotter.tech
// To monitor the XYZ stepper currents for the Portable MPCNC I built. This uses the TTGO LilyGO V1.1 ESP32 board
//  which as a small TFT full-colour screen 135x240 pixels. I bought them from Tinytronics for about 8 euros each.
// The sensor is the INA226 breakout board (Tinytronics.nl) current/voltage/power monitor connected to the low side of the 24V supply to the
// stepper driver boards' Vmot pins. The INA266 works fine either above or below the load. I put it below so that the wires leading to
// and from it are close to ground potential.
//
// For measuring voltage (and thus power) the INA266 has a Vin max of 36VDC. If using a higher supply, use a voltage divider and conversion factor.
// 

#include <Wire.h>
#include <INA226_WE.h> // https://github.com/wollewald/INA226_WE
#define I2C_ADDRESS 0x40

INA226_WE ina226(I2C_ADDRESS);
//#include <OneWire.h>
//#include <DallasTemperature.h>
//#include <Preferences.h> // place on the ESP32 to store non-volatile variables.
//#include "WiFi.h"
#include <SPI.h>
#include <TFT_eSPI.h>       // Hardware-specific library for display on TTGO board
TFT_eSPI tft = TFT_eSPI(); // TTGO LilyGO V1.1 screen is 240 X 135 pixels

// Fonts I may use. 
#include "NotoSansBold15.h"
#include "NotoSansBold36.h"
#define I2C_SDA 21
#define I2C_SCL 22


#define AA_FONT_SMALL NotoSansBold15
#define AA_FONT_LARGE NotoSansBold36
#define MAX_AMPS 6.25 // This is the max for the 24V power supply.
#define LOOPDELAYMS 200

float Amps = 10.23; // just some dummy values to show if sensor is not hooked up.
String AmpsStr;
int Watts = 666;
float Volts = 22.2;
String VoltsStr;


/*  // May want to add temp sensors to the stepper motors later...
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

// arrays to hold DS18B20 device addresses
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
*/
void setup(void)
{
  // start serial port
  Serial.begin(115200);
  //pinMode(BEEPER, OUTPUT);
  Wire.begin (I2C_SDA, I2C_SCL);
  ina226.init();
   //set resistor and current range. Resistor value in ohm, current range in A
  ina226.setResistorRange(0.002, 20.0);
  ina226.setConversionTime(CONV_TIME_8244); // that is 8.244 ms, the longest option for the least noise.
  // The library can also do averaging. e.g.,  ina226.setAverage(AVERAGE_16);
  // ina226.waitUntilConversionCompleted(); //Uncomment if you are getting zeros for first reading.

  tft.init(); // Initialise the TFT registers
  tft.setRotation(0); // 0 is Portrait mode
  Serial.println("Stepper motor current monitor using Adafruit INA260 Breakout board ");

  /*
  // Start up the temp sensors library
  sensors.begin();
  //delay(1000);
*/

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

  /*
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
*/

  //Draw unchanging stuff:
  //Background:
        tft.fillScreen(TFT_BLACK);
// Words: 
        tft.setTextColor(TFT_RED, TFT_BLACK); // Set the font colour AND the background colour
        tft.drawString("STEPPERS", 5, 5, 4);// (X, Y, Size)
        
  //Scale: // maybe add this after I know what the range is.
  /*
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
 */

   tft.loadFont(AA_FONT_LARGE); // Load another different font to be used for changing values
   tft.setTextColor(TFT_YELLOW, TFT_BLACK);
   tft.drawString("A", 110, 53, 4);

   tft.setTextColor(TFT_MAGENTA, TFT_BLACK);
   tft.drawString("W", 100, 113, 4);

   tft.setTextColor(TFT_GREEN, TFT_BLACK);
   tft.drawString("V", 110, 173, 4);

} //end setup

/*
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
}*/

void loop(void)  // MAIN LOOP
{
  long startmillis = millis();
  Serial.print("Requesting data...");
 // sensors.requestTemperatures();   // temperature request to all devices on the bus

  float shuntVoltage_mV = 0.0;
  float loadVoltage_V = 0.0;
  float busVoltage_V = 0.0;
  float current_mA = 0.0;
  float power_mW = 0.0; 

  ina226.readAndClearFlags();
  shuntVoltage_mV = ina226.getShuntVoltage_mV(); // Breakout board as 2 milliohm shunt resistor.
  Volts = ina226.getBusVoltage_V();
  current_mA = ina226.getCurrent_mA();
  Amps = (current_mA / 1000); // Device reads out mA.
  power_mW = ina226.getBusPower();
  Watts = (power_mW / 1000); // Device reads out mW.
 
  Serial.print("Shunt Voltage [mV]: "); Serial.println(shuntVoltage_mV);
  
  AmpsStr = String(Amps, 2);
  Serial.print(AmpsStr);
  Serial.print("A  ");
  Serial.print(Watts);
  Serial.print("W  ");
  
  VoltsStr = String(Volts, 1);
  Serial.print(VoltsStr);
  Serial.println("V");
 
  int elapsedmillis = millis() - startmillis;
  Serial.print(elapsedmillis);  // 
  Serial.println(" ms to get data.");

 

 // Display current as vertical bar.
 // Cover previous bar:
  tft.fillRect(5, 35, 20, 200, TFT_BLACK); // (ULX, ULY, w, h, TFT_color)
 // Convert from Amps to pixels:
  //Amps = 6.2; // for testing the max size of the bar
  int AmpsHeight = mapfloat(Amps, 0, MAX_AMPS, 235, 35); // mapfloat(value, fromLow, fromHigh, toLow, toHigh)
                                                     // Bar can be up to 200 pixels high.
  tft.fillRect(5, AmpsHeight, 20, (230 - AmpsHeight), TFT_YELLOW); // (ULX, ULY, w, h, TFT_color)

 

// Print the data in big numbers.
  tft.fillRect (25, 50, 60, 30, TFT_BLACK); // Overprint with a filled rectangle
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.drawString(AmpsStr,35, 53, 6);// ULx, ULy, size or drawNumber(long intNumber, int32_t x, int32_t y)
 // tft.drawString("4.38",35, 53, 6);//

  tft.fillRect (25, 110, 50, 30, TFT_BLACK); // Overprint with a filled rectangle
  tft.setTextColor(TFT_MAGENTA, TFT_BLACK);
  tft.drawString(String(Watts),35, 113, 6);
 // tft.drawString("302",35, 113, 6);
  
  tft.fillRect (25, 170, 50, 30, TFT_BLACK); // Overprint with a filled rectangle
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawString(VoltsStr,35, 173, 6);
 // tft.drawString("24.3",35, 173, 6);
  
 // tft.unloadFont(); // Remove the font to recover memory used

  delay(LOOPDELAYMS); // 
}  // END main loop

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
