/*****************************************************************************

Reads a decagon CTD sensor and onboard sensors, including an analog input for the
battery voltage. Values are then uploaded to monitormywatershed.org.


Modified by Rachel Murray from:
Written By:  Sara Damiano (sdamiano@stroudcenter.org)
Development Environment: PlatformIO
Hardware Platform: EnviroDIY Mayfly Arduino Datalogger
Software License: BSD-3.
  Copyright (c) 2017, Stroud Water Research Center (SWRC)
  and the EnviroDIY Development Team
This example sketch is written for ModularSensors library version 0.23.10
This shows most of the standard functions of the library at once.
DISCLAIMER:
THIS CODE IS PROVIDED "AS IS" - NO WARRANTY IS GIVEN.
*****************************************************************************/

// ==========================================================================
//    Include the base required libraries
// ==========================================================================
#include <Arduino.h>  // The base Arduino library
#include <EnableInterrupt.h>  // for external and pin change interrupts
#include <LoggerBase.h>  // The modular sensors library


// ==========================================================================
//    Data Logger Settings
// ==========================================================================
// The library version this example was written for
const char *libraryVersion = "0.23.16";
// The name of this file
const char *sketchName = "turbidity_ctd.ino";
// Logger ID, also becomes the prefix for the name of the data file on SD card
const char *LoggerID = "Tarawera";
// How frequently (in minutes) to log data
const uint8_t loggingInterval = 5; // set to 2 minutes -rm
// Your logger's timezone.
const int8_t timeZone = 12;  // Eastern Standard Time
// NOTE:  Daylight savings time will not be applied!  Please use standard time!
float minimum_bat_voltage = 11.0; // defining minimum and moderate battery voltages
float moderate_bat_voltage = 11.5; // -rm

// ==========================================================================
//    Primary Arduino-Based Board and Processor
// ==========================================================================
#include <sensors/ProcessorStats.h>

const long serialBaud = 115200;   // Baud rate for the primary serial port for debugging
const int8_t greenLED = 8;        // MCU pin for the green LED (-1 if not applicable)
const int8_t redLED = 9;          // MCU pin for the red LED (-1 if not applicable)
const int8_t buttonPin = 21;      // MCU pin for a button to use to enter debugging mode  (-1 if not applicable)
const int8_t wakePin = A7;        // MCU interrupt/alarm pin to wake from sleep
// Set the wake pin to -1 if you do not want the main processor to sleep.
// In a SAMD system where you are using the built-in rtc, set wakePin to 1
const int8_t sdCardPwrPin = -1;     // MCU SD card power pin (-1 if not applicable)
const int8_t sdCardSSPin = 12;      // MCU SD card chip select/slave select pin (must be given!)
const int8_t sensorPowerPin = 22;  // MCU pin controlling main sensor power (-1 if not applicable)

// Create the main processor chip "sensor" - for general metadata
const char *mcuBoardVersion = "v0.5b";
ProcessorStats mcuBoard(mcuBoardVersion);

// =================================================================
// Turbidity correction formula
//==========================================

// correction for raw voltage from turbidimeter using formula
// of the form ax^2 + bx + c

float turbidity_correction_A = 0.0;
float turbidity_correction_B = 1.0; 
float turbidity_correction_C = 0.0;

// ==========================================================================
//    Maxim DS3231 RTC (Real Time Clock)
// ==========================================================================
#include <sensors/MaximDS3231.h>

// Create a DS3231 sensor object
MaximDS3231 ds3231(1);



// ==========================================================================
//    Decagon CTD Conductivity, Temperature, and Depth Sensor
// ==========================================================================
#include <sensors/DecagonCTD.h>

const char *CTDSDI12address = "1";  // The SDI-12 Address of the CTD
const uint8_t CTDNumberReadings = 6;  // The number of readings to average
const int8_t SDI12Power = sensorPowerPin;  // Pin to switch power on and off (-1 if unconnected)
const int8_t SDI12Data = 7;  // The SDI12 data pin

// Create a Decagon CTD sensor object
DecagonCTD ctd(*CTDSDI12address, SDI12Power, SDI12Data, CTDNumberReadings);


// ==========================================================================
//    External Voltage via TI ADS1115
// ==========================================================================
#include <sensors/ExternalVoltage.h>

const int8_t ADSPower = sensorPowerPin;  // Pin to switch power on and off (-1 if unconnected)
const int8_t ADSChannel = 3;  // The ADS channel of interest
const float dividerGain = 5;  //  Default 1/gain for grove voltage divider is 10x
const uint8_t ADSi2c_addr = 0x48;  // The I2C address of the ADS1115 ADC
const uint8_t VoltReadsToAvg = 1;  // Only read one sample

// Create an External Voltage sensor object
ExternalVoltage extvolt(ADSPower, ADSChannel, dividerGain, ADSi2c_addr, VoltReadsToAvg);

// Create a voltage variable pointer
Variable *extvoltV = new ExternalVoltage_Volt(&extvolt, "12345678-abcd-1234-ef00-1234567890ab");

//=============================================================
// Turbidity sensor as a calculated variable
//===========================================================



float calculateTurbidity(void)
{
    float calculatedResult = -9999;  // Always safest to start with a bad value
    Serial.print(F(" Calculating Turbidity... "));
    
    // Reading analog 1 pin to get raw value
    int sensorValue = analogRead(A2);
    Serial.print(F(" Sensor value is ... "));
    Serial.println(sensorValue);
    // converting to voltage 
    float voltage = sensorValue * (5.0 / 1024.0);

    // adjusting turbidity using formula ax^2 + bx + c

    float Ta = turbidity_correction_A;
    float Tb = turbidity_correction_B;
    float Tc = turbidity_correction_C;

     if (sensorValue != -9999)  // make sure input is good
     {
         calculatedResult = Ta*(voltage*voltage) + Tb*voltage + Tc;
            Serial.print(F(" Calculated result is ... "));
            Serial.println(calculatedResult);
     }
    return calculatedResult;
}

// Properties of the calculated variable
const uint8_t calculatedVarResolution = 2;  // The number of digits after the decimal place
const char *calculatedVarName = "Turbidity";  // This must be a value from http://vocabulary.odm2.org/variablename/
const char *calculatedVarUnit = "NTU";  // This must be a value from http://vocabulary.odm2.org/units/
const char *calculatedVarCode = "Turb";  // A short code for the variable
const char *calculatedVarUUID = "12345678-abcd-1234-ef00-1234567890ab";  // The (optional) universallly unique identifier

// Finally, Create a calculated variable pointer and return a variable pointer to it
Variable *TurbidityVariable = new Variable(calculateTurbidity, calculatedVarResolution,
                                       calculatedVarName, calculatedVarUnit,
                                       calculatedVarCode, calculatedVarUUID);



// ==========================================================================
//    Creating the Variable Array[s] and Filling with Variable Objects
// ==========================================================================

// FORM1: Create pointers for all of the variables from the sensors,
// at the same time putting them into an array
// NOTE:  Forms one and two can be mixed
Variable *variableList[] = {
 
    new DecagonCTD_Cond(&ctd, "12345678-abcd-1234-ef00-1234567890ab"), 
    new DecagonCTD_Temp(&ctd, "12345678-abcd-1234-ef00-1234567890ab"),
    new DecagonCTD_Depth(&ctd, "12345678-abcd-1234-ef00-1234567890ab"),
    new ProcessorStats_Battery(&mcuBoard, "12345678-abcd-1234-ef00-1234567890ab"),
    new MaximDS3231_Temp(&ds3231, "12345678-abcd-1234-ef00-1234567890ab"),
 //   new ExternalVoltage_Volt(&extvolt, "12345678-abcd-1234-ef00-1234567890ab"),
    TurbidityVariable,
};

// Count up the number of pointers in the array
int variableCount = sizeof(variableList) / sizeof(variableList[0]);

// Create the VariableArray object
VariableArray varArray(variableCount, variableList);

// ==================================================================
 // Analog battery voltage reading -rm
// ==================================================================
// mooched from Conrad's pump controller script -rm

float getBatteryVoltage(void)
{
     if (extvolt.sensorValues[0] == -9999) extvolt.update();
     return extvolt.sensorValues[0];
 //   return 13.0;
}

// ==========================================================================
//     The Logger Object[s]
// ==========================================================================

// Create a new logger instance
Logger dataLogger(LoggerID, loggingInterval, &varArray);

const char *registrationToken = "12345678-abcd-1234-ef00-1234567890ab";   // Device registration token
const char *samplingFeature = "12345678-abcd-1234-ef00-1234567890ab";     // Sampling feature UUID


// ==========================================================================
//    Working Functions
// ==========================================================================

// Flashes the LED's on the primary board
void greenredflash(uint8_t numFlash = 4, uint8_t rate = 75)
{
    for (uint8_t i = 0; i < numFlash; i++) {
        digitalWrite(greenLED, HIGH);
        digitalWrite(redLED, LOW);
        delay(rate);
        digitalWrite(greenLED, LOW);
        digitalWrite(redLED, HIGH);
        delay(rate);
    }
    digitalWrite(redLED, LOW);
}


// Read's the battery voltage
// original from monitormywatershed, for use without telemetry
// NOTE: This will actually return the battery level from the previous update!
// I don't need this because I am using an analog pin to read the voltage -rm
//float getBatteryVoltage()
//{
//    if (mcuBoard.sensorValues[0] == -9999) mcuBoard.update();
//    return mcuBoard.sensorValues[0];
//}


// ==========================================================================
// Main setup function
// ==========================================================================
void setup()
{
    // Wait for USB connection to be established by PC
    // NOTE:  Only use this when debugging - if not connected to a PC, this
    // could prevent the script from starting
    #if defined SERIAL_PORT_USBVIRTUAL
      while (!SERIAL_PORT_USBVIRTUAL && (millis() < 10000)){}
    #endif

    // Start the primary serial connection
    Serial.begin(serialBaud);

    // Print a start-up note to the first serial port
    Serial.print(F("Now running "));
    Serial.print(sketchName);
    Serial.print(F(" on Logger "));
    Serial.println(LoggerID);
    Serial.println();

    Serial.print(F("Using ModularSensors Library version "));
    Serial.println(MODULAR_SENSORS_VERSION);

    if (String(MODULAR_SENSORS_VERSION) !=  String(libraryVersion))
        Serial.println(F(
            "WARNING: THIS EXAMPLE WAS WRITTEN FOR A DIFFERENT VERSION OF MODULAR SENSORS!!"));

     


    // Set up pins for the LED's
    pinMode(greenLED, OUTPUT);
    digitalWrite(greenLED, LOW);
    pinMode(redLED, OUTPUT);
    digitalWrite(redLED, LOW);
    
    // Blink the LEDs to show the board is on and starting up
    greenredflash();

    if (sensorPowerPin >= 0)
    {
        pinMode(sensorPowerPin, OUTPUT);
        digitalWrite(sensorPowerPin, LOW);
    }
    // print battery voltage Settings
    Serial.print(F("Logging will stop at battery voltage lower than "));
    Serial.print(minimum_bat_voltage);
    Serial.print(F(" volts. "));
    // Set the timezones for the logger/data and the RTC
    // Logging in the given time zone
    Logger::setLoggerTimeZone(timeZone);
    // It is STRONGLY RECOMMENDED that you set the RTC to be in UTC (UTC+0)
    Logger::setRTCTimeZone(0);

    // Attach the information pins to the logger

    dataLogger.setLoggerPins(wakePin, sdCardSSPin, sdCardPwrPin, buttonPin, greenLED);

    // Begin the logger
    dataLogger.begin();

 
    Serial.print ("current voltage = ");
    Serial.println (getBatteryVoltage());
    Serial.print ("");


    // Note:  Please change these battery voltages to match your battery

    // Set up the sensors, except at lowest battery level
    if (getBatteryVoltage() > minimum_bat_voltage)
    {
        Serial.println(F("Setting up sensors..."));
        varArray.setupSensors();
    }



    // Create the log file, adding the default header to it
    // Do this last so we have the best chance of getting the time correct and
    // all sensor names correct
    // Writing to the SD card can be power intensive, so if we're skipping
    // the sensor setup we'll skip this too.
    if (getBatteryVoltage() > minimum_bat_voltage)
    {
        Serial.println(F("Setting up file on SD card"));
        dataLogger.turnOnSDcard(true);  // true = wait for card to settle after power up
        dataLogger.createLogFile(true);  // true = write a new header
        dataLogger.turnOffSDcard(true);  // true = wait for internal housekeeping after write
    }

    // Call the processor sleep
    Serial.println(F("Putting processor to sleep\n"));
    dataLogger.systemSleep();
}


// ==========================================================================
// Main loop function
// ==========================================================================

// Use this short loop for simple data logging and sending
// /*
void loop()
{

    
    Serial.print ("current voltage = ");
    Serial.println (getBatteryVoltage());
    Serial.print ("");


    // Note:  Please change these battery voltages to match your battery
    // At very low battery, just go back to sleep
    if (getBatteryVoltage() < minimum_bat_voltage)
    {
        dataLogger.systemSleep();
    }
    // If the battery is good, send the data to the world
    else
    {
        dataLogger.logData();
    }
}