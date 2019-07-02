#include <dataPublisherBase.h>
#include <LoggerBase.h>
#include <LoggerModem.h>
#include <ModSensorDebugger.h>
#include <SensorBase.h>
#include <VariableArray.h>
#include <VariableBase.h>

/*****************************************************************************
Sketch modified for waikato mayfly deployment for Troy Baysden by
Chris Eager
Rachael Murray
Peter Jarman
02-7-2019



DRWI_NoCellular.ino
Written By:  Sara Damiano (sdamiano@stroudcenter.org)
Development Environment: PlatformIO
Hardware Platform: EnviroDIY Mayfly Arduino Datalogger
Software License: BSD-3.
  Copyright (c) 2017, Stroud Water Research Center (SWRC)
  and the EnviroDIY Development Team

This example sketch is written for ModularSensors library version 0.19.6

This sketch is an example of logging data to an SD card as should be used by
groups involved with The William Penn Foundation's Delaware River Watershed
Initiative at sites without cellular service.

DISCLAIMER:
THIS CODE IS PROVIDED "AS IS" - NO WARRANTY IS GIVEN.
*****************************************************************************/

// ==========================================================================
//    Include the base required libraries
// ==========================================================================
#include <Arduino.h>  // The base Arduino library
#include <EnableInterrupt.h>  // for external and pin change interrupts


// ==========================================================================
//    Data Logger Settings
// ==========================================================================
// The library version this example was written for
const char *libraryVersion = "0.19.6";
// The name of this file
const char *sketchName = "Okaro_1_15_min.ino";
// Logger ID, also becomes the prefix for the name of the data file on SD card
const char *LoggerID = "Okaro_1";
// How frequently (in minutes) to log data
const uint8_t loggingInterval = 15; //set to 1 minute for testing pj
// Your logger's timezone.
const int8_t timeZone =12;  // Auckland :note timezone wont take 2 digit number pj 
// NOTE:  Daylight savings time will not be applied!  Please use standard time!

const float battery_minimum = 11.0; // minimum allowable battery voltage
                                    //set for PS1270-12V7.0Ah battery 25/6/19 pj

// ==========================================================================
//    Primary Arduino-Based Board and Processor
// ==========================================================================
#include <sensors/ProcessorStats.h>

const long serialBaud = 57600;   // Baud rate for the primary serial port for debugging
const int8_t greenLED = 8;        // MCU pin for the green LED (-1 if not applicable)
const int8_t redLED = 9;          // MCU pin for the red LED (-1 if not applicable)
const int8_t buttonPin = 21;      // MCU pin for a button to use to enter debugging mode  (-1 if not applicable)
const int8_t wakePin = A7;        // MCU interrupt/alarm pin to wake from sleep
// Set the wake pin to -1 if you do not want the main processor to sleep.
// In a SAMD system where you are using the built-in rtc, set wakePin to 1
const int8_t sdCardPin = 12;      // MCU SD card chip select/slave select pin (must be given!)
const int8_t sensorPowerPin = 22; // MCU pin controlling main sensor power (-1 if not applicable)

// Create and return the main processor chip "sensor" - for general metadata
const char *mcuBoardVersion = "v0.5b";
ProcessorStats mcuBoard(mcuBoardVersion);


// ==========================================================================
//    Maxim DS3231 RTC (Real Time Clock)
// ==========================================================================
#include <sensors/MaximDS3231.h>

// Create and return the DS3231 sensor object
MaximDS3231 ds3231(1);


// ==========================================================================
//    CAMPBELL OBS 3 / OBS 3+ Analog Turbidity Sensor
// ==========================================================================
//#include <sensors/CampbellOBS3.h> //commented out pj

//const int8_t OBS3Power = sensorPowerPin;  // Pin to switch power on and off (-1 if unconnected)//commented out pj
//const uint8_t OBS3numberReadings = 10;//commented out pj
//const uint8_t ADSi2c_addr = 0x48;  // The I2C address of the ADS1115 ADC//commented out pj
// Campbell OBS 3+ Low Range calibration in Volts//commented out pj
//const int8_t OBSLowADSChannel = 0;  // The ADS channel for the low range output//commented out pj
//const float OBSLow_A = 0.000E+00;  // The "A" value (X^2) from the low range calibration//commented out pj
//const float OBSLow_B = 1.000E+00;  // The "B" value (X) from the low range calibration//commented out pj
//const float OBSLow_C = 0.000E+00;  // The "C" value from the low range calibration//commented out pj
// Create and return the Campbell OBS3+ LOW RANGE sensor object//commented out pj
//CampbellOBS3 osb3low(OBS3Power, OBSLowADSChannel, OBSLow_A, OBSLow_B, OBSLow_C, ADSi2c_addr, OBS3numberReadings);//commented out pj


// Campbell OBS 3+ High Range calibration in Volts
//const int8_t OBSHighADSChannel = 1;  // The ADS channel for the high range output  //commented out pj
//const float OBSHigh_A = 0.000E+00;  // The "A" value (X^2) from the high range calibration  //commented out pj
//const float OBSHigh_B = 1.000E+00;  // The "B" value (X) from the high range calibration  //commented out pj
//const float OBSHigh_C = 0.000E+00;  // The "C" value from the high range calibration  //commented out pj
// Create and return the Campbell OBS3+ HIGH RANGE sensor object
//CampbellOBS3 osb3high(OBS3Power, OBSHighADSChannel, OBSHigh_A, OBSHigh_B, OBSHigh_C, ADSi2c_addr, OBS3numberReadings);  //commented out pj


// ==========================================================================
//    Decagon CTD Conductivity, Temperature, and Depth Sensor
// ==========================================================================
#include <sensors/DecagonCTD.h>

const char *CTDSDI12address = "1";  // The SDI-12 Address of the CTD
const uint8_t CTDnumberReadings = 6;  // The number of readings to average
const int8_t SDI12Power = sensorPowerPin;  // Pin to switch power on and off (-1 if unconnected)
const int8_t SDI12Data = 7;  // The SDI12 data pin

// Create and return the Decagon CTD sensor object
DecagonCTD ctd(*CTDSDI12address, SDI12Power, SDI12Data, CTDnumberReadings);
  

// ==========================================================================
//    Creating the Variable Array[s] and Filling with Variable Objects
// ==========================================================================
#include <VariableArray.h>

// FORM1: Create pointers for all of the variables from the sensors,
// at the same time putting them into an array
Variable *variableList[] = {
    new DecagonCTD_Cond(&ctd, "12345678-abcd-1234-efgh-1234567890ab"),
    new DecagonCTD_Temp(&ctd, "12345678-abcd-1234-efgh-1234567890ab"),
    new DecagonCTD_Depth(&ctd, "12345678-abcd-1234-efgh-1234567890ab"),
    //new CampbellOBS3_Turbidity(&osb3low, "12345678-abcd-1234-efgh-1234567890ab", "TurbLow"), //removed pj
    //new CampbellOBS3_Turbidity(&osb3high, "12345678-abcd-1234-efgh-1234567890ab", "TurbHigh"),//removed pj
    new ProcessorStats_Batt(&mcuBoard, "12345678-abcd-1234-efgh-1234567890ab"),
    new MaximDS3231_Temp(&ds3231, "12345678-abcd-1234-efgh-1234567890ab")
};
// Count up the number of pointers in the array
int variableCount = sizeof(variableList) / sizeof(variableList[0]);

// Create the VariableArray object
VariableArray varArray(variableCount, variableList);


// ==========================================================================
//     The Logger Object[s]
// ==========================================================================
#include <LoggerBase.h>

// Create a new logger instance
Logger dataLogger(LoggerID, loggingInterval, sdCardPin, wakePin, &varArray);


// Device registration and sampling feature information
// This should be obtained after registration at http://data.envirodiy.org
// This is needed so the logger file will be "drag-and-drop" ready for manual
// upload to the portal.
const char *registrationToken = "12345678-abcd-1234-efgh-1234567890ab";   // Device registration token
const char *samplingFeature = "12345678-abcd-1234-efgh-1234567890ab";     // Sampling feature UUID


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
// NOTE: This will actually return the battery level from the previous update!
float getBatteryVoltage()
{
    if (mcuBoard.sensorValues[0] == -9999) mcuBoard.update();
    return mcuBoard.sensorValues[0];
}


// ==========================================================================
// Main setup function
// ==========================================================================
void setup()
{
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

    // Set up some of the power pins so the board boots up with them off
    // NOTE:  This isn't necessary at all.  The logger begin() function
    // should leave all power pins off when it finishes.
    if (sensorPowerPin >= 0)
    {
        pinMode(sensorPowerPin, OUTPUT);
        digitalWrite(sensorPowerPin, LOW);
    }

    // Set the timezone and offsets
    // Logging in the given time zone
    Logger::setTimeZone(timeZone);
    // Offset is the same as the time zone because the RTC is in UTC
    Logger::setTZOffset(timeZone);

    // Attach information pins to the logger
    dataLogger.setAlertPin(greenLED);
    dataLogger.setTestingModePin(buttonPin);
    dataLogger.setSamplingFeatureUUID(samplingFeature);

    // Begin the logger
    // Note:  Please change these battery voltages to match your battery
    // At lowest battery level, skip sensor set-up
    if (getBatteryVoltage()< battery_minimum)    // minimum allowable battery
    {
        dataLogger.begin(); // this originally said datalogger.begin(true) but I deleted
                            // the 'true' because that caused an error 2/7/19 rm
    }
    else  // set up file and sensors
    {
        dataLogger.begin();
    }

    // Call the processor sleep
    dataLogger.systemSleep();
}


// ==========================================================================
// Main loop function
// ==========================================================================

// Use this short loop for simple data logging and sending
void loop()
{
    // Note:  Please change these battery voltages to match your battery
    // At very low battery, just go back to sleep
   
       if (getBatteryVoltage() < battery_minimum) 
    {
        dataLogger.systemSleep();
    }
    // If the battery is OK, log data
    else dataLogger.logData();
}
