      
/*****************************************************************************

Modified from:

menu_a_la_carte.ino
Written By:  Sara Damiano (sdamiano@stroudcenter.org)
Development Environment: PlatformIO
Hardware Platform: EnviroDIY Mayfly Arduino Datalogger
Software License: BSD-3.
  Copyright (c) 2017, Stroud Water Research Center (SWRC)
  and the EnviroDIY Development Team
This example sketch is written for ModularSensors library version 0.23.2
This shows most of the standard functions of the library at once.
DISCLAIMER:
THIS CODE IS PROVIDED "AS IS" - NO WARRANTY IS GIVEN.

Modified by Rachel Murray on 5/08/2019

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
const char *libraryVersion = "0.23.2";
// The name of this file
const char *sketchName = "yosemitech_4000_with_4g_bee.ino";
// Logger ID, also becomes the prefix for the name of the data file on SD card
const char *LoggerID = "Uowtest(UOW_test_1)";
// How frequently (in minutes) to log data
const uint8_t loggingInterval = 1; // set to 15 for testing -rm
// Your logger's timezone.
const int8_t timeZone = 12;  // Set to Auckland -rm
// NOTE:  Daylight savings time will not be applied!  Please use standard time!
float minimum_bat_voltage = 2; // Don't forget to change this if you are running under USB power--pj
float moderate_bat_voltage = 3; // setting 'moderate' voltage to save power when 
                                  // sending data over modem - rm

//=============================================================================
// Wifi SETTINGS // moved this to the top -rm
//======================================================================
const char *apn = "-";  // The APN for the gprs connection, unnecessary for WiFi
//const char *wifiId = "xxxxx";  // The WiFi access point, unnecessary for gprs
//const char *wifiPwd = "xxxxx";  // The password for connecting to WiFi, unnecessary for gprs

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
const int8_t sdCardPwrPin = -1;    // MCU SD card power pin (-1 if not applicable)
const int8_t sdCardSSPin = 12;     // MCU SD card chip select/slave select pin (must be given!)
const int8_t sensorPowerPin = 22;  // MCU pin controlling main sensor power (-1 if not applicable)

// Create the main processor chip "sensor" - for general metadata
const char *mcuBoardVersion = "v0.5b";
ProcessorStats mcuBoard(mcuBoardVersion);



// ==========================================================================
//    Settings for Additional Serial Ports
// ==========================================================================

// In some cases (ie, modbus communication) many sensors can share
// the same serial port.

// removed unnecessary lines -rm

// AltSoftSerial by Paul Stoffregen (https://github.com/PaulStoffregen/AltSoftSerial)
// is the most accurate software serial port for AVR boards.
// AltSoftSerial can only be used on one set of pins on each board so only one
// AltSoftSerial port can be used.
// Not all AVR boards are supported by AltSoftSerial.

#include <AltSoftSerial.h>
AltSoftSerial altSoftSerial;


// The "standard" software serial library uses interrupts that conflict
// with several other libraries used within this program, we must use a
// version of software serial that has been stripped of interrupts.
// NOTE:  Only use if necessary.  This is not a very accurate serial port!

const int8_t softSerialRx = 10;     // data in pin // changed from A3 -rm
const int8_t softSerialTx = 11;     // data out pin // changed from A4 -rm

#include <SoftwareSerial_ExtInts.h>  // for the stream communication
SoftwareSerial_ExtInts softSerial1(softSerialRx, softSerialTx);

// ==========================================================================
//    Wifi/Cellular Modem 
// ==========================================================================

// Create a reference to the serial port for the modem
// Extra hardware and software serial ports are created in the "Settings for Additional Serial Ports" section
HardwareSerial &modemSerial = Serial1;  // Use hardware serial if possible
// AltSoftSerial &modemSerial = altSoftSerial;  // For software serial if needed
// NeoSWSerial &modemSerial = neoSSerial1;  // For software serial if needed


// Modem Pins - Describe the physical pin connection of your modem to your board
const int8_t modemVccPin = -1;      // MCU pin controlling modem power (-1 if not applicable) //changed from -2 -rm
const int8_t modemStatusPin = 19;   // MCU pin used to read modem status (-1 if not applicable)
const int8_t modemResetPin = -1;    // MCU pin connected to modem reset pin (-1 if unconnected)//was 20 
const int8_t modemSleepRqPin = 23;  // MCU pin used for modem sleep/wake request (-1 if not applicable)was 23
const int8_t modemLEDPin = redLED;  // MCU pin connected an LED to show modem status (-1 if unconnected)




// For the u-blox SARA R410M based Digi LTE-M XBee3
// NOTE:  According to the manual, this should be less stable than transparent
// mode, but my experience is the complete reverse.
//#include <modems/DigiXBeeLTEBypass.h>
//const long modemBaud = 9600;  // All XBee's use 9600 by default
//const bool useCTSforStatus = true;   // Flag to use the modem CTS pin for status
//DigiXBeeLTEBypass modemXBLTEB(&modemSerial,
//                              modemVccPin, modemStatusPin, useCTSforStatus,
//                              modemResetPin, modemSleepRqPin,
//                              apn);
// Create an extra reference to the modem by a generic name (not necessary)
//DigiXBeeLTEBypass modem = modemXBLTEB;


// ==========================================================================

// // For any Digi Cellular XBee's
// // NOTE:  The u-blox based Digi XBee's (3G global and LTE-M global)
// // are more stable used in bypass mode (below)
//  The Telit based Digi XBees (LTE Cat1) can only use this mode.
 #include <modems/DigiXBeeCellularTransparent.h>
 const long modemBaud = 9600;  // All XBee's use 9600 by default
 const bool useCTSforStatus = true;   // Flag to use the modem CTS pin for status
DigiXBeeCellularTransparent modemXBCT(&modemSerial,
                                      modemVccPin, modemStatusPin, useCTSforStatus,
                                     modemResetPin, modemSleepRqPin,
                                   apn);
// // Create an extra reference to the modem by a generic name (not necessary)
 DigiXBeeCellularTransparent modem = modemXBCT;

 // ==========================================================================

// // For the u-blox SARA U201 based Digi 3G XBee with 2G fallback
// // NOTE:  According to the manual, this should be less stable than transparent
// // mode, but my experience is the complete reverse.
// #include <modems/DigiXBee3GBypass.h>
//const long modemBaud = 9600;  // All XBee's use 9600 by default
//const bool useCTSforStatus = true;   // had to add this flag in order to compile -rm
// DigiXBee3GBypass modemXB3GB(&modemSerial, // had to change this from 'DigiXBeeLTEBypass' -rm
//                            modemVccPin, modemStatusPin, useCTSforStatus,
//                             modemResetPin, modemSleepRqPin,
//                             apn);
// // Create an extra reference to the modem by a generic name (not necessary)
//DigiXBee3GBypass modem = modemXB3GB;


// ==========================================================================
//    Maxim DS3231 RTC (Real Time Clock)
// ==========================================================================
#include <sensors/MaximDS3231.h>

// Create a DS3231 sensor object
MaximDS3231 ds3231(1);

// Create a temperature variable pointer for the DS3231
 Variable *ds3231Temp = new MaximDS3231_Temp(&ds3231, "2a54745f-73bc-4843-a29c-37774c9f4118");


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

// Create conductivity, temperature, and depth variable pointers for the CTD
Variable *ctdCond = new DecagonCTD_Cond(&ctd, "6b3ef348-eb2d-4c94-8129-ee4559bb3217");
Variable *ctdTemp = new DecagonCTD_Temp(&ctd, "52f50dd5-b6b6-4975-ad61-55c5f864fd3e");
Variable *ctdDepth = new DecagonCTD_Depth(&ctd, "d3eff5f5-92fe-4316-ae0f-75de31b0ce50");




// ==========================================================================
//    Yosemitech Y4000 Multiparameter Sonde (DOmgL, Turbidity, Cond, pH, Temp, ORP, Chlorophyll, BGA)
// ==========================================================================
#include <sensors/YosemitechY4000.h>

// Create a reference to the serial port for modbus
  AltSoftSerial &modbusSerial = altSoftSerial;  // For software serial if needed
//removed unnecessary lines -rm

byte y4000ModbusAddress = 0x01;  // The modbus address of the Y4000
 const int8_t rs485AdapterPower = sensorPowerPin;  // Pin to switch RS485 adapter power on and off (-1 if unconnected)
 const int8_t modbusSensorPower = A3;  // Pin to switch sensor power on and off (-1 if unconnected)
 const int8_t max485EnablePin = -1;  // Pin connected to the RE/DE on the 485 chip (-1 if unconnected)
const uint8_t y4000NumberReadings = 5;  // The manufacturer recommends averaging 10 readings, but we take 5 to minimize power consumption

// Create a Yosemitech Y4000 multi-parameter sensor object
YosemitechY4000 y4000(y4000ModbusAddress, modbusSerial, rs485AdapterPower, modbusSensorPower, max485EnablePin, y4000NumberReadings);

// Create all of the variable pointers for the Y4000
 Variable *y4000DO = new YosemitechY4000_DOmgL(&y4000, "48589f6a-046c-48f4-84ef-3eb25ad07702");
 Variable *y4000Turb = new YosemitechY4000_Turbidity(&y4000, "46daf79d-3cd6-43b2-b8b2-dbcdce1e4c1c");
 Variable *y4000Cond = new YosemitechY4000_Cond(&y4000, "2dab997c-0d38-4b0f-9116-43e27893feaf");
 Variable *y4000pH = new YosemitechY4000_pH(&y4000, "47924fae-46ad-4fc6-99c4-d5bd629f6747");
 Variable *y4000Temp = new YosemitechY4000_Temp(&y4000, "9ed4d2a1-492c-4ae4-bd69-22dcd429c526");
// Variable *y4000ORP = new YosemitechY4000_ORP(&y4000, "12345678-abcd-1234-ef00-1234567890ab"); // not currently using -rm
// Variable *y4000Chloro = new YosemitechY4000_Chlorophyll(&y4000, "12345678-abcd-1234-ef00-1234567890ab");
// Variable *y4000BGA = new YosemitechY4000_BGA(&y4000, "12345678-abcd-1234-ef00-1234567890ab");


// ==========================================================================
//    Creating the Variable Array[s] and Filling with Variable Objects
// ==========================================================================
#include <VariableArray.h>

// FORM1: Create pointers for all of the variables from the sensors,
// at the same time putting them into an array
// NOTE:  Forms one and two can be mixed
Variable *variableList[] = {
   
    new ProcessorStats_Battery(&mcuBoard, "8d2a269f-0bce-4faa-b79f-7a19695f5bd2"),
    new MaximDS3231_Temp(&ds3231, "c6bc39f6-04a1-4ba4-8f6d-10130ec291b6"),

    new YosemitechY4000_DOmgL(&y4000, "48589f6a-046c-48f4-84ef-3eb25ad07702"),
    new YosemitechY4000_Turbidity(&y4000, "46daf79d-3cd6-43b2-b8b2-dbcdce1e4c1c"),
    new YosemitechY4000_Cond(&y4000, "2dab997c-0d38-4b0f-9116-43e27893feaf"),
    new YosemitechY4000_pH(&y4000, "47924fae-46ad-4fc6-99c4-d5bd629f6747"),
    new YosemitechY4000_Temp(&y4000, "9ed4d2a1-492c-4ae4-bd69-22dcd429c526"),
    new DecagonCTD_Cond(&ctd, "6b3ef348-eb2d-4c94-8129-ee4559bb3217"), 
    new DecagonCTD_Temp(&ctd, "52f50dd5-b6b6-4975-ad61-55c5f864fd3e"),
    new DecagonCTD_Depth(&ctd, "d3eff5f5-92fe-4316-ae0f-75de31b0ce50"),

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
Logger dataLogger(LoggerID, loggingInterval, &varArray);


// ==========================================================================
//    A Publisher to Monitor My Watershed / EnviroDIY Data Sharing Portal
// ==========================================================================
// Device registration and sampling feature information can be obtained after
// registration at https://monitormywatershed.org or https://data.envirodiy.org
const char *registrationToken = "e2489981-e06d-471d-8b0b-74d262612fc2";   // Device registration token
const char *samplingFeature = "e2231892-a21b-48ab-a93c-2df3badd4bea";     // Sampling feature UUID

// Create a data publisher for the EnviroDIY/WikiWatershed POST endpoint
#include <publishers/EnviroDIYPublisher.h>
EnviroDIYPublisher EnviroDIYPOST(dataLogger, &modem.gsmClient, registrationToken, samplingFeature);




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

    // Allow interrupts for software serial
    #if defined SoftwareSerial_ExtInts_h
        enableInterrupt(softSerialRx, SoftwareSerial_ExtInts::handle_interrupt, CHANGE);
    #endif
    #if defined NeoSWSerial_h
        enableInterrupt(neoSSerial1Rx, neoSSerial1ISR, CHANGE);
    #endif

    // Start the serial connection with the modem
    modemSerial.begin(modemBaud);

    // Start the stream for the modbus sensors; all currently supported modbus sensors use 9600 baud
    modbusSerial.begin(9600);


    // Assign pins SERCOM functionality for SAMD boards
    // NOTE:  This must happen *after* the various serial.begin statements
    #if defined ARDUINO_ARCH_SAMD
    #ifndef ENABLE_SERIAL2
    pinPeripheral(10, PIO_SERCOM);  // Serial2 Tx/Dout = SERCOM1 Pad #2
    pinPeripheral(11, PIO_SERCOM);  // Serial2 Rx/Din = SERCOM1 Pad #0
    #endif
    #ifndef ENABLE_SERIAL3
    pinPeripheral(2, PIO_SERCOM);  // Serial3 Tx/Dout = SERCOM2 Pad #2
    pinPeripheral(5, PIO_SERCOM);  // Serial3 Rx/Din = SERCOM2 Pad #3
    #endif
    #endif

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
    if (modemVccPin >= 0)
    {
        pinMode(modemVccPin, OUTPUT);
        digitalWrite(modemVccPin, LOW);
    }
    if (sensorPowerPin >= 0)
    {
        pinMode(sensorPowerPin, OUTPUT);
        digitalWrite(sensorPowerPin, LOW);
    }

    // Set the timezones for the logger/data and the RTC
    // Logging in the given time zone
    Logger::setLoggerTimeZone(timeZone);
    // It is STRONGLY RECOMMENDED that you set the RTC to be in UTC (UTC+0)
    Logger::setRTCTimeZone(0);

    // Attach the modem and information pins to the logger
    dataLogger.attachModem(modem);
    modem.setModemLED(modemLEDPin);
    dataLogger.setLoggerPins(wakePin, sdCardSSPin, sdCardPwrPin, buttonPin, greenLED);

    // Begin the logger
    dataLogger.begin();

    // Note:  Please change these battery voltages to match your battery
    // Check that the battery is OK before powering the modem
    if (getBatteryVoltage() > minimum_bat_voltage)
    {
        modem.modemPowerUp();
        modem.wake();
        modem.setup();

        // At very good battery voltage, or with suspicious time stamp, sync the clock
        // Note:  Please change these battery voltages to match your battery
        if (getBatteryVoltage() > moderate_bat_voltage ||
            dataLogger.getNowEpoch() < 1546300800 ||  /*Before 01/01/2019*/
            dataLogger.getNowEpoch() > 1735689600)  /*After 1/1/2025*/
        {
            // Synchronize the RTC with NIST
            Serial.println(F("Attempting to connect to the internet and synchronize RTC with NIST"));
            if (modem.connectInternet(120000L))
            {
                dataLogger.setRTClock(modem.getNISTTime());
            }
        }
    }

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

    // Power down the modem - but only if there will be more than 15 seconds before
    // the first logging interval - it can take the LTE modem that long to shut down
    if (Logger::getNowEpoch() % (loggingInterval*60) > 15 ||
        Logger::getNowEpoch() % (loggingInterval*60) < 6)
    {
        Serial.println(F("Putting modem to sleep"));
        modem.modemSleepPowerDown();
    }
    else
    {
        Serial.println(F("Leaving modem on until after first measurement"));
    }

    // Call the processor sleep
    Serial.println(F("Putting processor to sleep"));
    dataLogger.systemSleep();
}


// ==========================================================================
// Main loop function
// ==========================================================================

// Use this short loop for simple data logging and sending
// /*
void loop()
{
    
    // Note:  Please change these battery voltages to match your battery
    // At very low battery, just go back to sleep
    if (getBatteryVoltage() < minimum_bat_voltage)
    {
        dataLogger.systemSleep();
    }
    // At moderate voltage, log data but don't send it over the modem
    else if (getBatteryVoltage() < moderate_bat_voltage)
    {
        dataLogger.logData();
    }
    // If the battery is good, send the data to the world
    else
    {
       Serial.println(getBatteryVoltage()); //what is our returned voltage value?--pj
      
        dataLogger.logDataAndPublish();
    }
}
