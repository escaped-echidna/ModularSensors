/*****************************************************************************
cuahsi_workshop.ino
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
//    Defines for the Arduino IDE
//    In PlatformIO, set these build flags in your platformio.ini
// ==========================================================================
 //#ifndef SDI12_EXTERNAL_PCINT
 //#define SDI12_EXTERNAL_PCINT
 //#endif
 //#ifndef TINY_GSM_RX_BUFFER
 //#define TINY_GSM_RX_BUFFER 512
 //#endif
 //#ifndef TINY_GSM_YIELD_MS
 //#define TINY_GSM_YIELD_MS 2
 //#endif
 //#ifndef MS_ESPRESSIFESP8266_DEBUG_DEEP
 //#define MS_ESPRESSIFESP8266_DEBUG_DEEP
 //#endif


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
const char *libraryVersion = "0.23.10";
// The name of this file
const char *sketchName = "logging_to_MMW_test.ino";
// Logger ID, also becomes the prefix for the name of the data file on SD card
const char *LoggerID = "IOWtest";
// How frequently (in minutes) to log data
const uint8_t loggingInterval = 15; // set to 15 minutes -rm
// Your logger's timezone.
const int8_t timeZone = 12;  // Eastern Standard Time
// NOTE:  Daylight savings time will not be applied!  Please use standard time!
float minimum_bat_voltage = 11.0; // defining minimum and moderate battery voltages
float moderate_bat_voltage = 11.8; // -rm

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
//    Wifi/Cellular Modem Settings
// ==========================================================================

// Create a reference to the serial port for the modem
// Extra hardware and software serial ports are created in the "Settings for Additional Serial Ports" section
HardwareSerial &modemSerial = Serial1;  // Use hardware serial if possible
// AltSoftSerial &modemSerial = altSoftSerial;  // For software serial if needed
// NeoSWSerial &modemSerial = neoSSerial1;  // For software serial if needed


// Modem Pins - Describe the physical pin connection of your modem to your board// DFRobot ESP8266 Bee with Mayfly
const int8_t modemVccPin = -1;       // MCU pin controlling modem power (-1 if not applicable)
const int8_t modemStatusPin = -1;    // MCU pin used to read modem status (-1 if not applicable)
const int8_t modemResetPin = -1;     // MCU pin connected to modem reset pin (-1 if unconnected)
const int8_t modemSleepRqPin = -1;   // MCU pin used for wake from light sleep (-1 if not applicable)
const int8_t modemLEDPin = redLED;   // MCU pin connected an LED to show modem status (-1 if unconnected)

// Network connection information
const char *wifiId = "HUAWEI-2A09";  // The WiFi access point, unnecessary for gprs
const char *wifiPwd = "R09168991R4";  // The password for connecting to WiFi, unnecessary for gprs


// ==========================================================================
//    The modem object
//    Note:  Don't use more than one!
// ==========================================================================

// For almost anything based on the Espressif ESP8266 using the AT command firmware
#include <modems/EspressifESP8266.h>
const long modemBaud = 9600;  // Communication speed of the modem
// NOTE:  This baud rate too fast for an 8MHz board, like the Mayfly!  The module
// should be programmed to a slower baud rate or set to auto-baud using the
// AT+UART_CUR or AT+UART_DEF command *before* attempting control with this library.
// Pins for light sleep on the ESP8266.
// For power savings, I recommend NOT using these if it's possible to use deep sleep.
const int8_t espSleepRqPin = -1;  // GPIO# ON THE ESP8266 to assign for light sleep request (-1 if not applicable)
const int8_t espStatusPin = -1;  // GPIO# ON THE ESP8266 to assign for light sleep status (-1 if not applicable)
EspressifESP8266 modemESP(&modemSerial,
                          modemVccPin, modemStatusPin,
                          modemResetPin, modemSleepRqPin,
                          wifiId, wifiPwd,
                          1,  // measurements to average, optional
                          espSleepRqPin, espStatusPin  // Optional arguments
                         );
// Create an extra reference to the modem by a generic name (not necessary)
EspressifESP8266 modem = modemESP;


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


// ==================================================================
 // Analog battery voltage reading -rm
// ==================================================================
// mooched from Conrad's pump controller script

float battery_multiplier = 1;

float getBatteryVoltage(void)
{
    // read the input on analog pin 3: // checking voltage using analog input -rm
    int sensorValue = 0 ;
    float battery_voltage = 0 ;

    sensorValue = analogRead(A3);
    // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
    battery_voltage = sensorValue * battery_multiplier; //

    return battery_voltage;
}

// Properties of the calculated variable
const uint8_t calculatedVarResolution = 2;  // The number of digits after the decimal place
const char *calculatedVarName = "batteryVoltage";  // This must be a value from http://vocabulary.odm2.org/variablename/
const char *calculatedVarUnit = "Volt";  // This must be a value from http://vocabulary.odm2.org/units/
const char *calculatedVarCode = "battVolt";  // A short code for the variable
const char *calculatedVarUUID = "8952f24c-ea91-47d8-a555-ea54f4f7b876";  // The (optional) universallly unique identifier

// Finally, Create a calculated variable pointer and return a variable pointer to it
Variable *batteryVoltageVariable = new Variable(getBatteryVoltage, calculatedVarResolution,
                                       calculatedVarName, calculatedVarUnit,
                                       calculatedVarCode, calculatedVarUUID);


// ==========================================================================
//    Yosemitech Y4000 Multiparameter Sonde (DOmgL, Turbidity, Cond, pH, Temp, ORP, Chlorophyll, BGA)
// ==========================================================================

//#include <AltSoftSerial.h>
//AltSoftSerial altSoftSerial;
//#include <sensors/YosemitechY4000.h>

// Create a reference to the serial port for modbus
// Extra hardware and software serial ports are created in the "Settings for Additional Serial Ports" section
// #if defined ARDUINO_ARCH_SAMD || defined ATMEGA2560
// HardwareSerial &modbusSerial = Serial2;  // Use hardware serial if possible
// #else
// AltSoftSerial &modbusSerial = altSoftSerial;  // For software serial if needed
// // NeoSWSerial &modbusSerial = neoSSerial1;  // For software serial if needed
// #endif

// byte y4000ModbusAddress = 0x01;  // The modbus address of the Y4000
// const int8_t rs485AdapterPower = sensorPowerPin;  // Pin to switch RS485 adapter power on and off (-1 if unconnected)
// const int8_t modbusSensorPower = A3;  // Pin to switch sensor power on and off (-1 if unconnected)
// const int8_t max485EnablePin = -1;  // Pin connected to the RE/DE on the 485 chip (-1 if unconnected)
// const uint8_t y4000NumberReadings = 5;  // The manufacturer recommends averaging 10 readings, but we take 5 to minimize power consumption

// Create a Yosemitech Y4000 multi-parameter sensor object
//YosemitechY4000 y4000(y4000ModbusAddress, modbusSerial, rs485AdapterPower, modbusSensorPower, max485EnablePin, y4000NumberReadings);

// Create all of the variable pointers for the Y4000

 //Variable *y4000DO = new YosemitechY4000_DOmgL(&y4000, "48589f6a-046c-48f4-84ef-3eb25ad07702")
// Variable *y4000Turb = new YosemitechY4000_Turbidity(&y4000, "46daf79d-3cd6-43b2-b8b2-dbcdce1e4c1c"),
// Variable *y4000Cond = new YosemitechY4000_Cond(&y4000, "2dab997c-0d38-4b0f-9116-43e27893feaf"),
// Variable *y4000pH = new YosemitechY4000_pH(&y4000, "47924fae-46ad-4fc6-99c4-d5bd629f6747"),
// Variable *y4000Temp = new YosemitechY4000_Temp(&y4000, "9ed4d2a1-492c-4ae4-bd69-22dcd429c526"),
// Variable *y4000ORP = new YosemitechY4000_ORP(&y4000, "12345678-abcd-1234-ef00-1234567890ab");
// Variable *y4000Chloro = new YosemitechY4000_Chlorophyll(&y4000, "12345678-abcd-1234-ef00-1234567890ab");
// Variable *y4000BGA = new YosemitechY4000_BGA(&y4000, "12345678-abcd-1234-ef00-1234567890ab");

// ==========================================================================
//    Creating the Variable Array[s] and Filling with Variable Objects
// ==========================================================================

// FORM1: Create pointers for all of the variables from the sensors,
// at the same time putting them into an array
// NOTE:  Forms one and two can be mixed
Variable *variableList[] = {
 //   new YosemitechY4000_DOmgL(&y4000, "48589f6a-046c-48f4-84ef-3eb25ad07702"),
 //   new YosemitechY4000_Turbidity(&y4000, "46daf79d-3cd6-43b2-b8b2-dbcdce1e4c1c"),
 //   new YosemitechY4000_Cond(&y4000, "2dab997c-0d38-4b0f-9116-43e27893feaf"),
 //   new YosemitechY4000_pH(&y4000, "47924fae-46ad-4fc6-99c4-d5bd629f6747"),
 //   new YosemitechY4000_Temp(&y4000, "9ed4d2a1-492c-4ae4-bd69-22dcd429c526"),
    new DecagonCTD_Cond(&ctd, "6b3ef348-eb2d-4c94-8129-ee4559bb3217"), 
    new DecagonCTD_Temp(&ctd, "52f50dd5-b6b6-4975-ad61-55c5f864fd3e"),
    new DecagonCTD_Depth(&ctd, "d3eff5f5-92fe-4316-ae0f-75de31b0ce50"),
 //   new ProcessorStats_Battery(&mcuBoard, "8d2a269f-0bce-4faa-b79f-7a19695f5bd2"),
    new MaximDS3231_Temp(&ds3231, "c6bc39f6-04a1-4ba4-8f6d-10130ec291b6"),
    batteryVoltageVariable,

};

// Count up the number of pointers in the array
int variableCount = sizeof(variableList) / sizeof(variableList[0]);

// Create the VariableArray object
VariableArray varArray(variableCount, variableList);


// ==========================================================================
//     The Logger Object[s]
// ==========================================================================

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

    // Allow interrupts for software serial
    #if defined SoftwareSerial_ExtInts_h
        enableInterrupt(softSerialRx, SoftwareSerial_ExtInts::handle_interrupt, CHANGE);
    #endif

    // Start the serial connection with the modem
    modemSerial.begin(modemBaud);


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

   
    Serial.print ("\t");
    Serial.print ("current voltage = ");
    Serial.println (getBatteryVoltage()); // check battery voltage -rm
    Serial.print ("\t");

    // Note:  Please change these battery voltages to match your battery
    // Check that the battery is OK before powering the modem
    if (getBatteryVoltage() > minimum_bat_voltage)
    {
        modem.modemPowerUp();
        modem.wake();

        #if F_CPU == 8000000L
        if (modemBaud > 57600)
        {
            modemSerial.begin(115200);
            modem.gsmModem.sendAT(GF("+UART_DEF=9600,8,1,0,0"));
            modem.gsmModem.waitResponse();
            modemSerial.end();
            modemSerial.begin(9600);
        }
        #endif

        modem.setup();

        // At very good battery voltage, or with suspicious time stamp, sync the clock
        // Note:  Please change these battery voltages to match your battery
        if (getBatteryVoltage() > moderate_bat_voltage || //changed to use new voltage input on analog 2 -rm
            dataLogger.getNowEpoch() < 1546300800 ||  /*Before 01/01/2019*/
            dataLogger.getNowEpoch() > 1735689600)  /*After 1/1/2025*/
        {
            // Synchronize the RTC with NIST
            Serial.println(F("Attempting to connect to the internet and synchronize RTC with NIST"));
            if (modem.connectInternet(120000L))
            {
                dataLogger.setRTClock(modem.getNISTTime());
            }
            else
            {
                Serial.println(F("Could not connect to internet for clock sync."));
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
        modem.disconnectInternet();
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


Serial.print ("\t");
Serial.print ("current voltage = ");
Serial.println (getBatteryVoltage());

Serial.print ("\t");

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
        dataLogger.logDataAndPublish();
    }
}