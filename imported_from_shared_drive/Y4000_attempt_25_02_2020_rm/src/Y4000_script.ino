/*****************************************************************************
Modified from menu_a_la_carte.ino
Written By:  Sara Damiano (sdamiano@stroudcenter.org)
Development Environment: PlatformIO
Hardware Platform: EnviroDIY Mayfly Arduino Datalogger
Software License: BSD-3.
  Copyright (c) 2017, Stroud Water Research Center (SWRC)
  and the EnviroDIY Development Team
This example sketch is written for ModularSensors library version 0.23.16
This shows most of the standard functions of the library at once.
DISCLAIMER:
THIS CODE IS PROVIDED "AS IS" - NO WARRANTY IS GIVEN.

Attempt to log using Y4000 sonde, 
connecting to Modbus via NeoSoftSerial --rm

*****************************************************************************/
// ---------------------------------------------------------------------------
// Include the base required libraries
// ---------------------------------------------------------------------------

#include <Arduino.h>
//#include <SoftwareSerial.h>
//#include <AltSoftSerial.h>
#include <YosemitechModbus.h>
#include <EnableInterrupt.h>  // for external and pin change interrupts
#include <LoggerBase.h>  // The modular sensors library


// ==========================================================================
//    Data Logger Settings
// ==========================================================================
// The library version this example was written for
const char *libraryVersion = "0.23.16";
// The name of this file
const char *sketchName = "Y4000_script.ino";
// Logger ID, also becomes the prefix for the name of the data file on SD card
const char *LoggerID = "IOWtest";
// How frequently (in minutes) to log data
const uint8_t loggingInterval = 5;
// Your logger's timezone.
const int8_t timeZone = 12;  // NZ Standard Time
// NOTE:  Daylight savings time will not be applied!  Please use standard time!

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
float minimum_bat_voltage = 11; // defining minimum and moderate battery voltages, set to 1 for test
float moderate_bat_voltage = 11.5; // -rm


// Create the main processor chip "sensor" - for general metadata
const char *mcuBoardVersion = "v0.5b";
ProcessorStats mcuBoard(mcuBoardVersion);

// Create battery voltage pointers for the processor
 Variable *mcuBoardBatt = new ProcessorStats_Battery(&mcuBoard, "12345678-abcd-1234-ef00-1234567890ab");


// NeoSWSerial (https://github.com/SRGDamia1/NeoSWSerial) is the best software
// serial that can be used on any pin supporting interrupts.
// You can use as many instances of NeoSWSerial as you want.
// Not all AVR boards are supported by NeoSWSerial.
#include <NeoSWSerial.h>  // for the stream communication
const int8_t neoSSerial1Rx = 6;     // data in pin
const int8_t neoSSerial1Tx = 5;     // data out pin
NeoSWSerial neoSSerial1(neoSSerial1Rx, neoSSerial1Tx);
// To use NeoSWSerial in this library, we define a function to receive data
// This is just a short-cut for later
void neoSSerial1ISR()
{
    NeoSWSerial::rxISR(*portInputRegister(digitalPinToPort(neoSSerial1Rx)));
}

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


// ==========================================================================
//    Maxim DS3231 RTC (Real Time Clock)
// ==========================================================================
#include <sensors/MaximDS3231.h>

// Create a DS3231 sensor object
MaximDS3231 ds3231(1);

// Create a temperature variable pointer for the DS3231
// Variable *ds3231Temp = new MaximDS3231_Temp(&ds3231, "12345678-abcd-1234-ef00-1234567890ab");

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

 // =================================================================
// Turbidity correction formula
//==========================================

// correction for raw voltage from turbidimeter using formula
// of the form ax^2 + bx + c

float turbidity_correction_A = 0.0;
float turbidity_correction_B = 1.0; 
float turbidity_correction_C = 0.0;


 //=============================================================
// Turbidity sensor as a calculated variable
//===========================================================
#include <Adafruit_ADS1015.h>

uint8_t i2cAddressTurbidity = 0x48;
uint8_t measurementsToAverage = 7;
uint8_t ADSChannelTurbidity=1;

float calculateTurbidity(void)
{
    Adafruit_ADS1115 ads(i2cAddressTurbidity); // Use this for 16-bit version
   // Adafruit_ADS1015 ads(i2cAddressTurbidity);  // Use this for the 12-bit version

    // place to put the results...
    float calculatedResult = -9999;  // Always safest to start with a bad value
    float sensorValue = -9999; // Always safest to start with a bad value
    float sum_measurements = 0;

    ads.setGain(GAIN_ONE);
        // Begin ADC
    ads.begin();
    
    for (size_t i = 0; i < measurementsToAverage; i++)
    {
        sensorValue = ads.readADC_SingleEnded_V(ADSChannelTurbidity);  // Getting the reading
        sum_measurements = sum_measurements + sensorValue;
       
    }

    //converting to voltage...
    
    float voltage = (sum_measurements/measurementsToAverage);

   // Serial.print(F(" Calculating Turbidity... "));
    
    // Reading analog 1 pin to get raw value
    //int sensorValue = analogRead(A2);
   // Serial.print(F(" Sensor value is ... "));
   // Serial.println(sensorValue);

   // Serial.print(F(" Voltage is ... "));
   // Serial.println(voltage);    

    // adjusting turbidity using formula ax^2 + bx + c

    float Ta = turbidity_correction_A;
    float Tb = turbidity_correction_B;
    float Tc = turbidity_correction_C;

     if (sensorValue != -9999)  // make sure input is good
     {
         calculatedResult = Ta*(voltage*voltage) + Tb*voltage + Tc;
           // Serial.print(F(" Calculated result is ... "));
           // Serial.println(calculatedResult);
     }
    return calculatedResult;
}

// Properties of the calculated variable
const uint8_t calculatedVarResolution = 2;  // The number of digits after the decimal place
const char *calculatedVarName = "Turbidity Voltage, uncorrected";  // This must be a value from http://vocabulary.odm2.org/variablename/
const char *calculatedVarUnit = "volt";  // This must be a value from http://vocabulary.odm2.org/units/
const char *calculatedVarCode = "Turb";  // A short code for the variable
const char *calculatedVarUUID = "12345678-abcd-1234-ef00-1234567890ab";  // The (optional) universallly unique identifier

// Finally, Create a calculated variable pointer and return a variable pointer to it
Variable *TurbidityVoltage = new Variable(calculateTurbidity, calculatedVarResolution,
                                       calculatedVarName, calculatedVarUnit,
                                       calculatedVarCode, calculatedVarUUID);

// ==========================================================================
//    Yosemitech Y4000 Multiparameter Sonde (DOmgL, Turbidity, Cond, pH, Temp, ORP, Chlorophyll, BGA)
// ==========================================================================
#include <sensors/YosemitechY4000.h>

// Create a reference to the serial port for modbus
// Extra hardware and software serial ports are created in the "Settings for Additional Serial Ports" section
// #if defined ARDUINO_ARCH_SAMD || defined ATMEGA2560
// HardwareSerial &modbusSerial = Serial2;  // Use hardware serial if possible
// #else
// AltSoftSerial &modbusSerial = altSoftSerial;  // For software serial if needed
 NeoSWSerial &modbusSerial = neoSSerial1;  // For software serial if needed
// #endif

byte y4000ModbusAddress = 0x01;  // The modbus address of the Y4000
const int DEREPin = -1;   // The pin controlling Recieve Enable and Driver Enable
                          // on the RS485 adapter, if applicable (else, -1)
                          // Setting HIGH enables the driver (arduino) to send text
                          // Setting LOW enables the receiver (sensor) to send text
                       
const int8_t rs485AdapterPower = -1;  // Pin to switch RS485 adapter power on and off (-1 if unconnected) const int8_t modbusSensorPower = -1;  // Pin to switch sensor power on and off (-1 if unconnected)
const int8_t modbusSensorPower = 22;  // Pin to switch sensor power on and off (-1 if unconnected)
const int8_t max485EnablePin = -1;  // Pin connected to the RE/DE on the 485 chip (-1 if unconnected)
const uint8_t y4000NumberReadings = 5;  // The manufacturer recommends averaging 10 readings, but we take 5 to minimize power consumption

// Create a Yosemitech Y4000 multi-parameter sensor object
YosemitechY4000 y4000(y4000ModbusAddress, modbusSerial, rs485AdapterPower, modbusSensorPower, max485EnablePin, y4000NumberReadings);


 // ==========================================================================
//    Creating the Variable Array[s] and Filling with Variable Objects
// ==========================================================================

// FORM1: Create pointers for all of the variables from the sensors,
// at the same time putting them into an array
// NOTE:  Forms one and two can be mixed
Variable *variableList[] = {
 
    new MaximDS3231_Temp(&ds3231, "c6bc39f6-04a1-4ba4-8f6d-10130ec291b6"),    
    //new ProcessorStats_Battery(&mcuBoard, "12345678-abcd-1234-ef00-1234567890ab"),
    new YosemitechY4000_DOmgL(&y4000, "48589f6a-046c-48f4-84ef-3eb25ad07702"),
    new YosemitechY4000_Turbidity(&y4000, "46daf79d-3cd6-43b2-b8b2-dbcdce1e4c1c"),
    new YosemitechY4000_Cond(&y4000, "2dab997c-0d38-4b0f-9116-43e27893feaf"),
    new YosemitechY4000_pH(&y4000, "47924fae-46ad-4fc6-99c4-d5bd629f6747"),
    new YosemitechY4000_Temp(&y4000, "9ed4d2a1-492c-4ae4-bd69-22dcd429c526"),
   // new YosemitechY4000_ORP(&y4000, "12345678-abcd-1234-ef00-1234567890ab"),
   // new YosemitechY4000_Chlorophyll(&y4000, "12345678-abcd-1234-ef00-1234567890ab"),
   // new YosemitechY4000_BGA(&y4000, "12345678-abcd-1234-ef00-1234567890ab"),
    new ExternalVoltage_Volt(&extvolt, "8952f24c-ea91-47d8-a555-ea54f4f7b876"),
    TurbidityVoltage,
    
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

    #if defined NeoSWSerial_h
        enableInterrupt(neoSSerial1Rx, neoSSerial1ISR, CHANGE);
    #endif

    // Start the serial connection with the modem
    modemSerial.begin(modemBaud);

    // Start the stream for the modbus sensors; all currently supported modbus sensors use 9600 baud
    modbusSerial.begin(9600);
    
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

// Attach the modem and information pins to the logger

    dataLogger.attachModem(modem);
    modem.setModemLED(modemLEDPin);
    dataLogger.setLoggerPins(wakePin, sdCardSSPin, sdCardPwrPin, buttonPin, greenLED);

    // Begin the logger
    dataLogger.begin();

 
    Serial.print ("current voltage = ");
    Serial.println (getBatteryVoltage());
    Serial.print ("");

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