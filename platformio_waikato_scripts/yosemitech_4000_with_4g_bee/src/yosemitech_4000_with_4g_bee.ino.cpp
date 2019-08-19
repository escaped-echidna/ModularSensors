# 1 "c:\\users\\murrayr\\appdata\\local\\temp\\tmp2ihpgn"
#include <Arduino.h>
# 1 "C:/Users/murrayr/Documents/Arduino/libraries/ModularSensors/platformio_waikato_scripts/yosemitech_4000_with_4g_bee/src/yosemitech_4000_with_4g_bee.ino"
# 25 "C:/Users/murrayr/Documents/Arduino/libraries/ModularSensors/platformio_waikato_scripts/yosemitech_4000_with_4g_bee/src/yosemitech_4000_with_4g_bee.ino"
#include <Arduino.h>
#include <EnableInterrupt.h>






const char *libraryVersion = "0.23.2";

const char *sketchName = "yosemitech_4000_with_4g_bee.ino";

const char *LoggerID = "Uowtest(UOW_test_1)";

const uint8_t loggingInterval = 1;

const int8_t timeZone = 12;

float minimum_bat_voltage = 2;
float moderate_bat_voltage = 3;





const char *apn = "-";






#include <sensors/ProcessorStats.h>

const long serialBaud = 115200;
const int8_t greenLED = 8;
const int8_t redLED = 9;
const int8_t buttonPin = 21;
const int8_t wakePin = A7;


const int8_t sdCardPwrPin = -1;
const int8_t sdCardSSPin = 12;
const int8_t sensorPowerPin = 22;


const char *mcuBoardVersion = "v0.5b";
ProcessorStats mcuBoard(mcuBoardVersion);
# 91 "C:/Users/murrayr/Documents/Arduino/libraries/ModularSensors/platformio_waikato_scripts/yosemitech_4000_with_4g_bee/src/yosemitech_4000_with_4g_bee.ino"
#include <AltSoftSerial.h>
AltSoftSerial altSoftSerial;







const int8_t softSerialRx = 10;
const int8_t softSerialTx = 11;

#include <SoftwareSerial_ExtInts.h>
SoftwareSerial_ExtInts softSerial1(softSerialRx, softSerialTx);







HardwareSerial &modemSerial = Serial1;





const int8_t modemVccPin = -1;
const int8_t modemStatusPin = 19;
const int8_t modemResetPin = -1;
const int8_t modemSleepRqPin = 23;
const int8_t modemLEDPin = redLED;
# 147 "C:/Users/murrayr/Documents/Arduino/libraries/ModularSensors/platformio_waikato_scripts/yosemitech_4000_with_4g_bee/src/yosemitech_4000_with_4g_bee.ino"
 #include <modems/DigiXBeeCellularTransparent.h>
 const long modemBaud = 9600;
 const bool useCTSforStatus = true;
DigiXBeeCellularTransparent modemXBCT(&modemSerial,
                                      modemVccPin, modemStatusPin, useCTSforStatus,
                                     modemResetPin, modemSleepRqPin,
                                   apn);

 DigiXBeeCellularTransparent modem = modemXBCT;
# 176 "C:/Users/murrayr/Documents/Arduino/libraries/ModularSensors/platformio_waikato_scripts/yosemitech_4000_with_4g_bee/src/yosemitech_4000_with_4g_bee.ino"
#include <sensors/MaximDS3231.h>


MaximDS3231 ds3231(1);


 Variable *ds3231Temp = new MaximDS3231_Temp(&ds3231, "2a54745f-73bc-4843-a29c-37774c9f4118");





 #include <sensors/DecagonCTD.h>

const char *CTDSDI12address = "1";
const uint8_t CTDNumberReadings = 6;
const int8_t SDI12Power = sensorPowerPin;
const int8_t SDI12Data = 7;


DecagonCTD ctd(*CTDSDI12address, SDI12Power, SDI12Data, CTDNumberReadings);


Variable *ctdCond = new DecagonCTD_Cond(&ctd, "6b3ef348-eb2d-4c94-8129-ee4559bb3217");
Variable *ctdTemp = new DecagonCTD_Temp(&ctd, "52f50dd5-b6b6-4975-ad61-55c5f864fd3e");
Variable *ctdDepth = new DecagonCTD_Depth(&ctd, "d3eff5f5-92fe-4316-ae0f-75de31b0ce50");







#include <sensors/YosemitechY4000.h>


  AltSoftSerial &modbusSerial = altSoftSerial;


byte y4000ModbusAddress = 0x01;
 const int8_t rs485AdapterPower = sensorPowerPin;
 const int8_t modbusSensorPower = A3;
 const int8_t max485EnablePin = -1;
const uint8_t y4000NumberReadings = 5;


YosemitechY4000 y4000(y4000ModbusAddress, modbusSerial, rs485AdapterPower, modbusSensorPower, max485EnablePin, y4000NumberReadings);


 Variable *y4000DO = new YosemitechY4000_DOmgL(&y4000, "48589f6a-046c-48f4-84ef-3eb25ad07702");
 Variable *y4000Turb = new YosemitechY4000_Turbidity(&y4000, "46daf79d-3cd6-43b2-b8b2-dbcdce1e4c1c");
 Variable *y4000Cond = new YosemitechY4000_Cond(&y4000, "2dab997c-0d38-4b0f-9116-43e27893feaf");
 Variable *y4000pH = new YosemitechY4000_pH(&y4000, "47924fae-46ad-4fc6-99c4-d5bd629f6747");
 Variable *y4000Temp = new YosemitechY4000_Temp(&y4000, "9ed4d2a1-492c-4ae4-bd69-22dcd429c526");
# 238 "C:/Users/murrayr/Documents/Arduino/libraries/ModularSensors/platformio_waikato_scripts/yosemitech_4000_with_4g_bee/src/yosemitech_4000_with_4g_bee.ino"
#include <VariableArray.h>




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



int variableCount = sizeof(variableList) / sizeof(variableList[0]);


VariableArray varArray(variableCount, variableList);





#include <LoggerBase.h>


Logger dataLogger(LoggerID, loggingInterval, &varArray);







const char *registrationToken = "e2489981-e06d-471d-8b0b-74d262612fc2";
const char *samplingFeature = "e2231892-a21b-48ab-a93c-2df3badd4bea";


#include <publishers/EnviroDIYPublisher.h>
EnviroDIYPublisher EnviroDIYPOST(dataLogger, &modem.gsmClient, registrationToken, samplingFeature);
# 296 "C:/Users/murrayr/Documents/Arduino/libraries/ModularSensors/platformio_waikato_scripts/yosemitech_4000_with_4g_bee/src/yosemitech_4000_with_4g_bee.ino"
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
float getBatteryVoltage();
void setup();
void loop();
#line 312 "C:/Users/murrayr/Documents/Arduino/libraries/ModularSensors/platformio_waikato_scripts/yosemitech_4000_with_4g_bee/src/yosemitech_4000_with_4g_bee.ino"
float getBatteryVoltage()
{
    if (mcuBoard.sensorValues[0] == -9999) mcuBoard.update();
    return mcuBoard.sensorValues[0];
}





void setup()
{



    #if defined SERIAL_PORT_USBVIRTUAL
      while (!SERIAL_PORT_USBVIRTUAL && (millis() < 10000)){}
    #endif


    Serial.begin(serialBaud);


    Serial.print(F("Now running "));
    Serial.print(sketchName);
    Serial.print(F(" on Logger "));
    Serial.println(LoggerID);
    Serial.println();

    Serial.print(F("Using ModularSensors Library version "));
    Serial.println(MODULAR_SENSORS_VERSION);

    if (String(MODULAR_SENSORS_VERSION) != String(libraryVersion))
        Serial.println(F(
            "WARNING: THIS EXAMPLE WAS WRITTEN FOR A DIFFERENT VERSION OF MODULAR SENSORS!!"));


    #if defined SoftwareSerial_ExtInts_h
        enableInterrupt(softSerialRx, SoftwareSerial_ExtInts::handle_interrupt, CHANGE);
    #endif
    #if defined NeoSWSerial_h
        enableInterrupt(neoSSerial1Rx, neoSSerial1ISR, CHANGE);
    #endif


    modemSerial.begin(modemBaud);


    modbusSerial.begin(9600);




    #if defined ARDUINO_ARCH_SAMD
    #ifndef ENABLE_SERIAL2
    pinPeripheral(10, PIO_SERCOM);
    pinPeripheral(11, PIO_SERCOM);
    #endif
    #ifndef ENABLE_SERIAL3
    pinPeripheral(2, PIO_SERCOM);
    pinPeripheral(5, PIO_SERCOM);
    #endif
    #endif


    pinMode(greenLED, OUTPUT);
    digitalWrite(greenLED, LOW);
    pinMode(redLED, OUTPUT);
    digitalWrite(redLED, LOW);

    greenredflash();




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



    Logger::setLoggerTimeZone(timeZone);

    Logger::setRTCTimeZone(0);


    dataLogger.attachModem(modem);
    modem.setModemLED(modemLEDPin);
    dataLogger.setLoggerPins(wakePin, sdCardSSPin, sdCardPwrPin, buttonPin, greenLED);


    dataLogger.begin();



    if (getBatteryVoltage() > minimum_bat_voltage)
    {
        modem.modemPowerUp();
        modem.wake();
        modem.setup();



        if (getBatteryVoltage() > moderate_bat_voltage ||
            dataLogger.getNowEpoch() < 1546300800 ||
            dataLogger.getNowEpoch() > 1735689600)
        {

            Serial.println(F("Attempting to connect to the internet and synchronize RTC with NIST"));
            if (modem.connectInternet(120000L))
            {
                dataLogger.setRTClock(modem.getNISTTime());
            }
        }
    }


    if (getBatteryVoltage() > minimum_bat_voltage)
    {
        Serial.println(F("Setting up sensors..."));
        varArray.setupSensors();
    }






    if (getBatteryVoltage() > minimum_bat_voltage)
    {
        Serial.println(F("Setting up file on SD card"));
        dataLogger.turnOnSDcard(true);
        dataLogger.createLogFile(true);
        dataLogger.turnOffSDcard(true);
    }



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


    Serial.println(F("Putting processor to sleep"));
    dataLogger.systemSleep();
}
# 480 "C:/Users/murrayr/Documents/Arduino/libraries/ModularSensors/platformio_waikato_scripts/yosemitech_4000_with_4g_bee/src/yosemitech_4000_with_4g_bee.ino"
void loop()
{



    if (getBatteryVoltage() < minimum_bat_voltage)
    {
        dataLogger.systemSleep();
    }

    else if (getBatteryVoltage() < moderate_bat_voltage)
    {
        dataLogger.logData();
    }

    else
    {
       Serial.println(getBatteryVoltage());

        dataLogger.logDataAndPublish();
    }
}