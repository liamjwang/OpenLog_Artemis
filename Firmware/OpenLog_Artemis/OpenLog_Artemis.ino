/*
  OpenLog Artemis
  By: Nathan Seidle
  SparkFun Electronics
  Date: November 26th, 2019
  License: This code is public domain but you buy me a beer if you use this
  and we meet someday (Beerware license).
  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/15793

  This firmware runs the OpenLog Artemis. A large variety of system settings can be
  adjusted by connecting at 115200bps.

  The Board should be set to SparkFun Apollo3 \ RedBoard Artemis ATP.

  Please note: this version of the firmware compiles on v2.1.0 of the Apollo3 boards.

  (At the time of writing, data logging with the the u-blox ZED-F9P is problematic when using v2.1.1 of the core.)

  v1.0 Power Consumption:
   Sleep between reads, RTC fully charged, no Qwiic, SD, no USB, no Power LED: 260uA
   10Hz logging IMU, no Qwiic, SD, no USB, no Power LED: 9-27mA

  TODO:
  (done) Create settings file for sensor. Load after qwiic bus is scanned.
  (done on larger Strings) Remove String dependencies.
  (done) Bubble sort list of devices.
  (done) Remove listing for muxes.
  (done) Verify the printing of all sensors is %f, %d correct
  (done) Add begin function seperate from everything, call after wakeup instead of detect
  (done) Add counter to output to look for memory leaks on long runs
  (done) Add AHT20 support
  (done) Add SHTC3 support
  (done) Change settings extension to txt
  (done) Fix max I2C speed to use linked list
  Currently device settings are not recorded to EEPROM, only deviceSettings.txt
  Is there a better way to dynamically create size of outputData array so we don't ever get larger than X sensors outputting?
  Find way to store device configs into EEPROM
  Log four pressure sensors and graph them on plotter
  (checked) Test GPS - not sure about %d with int32s. Does lat, long, and alt look correct?
  (done) Test NAU7802s
  (done) Test SCD30s (Add an extended delay for the SCD30. (Issue #5))
  (won't do?) Add a 'does not like to be powered cycled' setting for each device type. I think this has been superceded by "Add individual power-on delays for each sensor type?.
  (done) Add support for logging VIN
  (done) Investigate error in time between logs (https://github.com/sparkfun/OpenLog_Artemis/issues/13)
  (done) Invesigate RTC reset issue (https://github.com/sparkfun/OpenLog_Artemis/issues/13 + https://forum.sparkfun.com/viewtopic.php?f=123&t=53157)
    The solution is to make sure that the OLA goes into deep sleep as soon as the voltage monitor detects that the power has failed.
    The user will need to press the reset button once power has been restored. Using the WDT to check the monitor and do a POR wasn't reliable.
  (done) Investigate requires-reset issue on battery power (") (X04 + CCS811/BME280 enviro combo)
  (done) Add a fix so that the MS8607 does not also appear as an MS5637
  (done) Add "set RTC from GPS" functionality
  (done) Add UTCoffset functionality (including support for negative numbers)
  (done) Figure out how to give the u-blox time to establish a fix if it has been powered down between log intervals. The user can specify up to 60s for the Qwiic power-on delay.
  Add support for VREG_ENABLE
  (done) Add support for PWR_LED
  (done) Use the WDT to reset the Artemis when power is reconnected (previously the Artemis would have stayed in deep sleep)
  Add a callback function to the u-blox library so we can abort waiting for UBX data if the power goes low
  (done) Add support for the ADS122C04 ADC (Qwiic PT100)
  (done) Investigate why usBetweenReadings appears to be longer than expected. We needed to read millis _before_ enabling the lower power clock!
  (done) Correct u-blox pull-ups
  (done) Add an olaIdentifier to prevent problems when using two code variants that have the same sizeOfSettings
  (done) Add a fix for the IMU wake-up issue identified in https://github.com/sparkfun/OpenLog_Artemis/issues/18
  (done) Add a "stop logging" feature on GPIO 32: allow the pin to be used to read a stop logging button instead of being an analog input
  (done) Allow the user to set the default qwiic bus pull-up resistance (u-blox will still use 'none')
  (done) Add support for low battery monitoring using VIN
  (done) Output sensor data via the serial TX pin (Issue #32)
  (done) Add support for SD card file transfer (ZMODEM) and delete. (Issue #33) With thanks to: ecm-bitflipper (https://github.com/ecm-bitflipper/Arduino_ZModem)
  (done) Add file creation and access timestamps
  (done) Add the ability to trigger data collection via Pin 11 (Issue #36)
  (done) Correct the measurement count misbehaviour (Issue #31)
  (done) Use the corrected IMU temperature calculation (Issue #28)
  (done) Add individual power-on delays for each sensor type. Add an extended delay for the SCD30. (Issue #5)
  (done) v1.7: Fix readVin after sleep bug: https://github.com/sparkfun/OpenLog_Artemis/issues/39
  (done) Change detectQwiicDevices so that the MCP9600 (Qwiic Thermocouple) is detected correctly
  (done) Add support for the MPRLS0025PA micro pressure sensor
  (done) Add support for the SN-GCJA5 particle sensor
  (done) Add IMU accelerometer and gyro full scale and digital low pass filter settings to menuIMU
  (done) Add a fix to make sure the MS8607 is detected correctly: https://github.com/sparkfun/OpenLog_Artemis/issues/54
  (done) Add logMicroseconds: https://github.com/sparkfun/OpenLog_Artemis/issues/49
  (done) Add an option to use autoPVT when logging GNSS data: https://github.com/sparkfun/OpenLog_Artemis/issues/50
  (done) Corrected an issue when using multiple MS8607's: https://github.com/sparkfun/OpenLog_Artemis/issues/62
  (done) Add a feature to use the TX and RX pins as a duplicate Terminal
  (done) Add serial log timestamps with a token (as suggested by @DennisMelamed in PR https://github.com/sparkfun/OpenLog_Artemis/pull/70 and Issue https://github.com/sparkfun/OpenLog_Artemis/issues/63)
  (done) Add "sleep on pin" functionality based @ryanneve's PR https://github.com/sparkfun/OpenLog_Artemis/pull/64 and Issue https://github.com/sparkfun/OpenLog_Artemis/issues/46
  (done) Add "wake at specified times" functionality based on Issue https://github.com/sparkfun/OpenLog_Artemis/issues/46
  (done) Add corrections for the SCD30 based on Forum post by paulvha: https://forum.sparkfun.com/viewtopic.php?p=222455#p222455
  (done) Add support for the SGP40 VOC Index sensor
  (done) Add support for the SDP3X Differential Pressure sensor
  (done) Add support for the MS5837 - as used in the BlueRobotics BAR02 and BAR30 water pressure sensors
  (done) Correct an issue which was causing the OLA to crash when waking from sleep and outputting serial data https://github.com/sparkfun/OpenLog_Artemis/issues/79
  (done) Correct low-power code as per https://github.com/sparkfun/OpenLog_Artemis/issues/78
  (done) Correct a bug in menuAttachedDevices when useTxRxPinsForTerminal is enabled https://github.com/sparkfun/OpenLog_Artemis/issues/82
  (done) Add ICM-20948 DMP support. Requires v1.2.6 of the ICM-20948 library. DMP logging is limited to: Quat6 or Quat9, plus raw accel, gyro and compass. https://github.com/sparkfun/OpenLog_Artemis/issues/47
  (done) Add support for exFAT. Requires v2.0.6 of Bill Greiman's SdFat library. https://github.com/sparkfun/OpenLog_Artemis/issues/34
  (done) Add minimum awake time: https://github.com/sparkfun/OpenLog_Artemis/issues/83
  (done) Add support for the Pulse Oximeter: https://github.com/sparkfun/OpenLog_Artemis/issues/81
  (done - but does not work) Add support for the Qwiic Button. The QB uses clock-stretching and the Artemis really doesn't enjoy that...
  (done) Increase DMP data resolution to five decimal places https://github.com/sparkfun/OpenLog_Artemis/issues/90

  (in progress) Update to Apollo3 v2.1.0 - FIRMWARE_VERSION_MAJOR = 2.
  (done) Implement printf float (OLA uses printf float in _so_ many places...): https://github.com/sparkfun/Arduino_Apollo3/issues/278
  (worked around) attachInterrupt(PIN_POWER_LOSS, powerDownOLA, FALLING); triggers an immediate interrupt - https://github.com/sparkfun/Arduino_Apollo3/issues/416
  (done) Add a setQwiicPullups function
  (done) Check if we need ap3_set_pin_to_analog when coming out of sleep
  (done) Investigate why code does not wake from deep sleep correctly
  (worked around) Correct SerialLog RX: https://github.com/sparkfun/Arduino_Apollo3/issues/401
    The work-around is to use Serial1 in place of serialLog and then to manually force UART1 to use pins 12 and 13
    We need a work-around anyway because if pins 12 or 13 have been used as analog inputs, Serial1.begin does not re-configure them for UART TX and RX
  (in progress) Reduce sleep current as much as possible. v1.2.1 achieved ~110uA. With v2.1.0 the draw is more like 260uA...

  v2.1:
  (in progress) Add BLE service and characteristics
*/

const int FIRMWARE_VERSION_MAJOR = 2;
const int FIRMWARE_VERSION_MINOR = 1;

//Define the OLA board identifier:
//  This is an int which is unique to this variant of the OLA and which allows us
//  to make sure that the settings in EEPROM are correct for this version of the OLA
//  (sizeOfSettings is not necessarily unique and we want to avoid problems when swapping from one variant to another)
//  It is the sum of:
//    the variant * 0x100 (OLA = 1; GNSS_LOGGER = 2; GEOPHONE_LOGGER = 3)
//    the major firmware version * 0x10
//    the minor firmware version
#define OLA_IDENTIFIER 0x120 // Stored as 288 decimal in OLA_settings.txt

#include "settings.h"

//Define the pin functions
//Depends on hardware version. This can be found as a marking on the PCB.
//x04 was the SparkX 'black' version.
//v10 was the first red version.
#define HARDWARE_VERSION_MAJOR 1
#define HARDWARE_VERSION_MINOR 0

#if(HARDWARE_VERSION_MAJOR == 0 && HARDWARE_VERSION_MINOR == 4)
const byte PIN_MICROSD_CHIP_SELECT = 10;
const byte PIN_IMU_POWER = 22;
#elif(HARDWARE_VERSION_MAJOR == 1 && HARDWARE_VERSION_MINOR == 0)
const byte PIN_MICROSD_CHIP_SELECT = 23;
const byte PIN_IMU_POWER = 27;
const byte PIN_PWR_LED = 29;
const byte PIN_VREG_ENABLE = 25;
const byte PIN_VIN_MONITOR = 34; // VIN/3 (1M/2M - will require a correction factor)
#endif

const byte PIN_POWER_LOSS = 3;
//const byte PIN_LOGIC_DEBUG = 11; // Useful for debugging issues like the slippery mux bug
const byte PIN_MICROSD_POWER = 15;
const byte PIN_QWIIC_POWER = 18;
const byte PIN_STAT_LED = 19;
const byte PIN_IMU_INT = 37;
const byte PIN_IMU_CHIP_SELECT = 44;
const byte PIN_STOP_LOGGING = 32;
const byte BREAKOUT_PIN_32 = 32;
const byte BREAKOUT_PIN_TX = 12;
const byte BREAKOUT_PIN_RX = 13;
const byte BREAKOUT_PIN_11 = 11;
const byte PIN_TRIGGER = 11;
const byte PIN_QWIIC_SCL = 8;
const byte PIN_QWIIC_SDA = 9;

const byte PIN_SPI_SCK = 5;
const byte PIN_SPI_CIPO = 6;
const byte PIN_SPI_COPI = 7;

// Include this many extra bytes when starting a mux - to try and avoid the slippery mux bug
// This should be 0 but 3 or 7 seem to work better depending on which way the wind is blowing.
const byte EXTRA_MUX_STARTUP_BYTES = 3;

enum returnStatus {
    STATUS_GETBYTE_TIMEOUT = 255,
    STATUS_GETNUMBER_TIMEOUT = -123455555,
    STATUS_PRESSED_X,
};

//Setup Qwiic Port
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
#include <Wire.h>

TwoWire qwiic(PIN_QWIIC_SDA, PIN_QWIIC_SCL); //Will use pads 8/9
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

//EEPROM for storing settings
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
#include <EEPROM.h>
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

//microSD Interface
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
#include <SPI.h>

#include <SdFat.h> //SdFat v2.0.7 by Bill Greiman: http://librarymanager/All#SdFat_exFAT

#define SD_FAT_TYPE 3 // SD_FAT_TYPE = 0 for SdFat/File, 1 for FAT16/FAT32, 2 for exFAT, 3 for FAT16/FAT32 and exFAT.
#define SD_CONFIG SdSpiConfig(PIN_MICROSD_CHIP_SELECT, SHARED_SPI, SD_SCK_MHZ(24)) // 24MHz

#if SD_FAT_TYPE == 1
typedef SdFat32 SdFileSystemType;
typedef File32 SdFileType;
#elif SD_FAT_TYPE == 2
typedef SdExFat SdFileSystemType;
typedef ExFile SdFileType;
#elif SD_FAT_TYPE == 3
typedef SdFs SdFileSystemType;
typedef FsFile SdFileType;
#else // SD_FAT_TYPE == 0
typedef SdFat SdFileSystemType;
typedef File SdFileType;
#endif  // SD_FAT_TYPE

SdFileSystemType sd;
SdFileType sensorDataFile; //File that all sensor data is written to
SdFileType serialDataFile; //File that all incoming serial data is written to

//#define PRINT_LAST_WRITE_TIME // Uncomment this line to enable the 'measure the time between writes' diagnostic

char sensorDataFileName[30] = ""; //We keep a record of this file name so that we can re-open it upon wakeup from sleep
char serialDataFileName[30] = ""; //We keep a record of this file name so that we can re-open it upon wakeup from sleep
const int sdPowerDownDelay = 100; //Delay for this many ms before turning off the SD card power
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

//Add RTC interface for Artemis
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
#include "RTC.h" //Include RTC library included with the Aruino_Apollo3 core

Apollo3RTC myRTC; //Create instance of RTC class
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

//Create UART instance for OpenLog style serial logging
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

//UART SerialLog(BREAKOUT_PIN_TX, BREAKOUT_PIN_RX);  // Declares a Uart object called SerialLog with TX on pin 12 and RX on pin 13

uint64_t lastSeriaLogSyncTime = 0;
uint64_t lastAwakeTimeMillis;
const int MAX_IDLE_TIME_MSEC = 500;
bool newSerialData = false;
char incomingBuffer[256 * 2]; //This size of this buffer is sensitive. Do not change without analysis using OpenLog_Serial.
int incomingBufferSpot = 0;
int charsReceived = 0; //Used for verifying/debugging serial reception
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

//Add ICM IMU interface
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
#include "ICM_20948.h"  // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU

ICM_20948_SPI myICM;
icm_20948_DMP_data_t dmpData; // Global storage for the DMP data - extracted from the FIFO
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

//Header files for all compatible Qwiic sensors
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

#include "SparkFun_I2C_Mux_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_I2C_Mux
#include "SparkFunCCS811.h" //Click here to get the library: http://librarymanager/All#SparkFun_CCS811
#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X
#include "SparkFunBME280.h" //Click here to get the library: http://librarymanager/All#SparkFun_BME280
#include "SparkFun_LPS25HB_Arduino_Library.h"  //Click here to get the library: http://librarymanager/All#SparkFun_LPS25HB
#include "SparkFun_VEML6075_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_VEML6075
#include "SparkFun_PHT_MS8607_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_PHT_MS8607
#include "SparkFun_MCP9600.h" //Click here to get the library: http://librarymanager/All#SparkFun_MCP9600
#include "SparkFun_SGP30_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_SGP30
#include "SparkFun_VCNL4040_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_VCNL4040
#include "SparkFun_MS5637_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_MS5637
#include "SparkFun_TMP117.h" //Click here to get the library: http://librarymanager/All#SparkFun_TMP117
#include "SparkFun_u-blox_GNSS_Arduino_Library.h" //http://librarymanager/All#SparkFun_u-blox_GNSS
#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_NAU7802
#include "SparkFun_SCD30_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_SCD30
#include "SparkFun_Qwiic_Humidity_AHT20.h" //Click here to get the library: http://librarymanager/All#Qwiic_Humidity_AHT20 by SparkFun
#include "SparkFun_SHTC3.h" // Click here to get the library: http://librarymanager/All#SparkFun_SHTC3
#include "SparkFun_ADS122C04_ADC_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_ADS122C04
#include "SparkFun_MicroPressure.h" // Click here to get the library: http://librarymanager/All#SparkFun_MicroPressure
#include "SparkFun_Particle_Sensor_SN-GCJA5_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_Particle_Sensor_SN-GCJA5
#include "SparkFun_SGP40_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_SGP40
#include "SparkFun_SDP3x_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_SDP3x
#include "MS5837.h" // Click here to download the library: https://github.com/sparkfunX/BlueRobotics_MS5837_Library
#include "SparkFun_Qwiic_Button.h" // Click here to get the library: http://librarymanager/All#SparkFun_Qwiic_Button_Switch
#include "SparkFun_Bio_Sensor_Hub_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_Bio_Sensor

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

//Global variables
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
uint64_t measurementStartTime; //Used to calc the actual update rate. Max is ~80,000,000ms in a 24 hour period.
uint64_t lastSDFileNameChangeTime; //Used to calculate the interval since the last SD filename change
unsigned long measurementCount = 0; //Used to calc the actual update rate.
float lastMeasurementValue = 0; //Used to calc the actual update rate.
unsigned long measurementTotal = 0; //The total number of recorded measurements. (Doesn't get reset when the menu is opened)
char outputData[512 * 2]; //Factor of 512 for easier recording to SD in 512 chunks

bool newImuDataFlag = false;
struct MSG {
    // timestamp
    uint32_t stamp;
    uint64_t micros;
    // gyro
    int32_t Q1;
    int32_t Q2;
    int32_t Q3;
    // accel
    int16_t X;
    int16_t Y;
    int16_t Z;
};

typedef union {
    MSG number;
    uint8_t bytes[sizeof(MSG)];
} MSG_UNION;
MSG_UNION outputDataBin;
//char outputDataBin[512 * 2]; //Factor of 512 for easier recording to SD in 512 chunks
//uint32_t outputDataLen; //Factor of 512 for easier recording to SD in 512 chunks
unsigned long lastReadTime = 0; //Used to delay until user wants to record a new reading
unsigned long lastDataLogSyncTime = 0; //Used to record to SD every half second
unsigned int totalCharactersPrinted = 0; //Limit output rate based on baud rate and number of characters to print
bool takeReading = true; //Goes true when enough time has passed between readings or we've woken from sleep
bool sleepAfterRead = false; //Used to keep the code awake for at least minimumAwakeTimeMillis
const uint64_t maxUsBeforeSleep = 2000000ULL; //Number of us between readings before sleep is activated.
const byte menuTimeout = 15; //Menus will exit/timeout after this number of seconds
volatile static bool stopLoggingSeen = false; //Flag to indicate if we should stop logging
uint64_t qwiicPowerOnTime = 0; //Used to delay after Qwiic power on to allow sensors to power on, then answer autodetect
unsigned long qwiicPowerOnDelayMillis; //Wait for this many milliseconds after turning on the Qwiic power before attempting to communicate with Qwiic devices
int lowBatteryReadings = 0; // Count how many times the battery voltage has read low
const int lowBatteryReadingsLimit = 10; // Don't declare the battery voltage low until we have had this many consecutive low readings (to reject sampling noise)
volatile static bool triggerEdgeSeen = false; //Flag to indicate if a trigger interrupt has been seen
char serialTimestamp[40]; //Buffer to store serial timestamp, if needed
volatile static bool powerLossSeen = false; //Flag to indicate if a power loss event has been seen
bool usingBLE = false; //Flag to indicate if BLE is in use
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

uint8_t getByteChoice(int numberOfSeconds, bool updateDZSERIAL = false); // Header

// gfvalvo's flash string helper code: https://forum.arduino.cc/index.php?topic=533118.msg3634809#msg3634809
void SerialPrint(const char *);

void SerialPrint(const __FlashStringHelper *);

void SerialPrintln(const char *);

void SerialPrintln(const __FlashStringHelper *);

void DoSerialPrint(char (*)(const char *), const char *, bool newLine = false);

#define DUMP(varname) {Serial.printf("%s: %d\r\n", #varname, varname); if (settings.useTxRxPinsForTerminal == true) Serial1.printf("%s: %d\r\n", #varname, varname);}
#define SerialPrintf1(var) {Serial.printf( var ); if (settings.useTxRxPinsForTerminal == true) Serial1.printf( var );}
#define SerialPrintf2(var1, var2) {Serial.printf( var1, var2 ); if (settings.useTxRxPinsForTerminal == true) Serial1.printf( var1, var2 );}
#define SerialPrintf3(var1, var2, var3) {Serial.printf( var1, var2, var3 ); if (settings.useTxRxPinsForTerminal == true) Serial1.printf( var1, var2, var3 );}
#define SerialPrintf4(var1, var2, var3, var4) {Serial.printf( var1, var2, var3, var4 ); if (settings.useTxRxPinsForTerminal == true) Serial1.printf( var1, var2, var3, var4 );}
#define SerialPrintf5(var1, var2, var3, var4, var5) {Serial.printf( var1, var2, var3, var4, var5 ); if (settings.useTxRxPinsForTerminal == true) Serial1.printf( var1, var2, var3, var4, var5 );}

// The Serial port for the Zmodem connection
// must not be the same as DSERIAL unless all
// debugging output to DSERIAL is removed
Stream *ZSERIAL;

// Serial output for debugging info for Zmodem
Stream *DSERIAL;

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

//BLE Support
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
#include <ArduinoBLE.h>

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define kTargetServiceUUID    "2488bd28-b1df-4fe0-8611-22fda7c645f0"
#define kTargetServiceName    "OpenLog Right"

#define kMaxCharacteristics   50

#define kCharacteristicUUID00 "19829bd6-5ec8-4623-90c1-2e7094653cc9"
#define kCharacteristicUUID01 "1e182bb7-f3fd-4240-bc91-aabb9436c0b7"
#define kCharacteristicUUID02 "3198a4f7-e0da-4ec0-aafe-d978201bdcaa"
#define kCharacteristicUUID03 "4a3c7e31-deae-43ed-90a3-01a71c7ad28b"
#define kCharacteristicUUID04 "5c697a4f-7590-47cb-9b90-90d7732a7529"
#define kCharacteristicUUID05 "21680407-7051-434a-8fb5-362ea1c01916"
#define kCharacteristicUUID06 "53acd7ae-09f2-4015-9bfc-092342c68b1d"
#define kCharacteristicUUID07 "7838e6fa-b9da-4221-985b-12116226cacf"
#define kCharacteristicUUID08 "5b7e835d-fe02-48c3-9ba5-748ce472091e"
#define kCharacteristicUUID09 "1934c672-d676-4b6f-92f8-1a144bc22155"
#define kCharacteristicUUID10 "4a00713a-5a15-445f-9c8e-24b80ca99a64"
#define kCharacteristicUUID11 "ff943f44-547d-4533-9368-e8b3ef077a78"
#define kCharacteristicUUID12 "1bb82b3c-3d26-46f7-b897-edd5cc58d67d"
#define kCharacteristicUUID13 "a09f1db3-6031-427a-a72b-37f352591f88"
#define kCharacteristicUUID14 "492ff514-066a-467b-b104-83138aa66a9c"
#define kCharacteristicUUID15 "b03e30a0-f6af-47fd-b393-29e64a814f54"
#define kCharacteristicUUID16 "2a286649-ba85-46eb-9305-1e6e23b0dd55"
#define kCharacteristicUUID17 "ca16dc92-3818-466c-a3da-145d8be3b321"
#define kCharacteristicUUID18 "cc4d8218-e374-40d4-a898-5c23ffe5902e"
#define kCharacteristicUUID19 "313e22de-f2cf-4c34-9f40-f52c98c48336"
#define kCharacteristicUUID20 "ede60308-c96e-4215-8fc5-dce0b8b8b927"
#define kCharacteristicUUID21 "e318dbfd-ca5e-4aef-a57c-00d2481b597b"
#define kCharacteristicUUID22 "8ec7604b-c40e-452e-a529-1a2d63dc3b5d"
#define kCharacteristicUUID23 "7c4baad4-b9a7-411d-af6d-15cbbc9141ec"
#define kCharacteristicUUID24 "79259096-16f4-4fc3-892f-ee49f157e006"
#define kCharacteristicUUID25 "6fe3cbaf-76fa-4156-b8ef-577f6800e37a"
#define kCharacteristicUUID26 "e8c2eb69-90e3-472f-ad99-a83ff56314dc"
#define kCharacteristicUUID27 "c7b2940c-b79a-4dff-bc19-bb2f5def8c0f"
#define kCharacteristicUUID28 "ba5325ae-ac70-4183-b68a-a182dc3a8274"
#define kCharacteristicUUID29 "a2806d24-1084-4c9f-b411-a5b98225d74f"
#define kCharacteristicUUID30 "883ff678-a5a6-4f4f-84e7-0d73c650b16b"
#define kCharacteristicUUID31 "ac06c415-824a-49e4-9010-7bfecdd4a46f"
#define kCharacteristicUUID32 "02b6989e-0e98-476e-90fa-f5b6b75c2bc8"
#define kCharacteristicUUID33 "252994d6-c986-4088-81e1-fef9b4838cf4"
#define kCharacteristicUUID34 "3bc99052-d8b3-49a7-9fae-5e1355e90ff6"
#define kCharacteristicUUID35 "8ac2d6cf-4a1a-430b-9114-4c3471e079ac"
#define kCharacteristicUUID36 "a0ba08b1-597c-41d7-8fac-036376cb5831"
#define kCharacteristicUUID37 "f8bc6eb3-3537-45ce-ad3e-0934039b60ac"
#define kCharacteristicUUID38 "5f417941-784d-4c6e-aeb0-7d7879f1cc24"
#define kCharacteristicUUID39 "84203f2e-242d-4e2e-9161-13404b4caed4"
#define kCharacteristicUUID40 "e2f40654-85da-4bba-83cd-5542e8cc7684"
#define kCharacteristicUUID41 "312aa6b1-fd93-4915-b8aa-6164c513b2cd"
#define kCharacteristicUUID42 "fa0f1903-a681-46e9-b0d4-7ca41fcd5105"
#define kCharacteristicUUID43 "3b6aba88-dc1f-4181-8419-074a2435a6da"
#define kCharacteristicUUID44 "49d41d09-2e3e-4527-825e-204fa5bace4c"
#define kCharacteristicUUID45 "8b2298e6-d49b-4461-883d-c35943838726"
#define kCharacteristicUUID46 "0728e304-2f04-406d-bb76-35a7cf377233"
#define kCharacteristicUUID47 "0a4ede58-7a90-4961-ad0e-4ca642030810"
#define kCharacteristicUUID48 "8a3e3003-dd0f-4ea4-916e-871cc3a213a0"
#define kCharacteristicUUID49 "508a72cd-6598-4468-9b76-69840a842c4f"

// helper for message limits
#define kMessageMax 50

#define BLE_UUID(val) ("bf88b656-" val "-4a61-86e0-69840a842c4f")

BLEService bleService(BLE_UUID("0000"));
BLEIntCharacteristic statusCharacteristic(BLE_UUID("0001"), BLERead | BLENotify); // 0 idle, 1 recording, 2 error
BLEIntCharacteristic startRecordingCharacteristic(BLE_UUID("0002"), BLEWrite); // write ms until start
BLEIntCharacteristic stopRecordingCharacteristic(BLE_UUID("0005"), BLEWrite); // write ms until stop
BLEStringCharacteristic fileNameCharacteristic(BLE_UUID("0003"), BLEWrite, kMessageMax);
BLEUnsignedIntCharacteristic rtcTimeCharacteristic(BLE_UUID("0004"), BLEWrite);

BLEStringCharacteristic bleCharacteristic00(kCharacteristicUUID00, BLERead | BLENotify, kMessageMax);
BLEStringCharacteristic bleCharacteristic01(kCharacteristicUUID01, BLERead | BLENotify, kMessageMax);
BLEStringCharacteristic bleCharacteristic02(kCharacteristicUUID02, BLERead | BLENotify, kMessageMax);
BLEStringCharacteristic bleCharacteristic03(kCharacteristicUUID03, BLERead | BLENotify, kMessageMax);
BLEStringCharacteristic bleCharacteristic04(kCharacteristicUUID04, BLERead | BLENotify, kMessageMax);
BLEStringCharacteristic bleCharacteristic05(kCharacteristicUUID05, BLERead | BLENotify, kMessageMax);
BLEStringCharacteristic bleCharacteristic06(kCharacteristicUUID06, BLERead | BLENotify, kMessageMax);
BLEStringCharacteristic bleCharacteristic07(kCharacteristicUUID07, BLERead | BLENotify, kMessageMax);
BLEStringCharacteristic bleCharacteristic08(kCharacteristicUUID08, BLERead | BLENotify, kMessageMax);
BLEStringCharacteristic bleCharacteristic09(kCharacteristicUUID09, BLERead | BLENotify, kMessageMax);
BLEStringCharacteristic bleCharacteristic10(kCharacteristicUUID10, BLERead | BLENotify, kMessageMax);
BLEStringCharacteristic bleCharacteristic11(kCharacteristicUUID11, BLERead | BLENotify, kMessageMax);
BLEStringCharacteristic bleCharacteristic12(kCharacteristicUUID12, BLERead | BLENotify, kMessageMax);
BLEStringCharacteristic bleCharacteristic13(kCharacteristicUUID13, BLERead | BLENotify, kMessageMax);
BLEStringCharacteristic bleCharacteristic14(kCharacteristicUUID14, BLERead | BLENotify, kMessageMax);
BLEStringCharacteristic bleCharacteristic15(kCharacteristicUUID15, BLERead | BLENotify, kMessageMax);
BLEStringCharacteristic bleCharacteristic16(kCharacteristicUUID16, BLERead | BLENotify, kMessageMax);
BLEStringCharacteristic bleCharacteristic17(kCharacteristicUUID17, BLERead | BLENotify, kMessageMax);
BLEStringCharacteristic bleCharacteristic18(kCharacteristicUUID18, BLERead | BLENotify, kMessageMax);
BLEStringCharacteristic bleCharacteristic19(kCharacteristicUUID19, BLERead | BLENotify, kMessageMax);
BLEStringCharacteristic bleCharacteristic20(kCharacteristicUUID20, BLERead | BLENotify, kMessageMax);
BLEStringCharacteristic bleCharacteristic21(kCharacteristicUUID21, BLERead | BLENotify, kMessageMax);
BLEStringCharacteristic bleCharacteristic22(kCharacteristicUUID22, BLERead | BLENotify, kMessageMax);
BLEStringCharacteristic bleCharacteristic23(kCharacteristicUUID23, BLERead | BLENotify, kMessageMax);
BLEStringCharacteristic bleCharacteristic24(kCharacteristicUUID24, BLERead | BLENotify, kMessageMax);
BLEStringCharacteristic bleCharacteristic25(kCharacteristicUUID25, BLERead | BLENotify, kMessageMax);
BLEStringCharacteristic bleCharacteristic26(kCharacteristicUUID26, BLERead | BLENotify, kMessageMax);
BLEStringCharacteristic bleCharacteristic27(kCharacteristicUUID27, BLERead | BLENotify, kMessageMax);
BLEStringCharacteristic bleCharacteristic28(kCharacteristicUUID28, BLERead | BLENotify, kMessageMax);
BLEStringCharacteristic bleCharacteristic29(kCharacteristicUUID29, BLERead | BLENotify, kMessageMax);
BLEStringCharacteristic bleCharacteristic30(kCharacteristicUUID30, BLERead | BLENotify, kMessageMax);
BLEStringCharacteristic bleCharacteristic31(kCharacteristicUUID31, BLERead | BLENotify, kMessageMax);
BLEStringCharacteristic bleCharacteristic32(kCharacteristicUUID32, BLERead | BLENotify, kMessageMax);
BLEStringCharacteristic bleCharacteristic33(kCharacteristicUUID33, BLERead | BLENotify, kMessageMax);
BLEStringCharacteristic bleCharacteristic34(kCharacteristicUUID34, BLERead | BLENotify, kMessageMax);
BLEStringCharacteristic bleCharacteristic35(kCharacteristicUUID35, BLERead | BLENotify, kMessageMax);
BLEStringCharacteristic bleCharacteristic36(kCharacteristicUUID36, BLERead | BLENotify, kMessageMax);
BLEStringCharacteristic bleCharacteristic37(kCharacteristicUUID37, BLERead | BLENotify, kMessageMax);
BLEStringCharacteristic bleCharacteristic38(kCharacteristicUUID38, BLERead | BLENotify, kMessageMax);
BLEStringCharacteristic bleCharacteristic39(kCharacteristicUUID39, BLERead | BLENotify, kMessageMax);
BLEStringCharacteristic bleCharacteristic40(kCharacteristicUUID40, BLERead | BLENotify, kMessageMax);
BLEStringCharacteristic bleCharacteristic41(kCharacteristicUUID41, BLERead | BLENotify, kMessageMax);
BLEStringCharacteristic bleCharacteristic42(kCharacteristicUUID42, BLERead | BLENotify, kMessageMax);
BLEStringCharacteristic bleCharacteristic43(kCharacteristicUUID43, BLERead | BLENotify, kMessageMax);
BLEStringCharacteristic bleCharacteristic44(kCharacteristicUUID44, BLERead | BLENotify, kMessageMax);
BLEStringCharacteristic bleCharacteristic45(kCharacteristicUUID45, BLERead | BLENotify, kMessageMax);
BLEStringCharacteristic bleCharacteristic46(kCharacteristicUUID46, BLERead | BLENotify, kMessageMax);
BLEStringCharacteristic bleCharacteristic47(kCharacteristicUUID47, BLERead | BLENotify, kMessageMax);
BLEStringCharacteristic bleCharacteristic48(kCharacteristicUUID48, BLERead | BLENotify, kMessageMax);
BLEStringCharacteristic bleCharacteristic49(kCharacteristicUUID49, BLERead | BLENotify, kMessageMax);

int numBLECharacteristics = 0;
char bleCharacteristicsValues[kMaxCharacteristics][kMessageMax];

unsigned long commandFutureTime = 0;
unsigned int commandFlag = 0;


void ConnectHandler(BLEDevice central) {
    // central connected event handler
    Serial.print("Connected event, central: ");
    Serial.println(central.address());
    BLE.advertise();
}

void DisconnectHandler(BLEDevice central) {
    // central disconnected event handler
    Serial.print("Disconnected event, central: ");
    Serial.println(central.address());
    BLE.advertise();
}

void onTimeWritten(BLEDevice central, BLECharacteristic characteristic) {
    uint32_t epoch_rcv;
    characteristic.readValue(epoch_rcv);
    myRTC.setEpoch(epoch_rcv); //Manually set RTC
    lastSDFileNameChangeTime = rtcMillis(); // Record the time of the file name change
    SerialPrint(F("BLE timestamp recieved! Value: "));
    SerialPrintf2("%u\n", epoch_rcv)
}

void onStartRecording(BLEDevice central, BLECharacteristic characteristic) {
    if (settings.enableSD && online.microSD) {
        int32_t command_value;
        characteristic.readValue(command_value);

        SerialPrint(F("BLE Command recieved! Value: "));
        SerialPrintf2("%i\n", command_value)


        SerialPrint(F("Recording start..."));
        commandFutureTime = millis() + command_value;
        commandFlag = 1;
    }
}


void onStopRecording(BLEDevice central, BLECharacteristic characteristic) {
    if (settings.enableSD && online.microSD) {
        int32_t command_value;
        characteristic.readValue(command_value);

        SerialPrint(F("BLE Command recieved! Value: "));
        SerialPrintf2("%i\n", command_value)

        SerialPrint(F("Recording stop..."));
        commandFutureTime = millis() + command_value;
        commandFlag = 2;
    }
}

void setup() {
    //If 3.3V rail drops below 3V, system will power down and maintain RTC
    pinMode(PIN_POWER_LOSS, INPUT); // BD49K30G-TL has CMOS output and does not need a pull-up

    delay(1); // Let PIN_POWER_LOSS stabilize

    if (digitalRead(PIN_POWER_LOSS) == LOW) powerDownOLA(); //Check PIN_POWER_LOSS just in case we missed the falling edge
    //attachInterrupt(PIN_POWER_LOSS, powerDownOLA, FALLING); // We can't do this with v2.1.0 as attachInterrupt causes a spontaneous interrupt
    attachInterrupt(PIN_POWER_LOSS, powerLossISR, FALLING);
    powerLossSeen = false; // Make sure the flag is clear

    powerLEDOn(); // Turn the power LED on - if the hardware supports it

    pinMode(PIN_STAT_LED, OUTPUT);
    digitalWrite(PIN_STAT_LED, HIGH); // Turn the STAT LED on while we configure everything

    SPI.begin(); //Needed if SD is disabled

    //Do not start Serial1 before productionTest() otherwise the pin configuration gets overwritten
    //and subsequent Serial1.begin's don't restore the pins to UART mode...

    productionTest(); //Check if we need to go into production test mode

    //We need to manually restore the Serial1 TX and RX pins after they were changed by productionTest()
    configureSerial1TxRx();

    Serial.begin(settings.serialTerminalBaudRate); //Default for initial debug messages if necessary
    Serial1.begin(settings.serialTerminalBaudRate); //Default for initial debug messages if necessary

    //pinMode(PIN_LOGIC_DEBUG, OUTPUT); // Debug pin to assist tracking down slippery mux bugs
    //digitalWrite(PIN_LOGIC_DEBUG, HIGH);

    // Use the worst case power on delay for the Qwiic bus for now as we don't yet know what sensors are connected
    // (worstCaseQwiicPowerOnDelay is defined in settings.h)
    qwiicPowerOnDelayMillis = worstCaseQwiicPowerOnDelay;

    EEPROM.init();

    beginQwiic(); // Turn the qwiic power on as early as possible

    beginSD(); //285 - 293ms

    enableCIPOpullUp(); // Enable CIPO pull-up _after_ beginSD

    loadSettings(); //50 - 250ms

    if (settings.useTxRxPinsForTerminal == true) {
        Serial1.flush(); //Complete any previous prints at the previous baud rate
        Serial1.begin(settings.serialTerminalBaudRate); // Restart the serial port
    } else {
        Serial1.flush(); //Complete any previous prints
        Serial1.end(); // Stop the SerialLog port
    }

    Serial.flush(); //Complete any previous prints
    Serial.begin(settings.serialTerminalBaudRate);

    SerialPrintf3("Artemis OpenLog v%d.%d\r\n", FIRMWARE_VERSION_MAJOR, FIRMWARE_VERSION_MINOR);
    SerialPrintf2("Msg size: %ld\r\n", sizeof(outputDataBin.number.stamp)+
            sizeof(outputDataBin.number.micros)+
            sizeof(outputDataBin.number.Q1)+
            sizeof(outputDataBin.number.Q2)+
            sizeof(outputDataBin.number.Q3)+
            sizeof(outputDataBin.number.X)+
            sizeof(outputDataBin.number.Y)+
            sizeof(outputDataBin.number.Z));


    if (settings.useGPIO32ForStopLogging == true) {
        SerialPrintln(F("Stop Logging is enabled. Pull GPIO pin 32 to GND to stop logging."));
        pinMode(PIN_STOP_LOGGING, INPUT_PULLUP);
        delay(1); // Let the pin stabilize
        attachInterrupt(PIN_STOP_LOGGING, stopLoggingISR, FALLING); // Enable the interrupt
        am_hal_gpio_pincfg_t intPinConfig = g_AM_HAL_GPIO_INPUT_PULLUP;
        intPinConfig.eIntDir = AM_HAL_GPIO_PIN_INTDIR_HI2LO;
        pin_config(PinName(PIN_STOP_LOGGING), intPinConfig); // Make sure the pull-up does actually stay enabled
        stopLoggingSeen = false; // Make sure the flag is clear
    }

    if (settings.useGPIO11ForTrigger == true) {
        pinMode(PIN_TRIGGER, INPUT_PULLUP);
        delay(1); // Let the pin stabilize
        am_hal_gpio_pincfg_t intPinConfig = g_AM_HAL_GPIO_INPUT_PULLUP;
        if (settings.fallingEdgeTrigger == true) {
            SerialPrintln(F("Falling-edge triggering is enabled. Sensor data will be logged on a falling edge on GPIO pin 11."));
            attachInterrupt(PIN_TRIGGER, triggerPinISR, FALLING); // Enable the interrupt
            intPinConfig.eIntDir = AM_HAL_GPIO_PIN_INTDIR_HI2LO;
        } else {
            SerialPrintln(F("Rising-edge triggering is enabled. Sensor data will be logged on a rising edge on GPIO pin 11."));
            attachInterrupt(PIN_TRIGGER, triggerPinISR, RISING); // Enable the interrupt
            intPinConfig.eIntDir = AM_HAL_GPIO_PIN_INTDIR_LO2HI;
        }
        pin_config(PinName(PIN_TRIGGER), intPinConfig); // Make sure the pull-up does actually stay enabled
        triggerEdgeSeen = false; // Make sure the flag is clear
    }

    analogReadResolution(14); //Increase from default of 10

    if (settings.useBLE) {
        for (int i = 0; i < kMaxCharacteristics; i++)
            bleCharacteristicsValues[i][0] = 0; //Clear the BLE characteristics values
    }

//    beginDataLogging(); //180ms
    lastSDFileNameChangeTime = rtcMillis(); // Record the time of the file name change

    serialTimestamp[0] = '\0'; // Empty the serial timestamp buffer

    if (settings.useTxRxPinsForTerminal == false) {
        beginSerialLogging(); //20 - 99ms
        beginSerialOutput(); // Begin serial data output on the TX pin
    }

    beginIMU(); //61ms

    if (online.microSD == true) SerialPrintln(F("SD card online"));
    else SerialPrintln(F("SD card offline"));

    if (online.dataLogging == true) SerialPrintln(F("Data logging online"));
    else SerialPrintln(F("Datalogging offline"));

    if (online.serialLogging == true) SerialPrintln(F("Serial logging online"));
    else SerialPrintln(F("Serial logging offline"));

    if (online.IMU == true) SerialPrintln(F("IMU online"));
    else SerialPrintln(F("IMU offline"));

    if (settings.logMaxRate == true) SerialPrintln(F("Logging analog pins at max data rate"));

    if (settings.enableTerminalOutput == false && settings.logData == true) SerialPrintln(F("Logging to microSD card with no terminal output"));

    if (detectQwiicDevices() == true) //159 - 865ms but varies based on number of devices attached
    {
        beginQwiicDevices(); //Begin() each device in the node list
        loadDeviceSettingsFromFile(); //Load config settings into node list
        configureQwiicDevices(); //Apply config settings to each device in the node list
        int deviceCount = printOnlineDevice(); // Pretty-print the online devices

        if ((deviceCount == 0) && (settings.resetOnZeroDeviceCount == true)) // Check for resetOnZeroDeviceCount
        {
            if ((Serial.available()) || ((settings.useTxRxPinsForTerminal == true) && (Serial1.available())))
                menuMain(); //Present user menu - in case the user wants to disable resetOnZeroDeviceCount
            else {
                SerialPrintln(F("*** Zero Qwiic Devices Found! Resetting... ***"));
                SerialFlush();
                resetArtemis(); //Thank you and goodnight...
            }
        }
    } else
        SerialPrintln(F("No Qwiic devices detected"));

    if (settings.showHelperText == true)
        printHelperText(true, true); //printHelperText to terminal and file
    else
        printHelperText(false, false); //call printHelperText to generate the number of characteristics

    if (settings.useBLE) {
        SerialPrintln(F("Starting BLE..."));

        if (!BLE.begin()) {
            SerialPrintln(F("BLE.begin failed!"));
        } else {
            BLE.setLocalName(kTargetServiceName);
            BLE.setDeviceName(kTargetServiceName);
            BLE.setAdvertisedService(bleService); //Add the service UUID

            bleService.addCharacteristic(statusCharacteristic);
            bleService.addCharacteristic(startRecordingCharacteristic);
            bleService.addCharacteristic(stopRecordingCharacteristic);
            bleService.addCharacteristic(fileNameCharacteristic);
            bleService.addCharacteristic(rtcTimeCharacteristic);

            startRecordingCharacteristic.setEventHandler(BLEWritten, onStartRecording);
            stopRecordingCharacteristic.setEventHandler(BLEWritten, onStopRecording);
            rtcTimeCharacteristic.setEventHandler(BLEWritten, onTimeWritten);
            BLE.setEventHandler(BLEConnected, ConnectHandler);
            BLE.setEventHandler(BLEDisconnected, DisconnectHandler);

            BLE.addService(bleService); //Add the service

            BLE.advertise(); //Start advertising
            usingBLE = true;
        }
    }

    //If we are sleeping between readings then we cannot rely on millis() as it is powered down
    //Use RTC instead
    measurementStartTime = bestMillis();

    digitalWrite(PIN_STAT_LED, LOW); // Turn the STAT LED off now that everything is configured

    lastAwakeTimeMillis = rtcMillis();

    //If we are immediately going to go to sleep after the first reading then
    //first present the user with the config menu in case they need to change something
    if (checkIfItIsTimeToSleep())
        menuMain();

    SerialPrintln(F("Setup complete!"));
    digitalWrite(PIN_STAT_LED, LOW);
}

void loop() {

    checkBattery(); // Check for low battery

    if (usingBLE)
    BLEDevice central = BLE.central();


    if (commandFlag != 0) {
        unsigned long currentTime = millis();
        if (currentTime < commandFutureTime) {
            delay(commandFutureTime - currentTime);
        }
        switch (commandFlag) {
            case 1:
                SerialPrint(F("Recording start..."));
                if (online.dataLogging == false) {
                    String string = fileNameCharacteristic.value();
                    if (string == "") {
                        string = "dataLog";
                    }
                    strcpy(sensorDataFileName, findNextAvailableLog(settings.nextDataLogNumber, string.c_str()));
                    beginDataLogging(); //180ms
                    if (settings.showHelperText == true) printHelperText(false, true); //printHelperText to sensor file
                    statusCharacteristic.writeValue(online.dataLogging ? 1 : 2);
                    if (online.dataLogging) digitalWrite(PIN_STAT_LED, HIGH);
                }
                break;
            case 2:
                SerialPrint(F("Recording stop..."));
                if (online.dataLogging == true) {
                    sensorDataFile.sync();
                    updateDataFileAccess(&sensorDataFile); // Update the file access time & date
                    sensorDataFile.close();
                    online.dataLogging = false;
                    statusCharacteristic.writeValue(0);
                    digitalWrite(PIN_STAT_LED, LOW);
                }
                break;
            default:
                SerialPrint(F("Unknown command!"));
                break;
        }
        measurementCount = 0;
        measurementStartTime = bestMillis();
        commandFlag = 0;
    }
    


    if ((Serial.available()) || ((settings.useTxRxPinsForTerminal == true) && (Serial1.available())))
        menuMain(); //Present user menu

    takeReading = true;

    //Is it time to get new data?
    if ((settings.logMaxRate == true) || (takeReading == true)) {
        takeReading = false;
        lastReadTime = micros();

        getData(); //Query all enabled sensors for data

        //Print to terminal
        if (settings.enableTerminalOutput == true)
            SerialPrint(outputData); //Print to terminal

        //Output to TX pin
        if ((settings.outputSerial == true) && (online.serialOutput == true))
            Serial1.print(outputData); //Print to TX pin

        //Record to SD
        if (settings.logData == true && newImuDataFlag) {
            newImuDataFlag = false;
            if (settings.enableSD && online.microSD) {
//                sensorDataFile.write(&outputDataBin.number.stamp, sizeof(outputDataBin.number.stamp));
                sensorDataFile.write(&outputDataBin.number.micros, sizeof(outputDataBin.number.micros));
                sensorDataFile.write(&outputDataBin.number.Q1, sizeof(outputDataBin.number.Q1));
                sensorDataFile.write(&outputDataBin.number.Q2, sizeof(outputDataBin.number.Q2));
                sensorDataFile.write(&outputDataBin.number.Q3, sizeof(outputDataBin.number.Q3));
                sensorDataFile.write(&outputDataBin.number.X, sizeof(outputDataBin.number.X));
                sensorDataFile.write(&outputDataBin.number.Y, sizeof(outputDataBin.number.Y));
                sensorDataFile.write(&outputDataBin.number.Z, sizeof(outputDataBin.number.Z));
            }
        }
    }

}

uint32_t howLongToSleepFor(void) {
    //Counter/Timer 6 will use the 32kHz clock
    //Calculate how many 32768Hz system ticks we need to sleep for:
    //sysTicksToSleep = msToSleep * 32768L / 1000
    //We need to be careful with the multiply as we will overflow uint32_t if msToSleep is > 131072

    //goToSleep will automatically compensate for how long we have been awake

    uint32_t msToSleep;

    if (checkSleepOnFastSlowPin())
        msToSleep = (uint32_t) (settings.slowLoggingIntervalSeconds * 1000UL);
    else if (checkSleepOnRTCTime()) {
        // checkSleepOnRTCTime has returned true, so we know that we are between slowLoggingStartMOD and slowLoggingStopMOD
        // We need to check how long it is until slowLoggingStopMOD (accounting for midnight!) and adjust the sleep duration
        // if slowLoggingStopMOD occurs before slowLoggingIntervalSeconds

        msToSleep = (uint32_t) (settings.slowLoggingIntervalSeconds * 1000UL); // Default to this

        myRTC.getTime(); // Get the RTC time
        long secondsOfDay = (myRTC.hour * 60 * 60) + (myRTC.minute * 60) + myRTC.seconds;

        long slowLoggingStopSOD = settings.slowLoggingStopMOD * 60; // Convert slowLoggingStop to seconds-of-day

        long secondsUntilStop = slowLoggingStopSOD - secondsOfDay; // Calculate how long it is until slowLoggingStop

        // If secondsUntilStop is negative then we know that now is before midnight and slowLoggingStop is after midnight
        if (secondsUntilStop < 0) secondsUntilStop += 24 * 60 * 60; // Add a day's worth of seconds if required to make secondsUntilStop positive

        if (secondsUntilStop < settings.slowLoggingIntervalSeconds) // If we need to sleep for less than slowLoggingIntervalSeconds
            msToSleep = (secondsUntilStop + 1) * 1000UL; // Adjust msToSleep, adding one extra second to make sure the next wake is > slowLoggingStop
    } else // checkSleepOnUsBetweenReadings
    {
        msToSleep = (uint32_t) (settings.usBetweenReadings / 1000ULL); // Sleep for usBetweenReadings
    }

    uint32_t sysTicksToSleep;
    if (msToSleep < 131000) {
        sysTicksToSleep = msToSleep * 32768L; // Do the multiply first for short intervals
        sysTicksToSleep = sysTicksToSleep / 1000L; // Now do the divide
    } else {
        sysTicksToSleep = msToSleep / 1000L; // Do the division first for long intervals (to avoid an overflow)
        sysTicksToSleep = sysTicksToSleep * 32768L; // Now do the multiply
    }

    return (sysTicksToSleep);
}

bool checkIfItIsTimeToSleep(void) {

    if (checkSleepOnUsBetweenReadings()
        || checkSleepOnRTCTime()
        || checkSleepOnFastSlowPin())
        return (true);
    else
        return (false);
}

//Go to sleep if the time between readings is greater than maxUsBeforeSleep (2 seconds) and triggering is not enabled
bool checkSleepOnUsBetweenReadings(void) {
    if ((settings.useGPIO11ForTrigger == false) && (settings.usBetweenReadings >= maxUsBeforeSleep))
        return (true);
    else
        return (false);
}

//Go to sleep if Fast/Slow logging on Pin 11 is enabled and Pin 11 is in the correct state
bool checkSleepOnFastSlowPin(void) {
    if ((settings.useGPIO11ForFastSlowLogging == true) && (digitalRead(PIN_TRIGGER) == settings.slowLoggingWhenPin11Is))
        return (true);
    else
        return (false);
}

// Go to sleep if useRTCForFastSlowLogging is enabled and RTC time is between the start and stop times
bool checkSleepOnRTCTime(void) {
    // Check if we should be sleeping based on useGPIO11ForFastSlowLogging and slowLoggingStartMOD + slowLoggingStopMOD
    bool sleepOnRTCTime = false;
    if (settings.useRTCForFastSlowLogging == true) {
        if (settings.slowLoggingStartMOD != settings.slowLoggingStopMOD) // Only perform the check if the start and stop times are not equal
        {
            myRTC.getTime(); // Get the RTC time
            int minutesOfDay = (myRTC.hour * 60) + myRTC.minute;

            if (settings.slowLoggingStartMOD > settings.slowLoggingStopMOD) // If slow logging starts later than the stop time (i.e. slow over midnight)
            {
                if ((minutesOfDay >= settings.slowLoggingStartMOD) || (minutesOfDay < settings.slowLoggingStopMOD))
                    sleepOnRTCTime = true;
            } else // Slow logging starts earlier than the stop time
            {
                if ((minutesOfDay >= settings.slowLoggingStartMOD) && (minutesOfDay < settings.slowLoggingStopMOD))
                    sleepOnRTCTime = true;
            }
        }
    }
    return (sleepOnRTCTime);
}

void beginQwiic() {
    pinMode(PIN_QWIIC_POWER, OUTPUT);
    pin_config(PinName(PIN_QWIIC_POWER), g_AM_HAL_GPIO_OUTPUT); // Make sure the pin does actually get re-configured
    qwiicPowerOn();
    qwiic.begin();
    setQwiicPullups(settings.qwiicBusPullUps); //Just to make it really clear what pull-ups are being used, set pullups here.
}

void setQwiicPullups(uint32_t qwiicBusPullUps) {
    //Change SCL and SDA pull-ups manually using pin_config
    am_hal_gpio_pincfg_t sclPinCfg = g_AM_BSP_GPIO_IOM1_SCL;
    am_hal_gpio_pincfg_t sdaPinCfg = g_AM_BSP_GPIO_IOM1_SDA;

    if (qwiicBusPullUps == 0) {
        sclPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_NONE; // No pull-ups
        sdaPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_NONE;
    } else if (qwiicBusPullUps == 1) {
        sclPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_1_5K; // Use 1K5 pull-ups
        sdaPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_1_5K;
    } else if (qwiicBusPullUps == 6) {
        sclPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_6K; // Use 6K pull-ups
        sdaPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_6K;
    } else if (qwiicBusPullUps == 12) {
        sclPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_12K; // Use 12K pull-ups
        sdaPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_12K;
    } else {
        sclPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_24K; // Use 24K pull-ups
        sdaPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_24K;
    }

    pin_config(PinName(PIN_QWIIC_SCL), sclPinCfg);
    pin_config(PinName(PIN_QWIIC_SDA), sdaPinCfg);
}

void beginSD() {
    pinMode(PIN_MICROSD_POWER, OUTPUT);
    pin_config(PinName(PIN_MICROSD_POWER), g_AM_HAL_GPIO_OUTPUT); // Make sure the pin does actually get re-configured
    pinMode(PIN_MICROSD_CHIP_SELECT, OUTPUT);
    pin_config(PinName(PIN_MICROSD_CHIP_SELECT), g_AM_HAL_GPIO_OUTPUT); // Make sure the pin does actually get re-configured
    digitalWrite(PIN_MICROSD_CHIP_SELECT, HIGH); //Be sure SD is deselected

    if (settings.enableSD == true) {
        // For reasons I don't understand, we seem to have to wait for at least 1ms after SPI.begin before we call microSDPowerOn.
        // If you comment the next line, the Artemis resets at microSDPowerOn when beginSD is called from wakeFromSleep...
        // But only on one of my V10 red boards. The second one I have doesn't seem to need the delay!?
        delay(5);

        microSDPowerOn();

        //Max power up time is 250ms: https://www.kingston.com/datasheets/SDCIT-specsheet-64gb_en.pdf
        //Max current is 200mA average across 1s, peak 300mA
        for (int i = 0; i < 10; i++) //Wait
        {
            checkBattery();
            delay(1);
        }

        if (sd.begin(SD_CONFIG) == false) // Try to begin the SD card using the correct chip select
        {
            printDebug(F("SD init failed (first attempt). Trying again...\r\n"));
            for (int i = 0; i < 250; i++) //Give SD more time to power up, then try again
            {
                checkBattery();
                delay(1);
            }
            if (sd.begin(SD_CONFIG) == false) // Try to begin the SD card using the correct chip select
            {
                SerialPrintln(F("SD init failed (second attempt). Is card present? Formatted?"));
                SerialPrintln(F("Please ensure the SD card is formatted correctly using https://www.sdcard.org/downloads/formatter/"));
                digitalWrite(PIN_MICROSD_CHIP_SELECT, HIGH); //Be sure SD is deselected
                online.microSD = false;
                return;
            }
        }

        //Change to root directory. All new file creation will be in root.
        if (sd.chdir() == false) {
            SerialPrintln(F("SD change directory failed"));
            online.microSD = false;
            return;
        }

        online.microSD = true;
    } else {
        microSDPowerOff();
        online.microSD = false;
    }
}

void enableCIPOpullUp() // updated for v2.1.0 of the Apollo3 core
{
    //Add 1K5 pull-up on CIPO
    am_hal_gpio_pincfg_t cipoPinCfg = g_AM_BSP_GPIO_IOM0_MISO;
    cipoPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_1_5K;
    pin_config(PinName(PIN_SPI_CIPO), cipoPinCfg);
}

void disableCIPOpullUp() // updated for v2.1.0 of the Apollo3 core
{
    am_hal_gpio_pincfg_t cipoPinCfg = g_AM_BSP_GPIO_IOM0_MISO;
    pin_config(PinName(PIN_SPI_CIPO), cipoPinCfg);
}

void configureSerial1TxRx(void) // Configure pins 12 and 13 for UART1 TX and RX
{
    am_hal_gpio_pincfg_t pinConfigTx = g_AM_BSP_GPIO_COM_UART_TX;
    pinConfigTx.uFuncSel = AM_HAL_PIN_12_UART1TX;
    pin_config(PinName(BREAKOUT_PIN_TX), pinConfigTx);
    am_hal_gpio_pincfg_t pinConfigRx = g_AM_BSP_GPIO_COM_UART_RX;
    pinConfigRx.uFuncSel = AM_HAL_PIN_13_UART1RX;
    pinConfigRx.ePullup = AM_HAL_GPIO_PIN_PULLUP_WEAK; // Put a weak pull-up on the Rx pin
    pin_config(PinName(BREAKOUT_PIN_RX), pinConfigRx);
}

void beginIMU() {
    pinMode(PIN_IMU_POWER, OUTPUT);
    pin_config(PinName(PIN_IMU_POWER), g_AM_HAL_GPIO_OUTPUT); // Make sure the pin does actually get re-configured
    pinMode(PIN_IMU_CHIP_SELECT, OUTPUT);
    pin_config(PinName(PIN_IMU_CHIP_SELECT), g_AM_HAL_GPIO_OUTPUT); // Make sure the pin does actually get re-configured
    digitalWrite(PIN_IMU_CHIP_SELECT, HIGH); //Be sure IMU is deselected

    if (settings.enableIMU == true && settings.logMaxRate == false) {
        //Reset ICM by power cycling it
        imuPowerOff();
        for (int i = 0; i < 10; i++) //10 is fine
        {
            checkBattery();
            delay(1);
        }
        imuPowerOn();
        for (int i = 0; i < 25; i++) //Allow ICM to come online. Typical is 11ms. Max is 100ms. https://cdn.sparkfun.com/assets/7/f/e/c/d/DS-000189-ICM-20948-v1.3.pdf
        {
            checkBattery();
            delay(1);
        }

        if (settings.printDebugMessages) myICM.enableDebugging();
        myICM.begin(PIN_IMU_CHIP_SELECT, SPI, 4000000); //Set IMU SPI rate to 4MHz
        if (myICM.status != ICM_20948_Stat_Ok) {
            printDebug("beginIMU: first attempt at myICM.begin failed. myICM.status = " + (String) myICM.status + "\r\n");
            //Try one more time with longer wait

            //Reset ICM by power cycling it
            imuPowerOff();
            for (int i = 0; i < 10; i++) //10 is fine
            {
                checkBattery();
                delay(1);
            }
            imuPowerOn();
            for (int i = 0; i < 100; i++) //Allow ICM to come online. Typical is 11ms. Max is 100ms.
            {
                checkBattery();
                delay(1);
            }

            myICM.begin(PIN_IMU_CHIP_SELECT, SPI, 4000000); //Set IMU SPI rate to 4MHz
            if (myICM.status != ICM_20948_Stat_Ok) {
                printDebug("beginIMU: second attempt at myICM.begin failed. myICM.status = " + (String) myICM.status + "\r\n");
                digitalWrite(PIN_IMU_CHIP_SELECT, HIGH); //Be sure IMU is deselected
                SerialPrintln(F("ICM-20948 failed to init."));
                imuPowerOff();
                online.IMU = false;
                return;
            }
        }

        //Give the IMU extra time to get its act together. This seems to fix the IMU-not-starting-up-cleanly-after-sleep problem...
        //Seems to need a full 25ms. 10ms is not enough.
        for (int i = 0; i < 25; i++) //Allow ICM to come online.
        {
            checkBattery();
            delay(1);
        }

        bool success = true;

        //Check if we are using the DMP
        if (settings.imuUseDMP == false) {
            //Perform a full startup (not minimal) for non-DMP mode
            ICM_20948_Status_e retval = myICM.startupDefault(false);
            if (retval != ICM_20948_Stat_Ok) {
                SerialPrintln(F("Error: Could not startup the IMU in non-DMP mode!"));
                success = false;
            }
            //Update the full scale and DLPF settings
            retval = myICM.enableDLPF(ICM_20948_Internal_Acc, settings.imuAccDLPF);
            if (retval != ICM_20948_Stat_Ok) {
                SerialPrintln(F("Error: Could not configure the IMU Accelerometer DLPF!"));
                success = false;
            }
            retval = myICM.enableDLPF(ICM_20948_Internal_Gyr, settings.imuGyroDLPF);
            if (retval != ICM_20948_Stat_Ok) {
                SerialPrintln(F("Error: Could not configure the IMU Gyro DLPF!"));
                success = false;
            }
            ICM_20948_dlpcfg_t dlpcfg;
            dlpcfg.a = settings.imuAccDLPFBW;
            dlpcfg.g = settings.imuGyroDLPFBW;
            retval = myICM.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), dlpcfg);
            if (retval != ICM_20948_Stat_Ok) {
                SerialPrintln(F("Error: Could not configure the IMU DLPF BW!"));
                success = false;
            }
            ICM_20948_fss_t FSS;
            FSS.a = settings.imuAccFSS;
            FSS.g = settings.imuGyroFSS;
            retval = myICM.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), FSS);
            if (retval != ICM_20948_Stat_Ok) {
                SerialPrintln(F("Error: Could not configure the IMU Full Scale!"));
                success = false;
            }
        } else {
            // Initialize the DMP
            ICM_20948_Status_e retval = myICM.initializeDMP();
            if (retval != ICM_20948_Stat_Ok) {
                SerialPrintln(F("Error: Could not startup the IMU in DMP mode!"));
                success = false;
            }
            if (settings.imuLogDMPQuat6) {
                retval = myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR);
                if (retval != ICM_20948_Stat_Ok) {
                    SerialPrintln(F("Error: Could not enable the Game Rotation Vector (Quat6)!"));
                    success = false;
                }
                retval = myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0); // Set ODR to 55Hz
                if (retval != ICM_20948_Stat_Ok) {
                    SerialPrintln(F("Error: Could not set the Quat6 ODR!"));
                    success = false;
                }
            }
            if (settings.imuLogDMPQuat9) {
                retval = myICM.enableDMPSensor(INV_ICM20948_SENSOR_ROTATION_VECTOR);
                if (retval != ICM_20948_Stat_Ok) {
                    SerialPrintln(F("Error: Could not enable the Rotation Vector (Quat9)!"));
                    success = false;
                }
                retval = myICM.setDMPODRrate(DMP_ODR_Reg_Quat9, 0); // Set ODR to 55Hz
                if (retval != ICM_20948_Stat_Ok) {
                    SerialPrintln(F("Error: Could not set the Quat9 ODR!"));
                    success = false;
                }
            }
            if (settings.imuLogDMPAccel) {
                retval = myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER);
                if (retval != ICM_20948_Stat_Ok) {
                    SerialPrintln(F("Error: Could not enable the DMP Accelerometer!"));
                    success = false;
                }
                retval = myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 0); // Set ODR to 55Hz
                if (retval != ICM_20948_Stat_Ok) {
                    SerialPrintln(F("Error: Could not set the Accel ODR!"));
                    success = false;
                }
            }
            if (settings.imuLogDMPGyro) {
                retval = myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE);
                if (retval != ICM_20948_Stat_Ok) {
                    SerialPrintln(F("Error: Could not enable the DMP Gyroscope!"));
                    success = false;
                }
                retval = myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 0); // Set ODR to 55Hz
                if (retval != ICM_20948_Stat_Ok) {
                    SerialPrintln(F("Error: Could not set the Gyro ODR!"));
                    success = false;
                }
                retval = myICM.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 0); // Set ODR to 55Hz
                if (retval != ICM_20948_Stat_Ok) {
                    SerialPrintln(F("Error: Could not set the Gyro Calibr ODR!"));
                    success = false;
                }
            }
            if (settings.imuLogDMPCpass) {
                retval = myICM.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED);
                if (retval != ICM_20948_Stat_Ok) {
                    SerialPrintln(F("Error: Could not enable the DMP Compass!"));
                    success = false;
                }
                retval = myICM.setDMPODRrate(DMP_ODR_Reg_Cpass, 0); // Set ODR to 55Hz
                if (retval != ICM_20948_Stat_Ok) {
                    SerialPrintln(F("Error: Could not set the Compass ODR!"));
                    success = false;
                }
                retval = myICM.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 0); // Set ODR to 55Hz
                if (retval != ICM_20948_Stat_Ok) {
                    SerialPrintln(F("Error: Could not set the Compass Calibr ODR!"));
                    success = false;
                }
            }
            retval = myICM.enableFIFO(); // Enable the FIFO
            if (retval != ICM_20948_Stat_Ok) {
                SerialPrintln(F("Error: Could not enable the FIFO!"));
                success = false;
            }
            retval = myICM.enableDMP(); // Enable the DMP
            if (retval != ICM_20948_Stat_Ok) {
                SerialPrintln(F("Error: Could not enable the DMP!"));
                success = false;
            }
            retval = myICM.resetDMP(); // Reset the DMP
            if (retval != ICM_20948_Stat_Ok) {
                SerialPrintln(F("Error: Could not reset the DMP!"));
                success = false;
            }
            retval = myICM.resetFIFO(); // Reset the FIFO
            if (retval != ICM_20948_Stat_Ok) {
                SerialPrintln(F("Error: Could not reset the FIFO!"));
                success = false;
            }
        }

        if (success) {
            online.IMU = true;
            delay(50); // Give the IMU time to get its first measurement ready
        } else {
            //Power down IMU
            imuPowerOff();
            online.IMU = false;
        }
    } else {
        //Power down IMU
        imuPowerOff();
        online.IMU = false;
    }
}


// initializeDMP is a weak function. Let's overwrite it so we can increase the sample rate
ICM_20948_Status_e ICM_20948::initializeDMP(void) {
    // First, let's check if the DMP is available
    if (_device._dmp_firmware_available != true) {
        debugPrint(F("ICM_20948::startupDMP: DMP is not available. Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
        return ICM_20948_Stat_DMPNotSupported;
    }

    ICM_20948_Status_e worstResult = ICM_20948_Stat_Ok;

#if defined(ICM_20948_USE_DMP)

    // The ICM-20948 is awake and ready but hasn't been configured. Let's step through the configuration
    // sequence from InvenSense's _confidential_ Application Note "Programming Sequence for DMP Hardware Functions".

    ICM_20948_Status_e result = ICM_20948_Stat_Ok; // Use result and worstResult to show if the configuration was successful
//    ICM_20948_Status_e  worstResult = ICM_20948_Stat_Ok;

    // Normally, when the DMP is not enabled, startupMagnetometer (called by startupDefault, which is called by begin) configures the AK09916 magnetometer
    // to run at 100Hz by setting the CNTL2 register (0x31) to 0x08. Then the ICM20948's I2C_SLV0 is configured to read
    // nine bytes from the mag every sample, starting from the STATUS1 register (0x10). ST1 includes the DRDY (Data Ready) bit.
    // Next are the six magnetometer readings (little endian). After a dummy byte, the STATUS2 register (0x18) contains the HOFL (Overflow) bit.
    //
    // But looking very closely at the InvenSense example code, we can see in inv_icm20948_resume_akm (in Icm20948AuxCompassAkm.c) that,
    // when the DMP is running, the magnetometer is set to Single Measurement (SM) mode and that ten bytes are read, starting from the reserved
    // RSV2 register (0x03). The datasheet does not define what registers 0x04 to 0x0C contain. There is definitely some secret sauce in here...
    // The magnetometer data appears to be big endian (not little endian like the HX/Y/Z registers) and starts at register 0x04.
    // We had to examine the I2C traffic between the master and the AK09916 on the AUX_DA and AUX_CL pins to discover this...
    //
    // So, we need to set up I2C_SLV0 to do the ten byte reading. The parameters passed to i2cControllerConfigurePeripheral are:
    // 0: use I2C_SLV0
    // MAG_AK09916_I2C_ADDR: the I2C address of the AK09916 magnetometer (0x0C unshifted)
    // AK09916_REG_RSV2: we start reading here (0x03). Secret sauce...
    // 10: we read 10 bytes each cycle
    // true: set the I2C_SLV0_RNW ReadNotWrite bit so we read the 10 bytes (not write them)
    // true: set the I2C_SLV0_CTRL I2C_SLV0_EN bit to enable reading from the peripheral at the sample rate
    // false: clear the I2C_SLV0_CTRL I2C_SLV0_REG_DIS (we want to write the register value)
    // true: set the I2C_SLV0_CTRL I2C_SLV0_GRP bit to show the register pairing starts at byte 1+2 (copied from inv_icm20948_resume_akm)
    // true: set the I2C_SLV0_CTRL I2C_SLV0_BYTE_SW to byte-swap the data from the mag (copied from inv_icm20948_resume_akm)
    result = i2cControllerConfigurePeripheral(0, MAG_AK09916_I2C_ADDR, AK09916_REG_RSV2, 10, true, true, false, true, true);
    if (result > worstResult) worstResult = result;
    //
    // We also need to set up I2C_SLV1 to do the Single Measurement triggering:
    // 1: use I2C_SLV1
    // MAG_AK09916_I2C_ADDR: the I2C address of the AK09916 magnetometer (0x0C unshifted)
    // AK09916_REG_CNTL2: we start writing here (0x31)
    // 1: not sure why, but the write does not happen if this is set to zero
    // false: clear the I2C_SLV0_RNW ReadNotWrite bit so we write the dataOut byte
    // true: set the I2C_SLV0_CTRL I2C_SLV0_EN bit. Not sure why, but the write does not happen if this is clear
    // false: clear the I2C_SLV0_CTRL I2C_SLV0_REG_DIS (we want to write the register value)
    // false: clear the I2C_SLV0_CTRL I2C_SLV0_GRP bit
    // false: clear the I2C_SLV0_CTRL I2C_SLV0_BYTE_SW bit
    // AK09916_mode_single: tell I2C_SLV1 to write the Single Measurement command each sample
    result = i2cControllerConfigurePeripheral(1, MAG_AK09916_I2C_ADDR, AK09916_REG_CNTL2, 1, false, true, false, false, false, AK09916_mode_single);
    if (result > worstResult) worstResult = result;

    // Set the I2C Master ODR configuration
    // It is not clear why we need to do this... But it appears to be essential! From the datasheet:
    // "I2C_MST_ODR_CONFIG[3:0]: ODR configuration for external sensor when gyroscope and accelerometer are disabled.
    //  ODR is computed as follows: 1.1 kHz/(2^((odr_config[3:0])) )
    //  When gyroscope is enabled, all sensors (including I2C_MASTER) use the gyroscope ODR.
    //  If gyroscope is disabled, then all sensors (including I2C_MASTER) use the accelerometer ODR."
    // Since both gyro and accel are running, setting this register should have no effect. But it does. Maybe because the Gyro and Accel are placed in Low Power Mode (cycled)?
    // You can see by monitoring the Aux I2C pins that the next three lines reduce the bus traffic (magnetometer reads) from 1125Hz to the chosen rate: 68.75Hz in this case.
    result = setBank(3);
    if (result > worstResult) worstResult = result; // Select Bank 3
    uint8_t mstODRconfig = 0x04; // Set the ODR configuration to 1100/2^4 = 68.75Hz
    result = write(AGB3_REG_I2C_MST_ODR_CONFIG, &mstODRconfig, 1);
    if (result > worstResult) worstResult = result; // Write one byte to the I2C_MST_ODR_CONFIG register

    // Configure clock source through PWR_MGMT_1
    // ICM_20948_Clock_Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
    result = setClockSource(ICM_20948_Clock_Auto);
    if (result > worstResult) worstResult = result; // This is shorthand: success will be set to false if setClockSource fails

    // Enable accel and gyro sensors through PWR_MGMT_2
    // Enable Accelerometer (all axes) and Gyroscope (all axes) by writing zero to PWR_MGMT_2
    result = setBank(0);
    if (result > worstResult) worstResult = result;                               // Select Bank 0
    uint8_t pwrMgmt2 = 0x40;                                                          // Set the reserved bit 6 (pressure sensor disable?)
    result = write(AGB0_REG_PWR_MGMT_2, &pwrMgmt2, 1);
    if (result > worstResult) worstResult = result; // Write one byte to the PWR_MGMT_2 register

    // Place _only_ I2C_Master in Low Power Mode (cycled) via LP_CONFIG
    // The InvenSense Nucleo example initially puts the accel and gyro into low power mode too, but then later updates LP_CONFIG so only the I2C_Master is in Low Power Mode
    result = setSampleMode(ICM_20948_Internal_Mst, ICM_20948_Sample_Mode_Cycled);
    if (result > worstResult) worstResult = result;

    // Disable the FIFO
    result = enableFIFO(false);
    if (result > worstResult) worstResult = result;

    // Disable the DMP
    result = enableDMP(false);
    if (result > worstResult) worstResult = result;

    // Set Gyro FSR (Full scale range) to 2000dps through GYRO_CONFIG_1
    // Set Accel FSR (Full scale range) to 4g through ACCEL_CONFIG
    ICM_20948_fss_t myFSS; // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors
    myFSS.a = gpm8;        // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
    // gpm2
    // gpm4
    // gpm8
    // gpm16
    myFSS.g = dps2000;     // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
    // dps250
    // dps500
    // dps1000
    // dps2000
    result = setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);
    if (result > worstResult) worstResult = result;

    // The InvenSense Nucleo code also enables the gyro DLPF (but leaves GYRO_DLPFCFG set to zero = 196.6Hz (3dB))
    // We found this by going through the SPI data generated by ZaneL's Teensy-ICM-20948 library byte by byte...
    // The gyro DLPF is enabled by default (GYRO_CONFIG_1 = 0x01) so the following line should have no effect, but we'll include it anyway
    result = enableDLPF(ICM_20948_Internal_Gyr, true);
    if (result > worstResult) worstResult = result;

    // Enable interrupt for FIFO overflow from FIFOs through INT_ENABLE_2
    // If we see this interrupt, we'll need to reset the FIFO
    //result = intEnableOverflowFIFO( 0x1F ); if (result > worstResult) worstResult = result; // Enable the interrupt on all FIFOs

    // Turn off what goes into the FIFO through FIFO_EN_1, FIFO_EN_2
    // Stop the peripheral data from being written to the FIFO by writing zero to FIFO_EN_1
    result = setBank(0);
    if (result > worstResult) worstResult = result; // Select Bank 0
    uint8_t zero = 0;
    result = write(AGB0_REG_FIFO_EN_1, &zero, 1);
    if (result > worstResult) worstResult = result;
    // Stop the accelerometer, gyro and temperature data from being written to the FIFO by writing zero to FIFO_EN_2
    result = write(AGB0_REG_FIFO_EN_2, &zero, 1);
    if (result > worstResult) worstResult = result;

    // Turn off data ready interrupt through INT_ENABLE_1
    result = intEnableRawDataReady(false);
    if (result > worstResult) worstResult = result;

    // Reset FIFO through FIFO_RST
    result = resetFIFO();
    if (result > worstResult) worstResult = result;

    // Set gyro sample rate divider with GYRO_SMPLRT_DIV
    // Set accel sample rate divider with ACCEL_SMPLRT_DIV_2
    ICM_20948_smplrt_t mySmplrt;
    //mySmplrt.g = 19; // ODR is computed as follows: 1.1 kHz/(1+GYRO_SMPLRT_DIV[7:0]). 19 = 55Hz. InvenSense Nucleo example uses 19 (0x13).
    //mySmplrt.a = 19; // ODR is computed as follows: 1.125 kHz/(1+ACCEL_SMPLRT_DIV[11:0]). 19 = 56.25Hz. InvenSense Nucleo example uses 19 (0x13).
    mySmplrt.g = 4; // 225Hz
    mySmplrt.a = 4; // 225Hz
    //mySmplrt.g = 8; // 112Hz
    //mySmplrt.a = 8; // 112Hz
    result = setSampleRate((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), mySmplrt);
    if (result > worstResult) worstResult = result;

    // Setup DMP start address through PRGM_STRT_ADDRH/PRGM_STRT_ADDRL
    result = setDMPstartAddress();
    if (result > worstResult) worstResult = result; // Defaults to DMP_START_ADDRESS

    // Now load the DMP firmware
    result = loadDMPFirmware();
    if (result > worstResult) worstResult = result;

    // Write the 2 byte Firmware Start Value to ICM PRGM_STRT_ADDRH/PRGM_STRT_ADDRL
    result = setDMPstartAddress();
    if (result > worstResult) worstResult = result; // Defaults to DMP_START_ADDRESS

    // Set the Hardware Fix Disable register to 0x48
    result = setBank(0);
    if (result > worstResult) worstResult = result; // Select Bank 0
    uint8_t fix = 0x48;
    result = write(AGB0_REG_HW_FIX_DISABLE, &fix, 1);
    if (result > worstResult) worstResult = result;

    // Set the Single FIFO Priority Select register to 0xE4
    result = setBank(0);
    if (result > worstResult) worstResult = result; // Select Bank 0
    uint8_t fifoPrio = 0xE4;
    result = write(AGB0_REG_SINGLE_FIFO_PRIORITY_SEL, &fifoPrio, 1);
    if (result > worstResult) worstResult = result;

    // Configure Accel scaling to DMP
    // The DMP scales accel raw data internally to align 1g as 2^25
    // In order to align internal accel raw data 2^25 = 1g write 0x04000000 when FSR is 4g
    if (myFSS.a != gpm8) { // update this if you do change the scaling factor
        while (true) {
            Serial.println("!!!!---------------- DMP accel scaling needs to be updated, right now its configured for gpm8 ----------!!!!");
            delay(1000);
        }
    }
    const unsigned char accScale[4] = {0x08, 0x00, 0x00, 0x00};
    result = writeDMPmems(ACC_SCALE, 4, &accScale[0]);
    if (result > worstResult) worstResult = result; // Write accScale to ACC_SCALE DMP register
    // In order to output hardware unit data as configured FSR write 0x00040000 when FSR is 4g
    const unsigned char accScale2[4] = {0x00, 0x08, 0x00, 0x00};
    result = writeDMPmems(ACC_SCALE2, 4, &accScale2[0]);
    if (result > worstResult) worstResult = result; // Write accScale2 to ACC_SCALE2 DMP register

    // Configure Compass mount matrix and scale to DMP
    // The mount matrix write to DMP register is used to align the compass axes with accel/gyro.
    // This mechanism is also used to convert hardware unit to uT. The value is expressed as 1uT = 2^30.
    // Each compass axis will be converted as below:
    // X = raw_x * CPASS_MTX_00 + raw_y * CPASS_MTX_01 + raw_z * CPASS_MTX_02
    // Y = raw_x * CPASS_MTX_10 + raw_y * CPASS_MTX_11 + raw_z * CPASS_MTX_12
    // Z = raw_x * CPASS_MTX_20 + raw_y * CPASS_MTX_21 + raw_z * CPASS_MTX_22
    // The AK09916 produces a 16-bit signed output in the range +/-32752 corresponding to +/-4912uT. 1uT = 6.66 ADU.
    // 2^30 / 6.66666 = 161061273 = 0x9999999
    const unsigned char mountMultiplierZero[4] = {0x00, 0x00, 0x00, 0x00};
    const unsigned char mountMultiplierPlus[4] = {0x09, 0x99, 0x99, 0x99};  // Value taken from InvenSense Nucleo example
    const unsigned char mountMultiplierMinus[4] = {0xF6, 0x66, 0x66, 0x67}; // Value taken from InvenSense Nucleo example
    result = writeDMPmems(CPASS_MTX_00, 4, &mountMultiplierPlus[0]);
    if (result > worstResult) worstResult = result;
    result = writeDMPmems(CPASS_MTX_01, 4, &mountMultiplierZero[0]);
    if (result > worstResult) worstResult = result;
    result = writeDMPmems(CPASS_MTX_02, 4, &mountMultiplierZero[0]);
    if (result > worstResult) worstResult = result;
    result = writeDMPmems(CPASS_MTX_10, 4, &mountMultiplierZero[0]);
    if (result > worstResult) worstResult = result;
    result = writeDMPmems(CPASS_MTX_11, 4, &mountMultiplierMinus[0]);
    if (result > worstResult) worstResult = result;
    result = writeDMPmems(CPASS_MTX_12, 4, &mountMultiplierZero[0]);
    if (result > worstResult) worstResult = result;
    result = writeDMPmems(CPASS_MTX_20, 4, &mountMultiplierZero[0]);
    if (result > worstResult) worstResult = result;
    result = writeDMPmems(CPASS_MTX_21, 4, &mountMultiplierZero[0]);
    if (result > worstResult) worstResult = result;
    result = writeDMPmems(CPASS_MTX_22, 4, &mountMultiplierMinus[0]);
    if (result > worstResult) worstResult = result;

    // Configure the B2S Mounting Matrix
    const unsigned char b2sMountMultiplierZero[4] = {0x00, 0x00, 0x00, 0x00};
    const unsigned char b2sMountMultiplierPlus[4] = {0x40, 0x00, 0x00, 0x00}; // Value taken from InvenSense Nucleo example
    result = writeDMPmems(B2S_MTX_00, 4, &b2sMountMultiplierPlus[0]);
    if (result > worstResult) worstResult = result;
    result = writeDMPmems(B2S_MTX_01, 4, &b2sMountMultiplierZero[0]);
    if (result > worstResult) worstResult = result;
    result = writeDMPmems(B2S_MTX_02, 4, &b2sMountMultiplierZero[0]);
    if (result > worstResult) worstResult = result;
    result = writeDMPmems(B2S_MTX_10, 4, &b2sMountMultiplierZero[0]);
    if (result > worstResult) worstResult = result;
    result = writeDMPmems(B2S_MTX_11, 4, &b2sMountMultiplierPlus[0]);
    if (result > worstResult) worstResult = result;
    result = writeDMPmems(B2S_MTX_12, 4, &b2sMountMultiplierZero[0]);
    if (result > worstResult) worstResult = result;
    result = writeDMPmems(B2S_MTX_20, 4, &b2sMountMultiplierZero[0]);
    if (result > worstResult) worstResult = result;
    result = writeDMPmems(B2S_MTX_21, 4, &b2sMountMultiplierZero[0]);
    if (result > worstResult) worstResult = result;
    result = writeDMPmems(B2S_MTX_22, 4, &b2sMountMultiplierPlus[0]);
    if (result > worstResult) worstResult = result;

    // Configure the DMP Gyro Scaling Factor
    // @param[in] gyro_div Value written to GYRO_SMPLRT_DIV register, where
    //            0=1125Hz sample rate, 1=562.5Hz sample rate, ... 4=225Hz sample rate, ...
    //            10=102.2727Hz sample rate, ... etc.
    // @param[in] gyro_level 0=250 dps, 1=500 dps, 2=1000 dps, 3=2000 dps
    result = setGyroSF(4, 3);
    if (result > worstResult) worstResult = result; // 4 = 225Hz (see above), 3 = 2000dps (see above)

    // Configure the Gyro full scale
    // 2000dps : 2^28
    // 1000dps : 2^27
    //  500dps : 2^26
    //  250dps : 2^25
    if (myFSS.g != dps2000) { // update this if you do change the scaling factor
        while (true) {
            Serial.println("!!!!---------------- DMP dps scaling needs to be updated, right now its configured for dps2000 ----------!!!!");
            delay(1000);
        }
    }
    const unsigned char gyroFullScale[4] = {0x10, 0x00, 0x00, 0x00}; // 2000dps : 2^28
    result = writeDMPmems(GYRO_FULLSCALE, 4, &gyroFullScale[0]);
    if (result > worstResult) worstResult = result;

    // Configure the Accel Only Gain: 15252014 (225Hz) 30504029 (112Hz) 61117001 (56Hz)
    //const unsigned char accelOnlyGain[4] = {0x03, 0xA4, 0x92, 0x49}; // 56Hz
    const unsigned char accelOnlyGain[4] = {0x00, 0xE8, 0xBA, 0x2E}; // 225Hz
    //const unsigned char accelOnlyGain[4] = {0x01, 0xD1, 0x74, 0x5D}; // 112Hz
    result = writeDMPmems(ACCEL_ONLY_GAIN, 4, &accelOnlyGain[0]);
    if (result > worstResult) worstResult = result;

    // Configure the Accel Alpha Var: 1026019965 (225Hz) 977872018 (112Hz) 882002213 (56Hz)
    //const unsigned char accelAlphaVar[4] = {0x34, 0x92, 0x49, 0x25}; // 56Hz
    const unsigned char accelAlphaVar[4] = {0x3D, 0x27, 0xD2, 0x7D}; // 225Hz
    //const unsigned char accelAlphaVar[4] = {0x3A, 0x49, 0x24, 0x92}; // 112Hz
    result = writeDMPmems(ACCEL_ALPHA_VAR, 4, &accelAlphaVar[0]);
    if (result > worstResult) worstResult = result;

    // Configure the Accel A Var: 47721859 (225Hz) 95869806 (112Hz) 191739611 (56Hz)
    //const unsigned char accelAVar[4] = {0x0B, 0x6D, 0xB6, 0xDB}; // 56Hz
    const unsigned char accelAVar[4] = {0x02, 0xD8, 0x2D, 0x83}; // 225Hz
    //const unsigned char accelAVar[4] = {0x05, 0xB6, 0xDB, 0x6E}; // 112Hz
    result = writeDMPmems(ACCEL_A_VAR, 4, &accelAVar[0]);
    if (result > worstResult) worstResult = result;

    // Configure the Accel Cal Rate
    const unsigned char accelCalRate[4] = {0x00, 0x00}; // Value taken from InvenSense Nucleo example
    result = writeDMPmems(ACCEL_CAL_RATE, 2, &accelCalRate[0]);
    if (result > worstResult) worstResult = result;

    // Configure the Compass Time Buffer. The I2C Master ODR Configuration (see above) sets the magnetometer read rate to 68.75Hz.
    // Let's set the Compass Time Buffer to 69 (Hz).
    const unsigned char compassRate[2] = {0x00, 0x45}; // 69Hz
    result = writeDMPmems(CPASS_TIME_BUFFER, 2, &compassRate[0]);
    if (result > worstResult) worstResult = result;

    // Enable DMP interrupt
    // This would be the most efficient way of getting the DMP data, instead of polling the FIFO
    //result = intEnableDMP(true); if (result > worstResult) worstResult = result;

#endif

    return worstResult;
}


void beginDataLogging() {
    if (online.microSD == true && settings.logData == true) {
        //If we don't have a file yet, create one. Otherwise, re-open the last used file
        if (strlen(sensorDataFileName) == 0)
            strcpy(sensorDataFileName, findNextAvailableLog(settings.nextDataLogNumber, "dataLog"));

        // O_CREAT - create the file if it does not exist
        // O_APPEND - seek to the end of the file prior to each write
        // O_WRITE - open for write
        if (sensorDataFile.open(sensorDataFileName, O_CREAT | O_APPEND | O_WRITE) == false) {
            SerialPrintln(F("Failed to create sensor data file"));
            online.dataLogging = false;
            return;
        }

        updateDataFileCreate(&sensorDataFile); // Update the file create time & date

        online.dataLogging = true;
    } else
        online.dataLogging = false;
}

void beginSerialLogging() {
    if (online.microSD == true && settings.logSerial == true) {
        //If we don't have a file yet, create one. Otherwise, re-open the last used file
        if (strlen(serialDataFileName) == 0)
            strcpy(serialDataFileName, findNextAvailableLog(settings.nextSerialLogNumber, "serialLog"));

        if (serialDataFile.open(serialDataFileName, O_CREAT | O_APPEND | O_WRITE) == false) {
            SerialPrintln(F("Failed to create serial log file"));
            //systemError(ERROR_FILE_OPEN);
            online.serialLogging = false;
            return;
        }

        updateDataFileCreate(&serialDataFile); // Update the file create time & date

        //We need to manually restore the Serial1 TX and RX pins
        configureSerial1TxRx();

        Serial1.begin(settings.serialLogBaudRate);

        online.serialLogging = true;
    } else
        online.serialLogging = false;
}

void beginSerialOutput() {
    if (settings.outputSerial == true) {
        //We need to manually restore the Serial1 TX and RX pins
        configureSerial1TxRx();

        Serial1.begin(settings.serialLogBaudRate); // (Re)start the serial port
        online.serialOutput = true;
    } else
        online.serialOutput = false;
}

void updateDataFileCreate(SdFileType *dataFile) {
    myRTC.getTime(); //Get the RTC time so we can use it to update the create time
    //Update the file create time
    dataFile->timestamp(T_CREATE, (myRTC.year + 2000), myRTC.month, myRTC.dayOfMonth, myRTC.hour, myRTC.minute, myRTC.seconds);
}

void updateDataFileAccess(SdFileType *dataFile) {
    myRTC.getTime(); //Get the RTC time so we can use it to update the last modified time
    //Update the file access time
    dataFile->timestamp(T_ACCESS, (myRTC.year + 2000), myRTC.month, myRTC.dayOfMonth, myRTC.hour, myRTC.minute, myRTC.seconds);
    //Update the file write time
    dataFile->timestamp(T_WRITE, (myRTC.year + 2000), myRTC.month, myRTC.dayOfMonth, myRTC.hour, myRTC.minute, myRTC.seconds);
}

//Called once number of milliseconds has passed
extern "C" void am_stimer_cmpr6_isr(void) {
    uint32_t ui32Status = am_hal_stimer_int_status_get(false);
    if (ui32Status & AM_HAL_STIMER_INT_COMPAREG) {
        am_hal_stimer_int_clear(AM_HAL_STIMER_INT_COMPAREG);
    }
}

//Power Loss ISR
void powerLossISR(void) {
    powerLossSeen = true;
}

//Stop Logging ISR
void stopLoggingISR(void) {
    stopLoggingSeen = true;
}

//Trigger Pin ISR
void triggerPinISR(void) {
    triggerEdgeSeen = true;
}

void SerialFlush(void) {
    Serial.flush();
    if (settings.useTxRxPinsForTerminal == true) {
        Serial1.flush();
    }
}

// gfvalvo's flash string helper code: https://forum.arduino.cc/index.php?topic=533118.msg3634809#msg3634809

void SerialPrint(const char *line) {
    DoSerialPrint([](const char *ptr) { return *ptr; }, line);
}

void SerialPrint(const __FlashStringHelper *line) {
    DoSerialPrint([](const char *ptr) { return (char) pgm_read_byte_near(ptr); }, (const char *) line);
}

void SerialPrintln(const char *line) {
    DoSerialPrint([](const char *ptr) { return *ptr; }, line, true);
}

void SerialPrintln(const __FlashStringHelper *line) {
    DoSerialPrint([](const char *ptr) { return (char) pgm_read_byte_near(ptr); }, (const char *) line, true);
}

void DoSerialPrint(char (*funct)(const char *), const char *string, bool newLine) {
    char ch;

    while ((ch = funct(string++))) {
        Serial.print(ch);
        if (settings.useTxRxPinsForTerminal == true)
            Serial1.print(ch);
    }

    if (newLine) {
        Serial.println();
        if (settings.useTxRxPinsForTerminal == true)
            Serial1.println();
    }
}
