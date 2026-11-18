#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <time.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <DHT.h>

#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"

// ============================================================
//                  SWASTHYASATHI WATCH OS
//                    COMPLETE BLE EDITION
// ============================================================
//
// Hardware:
//   ESP32-S3 DevKitC-1
//   SH1106 128x64 OLED
//   MAX30102
//   DHT22
//   NEO GPS
//   Touch sensor
//
// Navigation:
//   SHORT TAP  -> NEXT SCREEN
//   HOLD 3 SEC -> SOS
//
// Screens:
//   HOME -> HEALTH -> ENVIRONMENT -> LOCATION
//
// BLE:
//   Device: SWASTHYASATHI WATCH
//   Sends live health/environment/location telemetry.
//   Sends SOS packet when the user holds touch for 3 seconds.
//
// IMPORTANT:
//   SpO2 from MAX30102 is a prototype estimate and is NOT
//   medical-grade. Battery ADC also needs hardware calibration.
// ============================================================


// ============================================================
//                         WIFI
// ============================================================

const char* WIFI_SSID = "Redmi 13C 5G";
const char* WIFI_PASSWORD = "1234567890";

const long GMT_OFFSET_SEC = 19800;       // India Standard Time
const int DAYLIGHT_OFFSET_SEC = 0;

const char* NTP_SERVER_1 = "pool.ntp.org";
const char* NTP_SERVER_2 = "time.nist.gov";


// ============================================================
//                          PINS
// ============================================================

// OLED
#define OLED_SDA 4
#define OLED_SCL 5

// MAX30102
#define MAX_SDA 8
#define MAX_SCL 9

// DHT22
#define DHT_PIN 1

// GPS
#define GPS_TX 17
#define GPS_RX 18

// Battery ADC
#define BATTERY_PIN 12

// Touch sensor
#define TOUCH_PIN 45


// ============================================================
//                           OLED
// ============================================================

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SH1106G display(
    SCREEN_WIDTH,
    SCREEN_HEIGHT,
    &Wire,
    -1
);


// ============================================================
//                           DHT22
// ============================================================

#define DHT_TYPE DHT22

DHT dht(DHT_PIN, DHT_TYPE);

float temperatureC = NAN;
float humidity = NAN;

unsigned long lastDHTRead = 0;
const unsigned long DHT_INTERVAL = 2000;


// ============================================================
//                         MAX30102
// ============================================================

MAX30105 particleSensor;

#define SPO2_BUFFER_SIZE 100

uint32_t irBuffer[SPO2_BUFFER_SIZE];
uint32_t redBuffer[SPO2_BUFFER_SIZE];

int32_t spo2 = 0;
int8_t validSPO2 = 0;

int32_t algorithmHeartRate = 0;
int8_t validHeartRate = 0;

int spo2SampleCount = 0;

int displayedSpO2 = 0;
bool spo2Available = false;

bool fingerDetected = false;

// ------------------------------------------------------------
// Stable HR measurement state
// ------------------------------------------------------------

unsigned long lastBeat = 0;

float beatsPerMinute = 0.0;
int beatAvg = 0;

// MAX30102 finger detection threshold.
// Keep this low enough to work with different breakout boards.
const uint32_t IR_THRESHOLD = 500;

// Recent BPM values are averaged to remove single-beat spikes.
#define HR_HISTORY_SIZE 5

float hrHistory[HR_HISTORY_SIZE];
int hrHistoryCount = 0;
int hrHistoryIndex = 0;

// Require two valid intervals before publishing HR.
const int MIN_VALID_BEATS = 2;

// A beat interval is valid from 300-1500 ms
// (40-200 BPM).
const unsigned long MIN_IBI = 300;
const unsigned long MAX_IBI = 1500;

// If no beat is detected for this long, invalidate the HR.
const unsigned long HR_TIMEOUT = 5000;

// Short startup period to avoid startup noise.
unsigned long fingerStartTime = 0;
const unsigned long HR_WARMUP_TIME = 500;

// ------------------------------------------------------------
// ONE-MINUTE HEART-RATE AVERAGE
// ------------------------------------------------------------

unsigned long hrMinuteStart = 0;
unsigned long hrMinuteBpmSum = 0;
unsigned int hrMinuteBpmCount = 0;

int oneMinuteAverageHR = 0;
bool oneMinuteAverageValid = false;


// ============================================================
//                           GPS
// ============================================================

HardwareSerial GPS(1);

bool gpsConnected = false;
int gpsSatellites = 0;

double gpsLatitude = 0.0;
double gpsLongitude = 0.0;
bool gpsLocationValid = false;

String gpsLine;


// ============================================================
//                         BATTERY
// ============================================================

float batteryVoltage = 0.0;
int batteryPercent = 0;

unsigned long lastBatteryRead = 0;
const unsigned long BATTERY_INTERVAL = 3000;


// ============================================================
//                           BLE
// ============================================================

BLEServer* bleServer = nullptr;
BLEService* bleService = nullptr;

BLECharacteristic* bleStatusCharacteristic = nullptr;
BLECharacteristic* bleTelemetryCharacteristic = nullptr;
BLECharacteristic* bleCommandCharacteristic = nullptr;

bool phoneConnected = false;
bool bleDeviceConnected = false;

unsigned long lastBLETelemetry = 0;
const unsigned long BLE_TELEMETRY_INTERVAL = 2000;

// Custom UUIDs for SwasthyaSathi.
#define SWASTHYASATHI_SERVICE_UUID \
"7c8e0001-6b7a-4a91-9c21-6d7f4a100001"

#define STATUS_CHARACTERISTIC_UUID \
"7c8e0002-6b7a-4a91-9c21-6d7f4a100002"

#define TELEMETRY_CHARACTERISTIC_UUID \
"7c8e0003-6b7a-4a91-9c21-6d7f4a100003"

#define COMMAND_CHARACTERISTIC_UUID \
"7c8e0004-6b7a-4a91-9c21-6d7f4a100004"


// ============================================================
//                          SCREENS
// ============================================================

enum Screen
{
    HOME_SCREEN = 0,
    HEALTH_SCREEN,
    ENVIRONMENT_SCREEN,
    LOCATION_SCREEN
};

Screen currentScreen = HOME_SCREEN;
Screen previousScreen = HOME_SCREEN;

void drawCurrentScreen();

// Forward declarations for health display helpers.
// These helpers are implemented later in the file, but BLE telemetry
// is built before the OLED drawing functions.
int getDisplayHeartRate();
bool hasDisplayHeartRate();
int getDisplaySpO2();


// ============================================================
//                       TOUCH / SOS
// ============================================================

bool touchPressed = false;
bool longPressTriggered = false;

unsigned long touchStartTime = 0;
unsigned long lastTouchRelease = 0;

const unsigned long SOS_HOLD_TIME = 3000;
const unsigned long SOS_PREVIEW_TIME = 600;
const unsigned long TOUCH_DEBOUNCE = 250;


// ============================================================
//                         SOS STATE
// ============================================================

bool sosMessageActive = false;
bool sosWasSent = false;

unsigned long sosMessageStart = 0;

const unsigned long SOS_MESSAGE_DURATION = 3500;


// ============================================================
//                       ANIMATIONS
// ============================================================

unsigned long animationFrame = 0;
unsigned long lastAnimationFrame = 0;

const unsigned long ANIMATION_INTERVAL = 120;


// ============================================================
//                       BOOT ANIMATION
// ============================================================

void bootAnimation()
{
    display.clearDisplay();
    display.display();
    delay(150);

    for (int r = 2; r <= 25; r += 2)
    {
        display.clearDisplay();

        display.drawCircle(
            64,
            22,
            r,
            SH110X_WHITE
        );

        display.display();
        delay(20);
    }

    display.clearDisplay();

    display.drawCircle(
        64,
        22,
        25,
        SH110X_WHITE
    );

    int points[][2] =
    {
        {32, 23},
        {40, 23},
        {44, 18},
        {48, 30},
        {53, 10},
        {58, 35},
        {63, 23},
        {70, 23},
        {76, 19},
        {82, 23},
        {96, 23}
    };

    for (int i = 0; i < 10; i++)
    {
        display.drawLine(
            points[i][0],
            points[i][1],
            points[i + 1][0],
            points[i + 1][1],
            SH110X_WHITE
        );

        display.display();
        delay(35);
    }

    display.setTextSize(1);

    display.setCursor(25, 45);
    display.print("SWASTHYASATHI");

    display.setCursor(39, 56);
    display.print("WATCH OS");

    display.display();
    delay(700);
}


// ============================================================
//                       WIFI CONNECTION
// ============================================================

void connectWiFi()
{
    Serial.println();
    Serial.println("Connecting to WiFi...");

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    int attempts = 0;

    while (
        WiFi.status() != WL_CONNECTED &&
        attempts < 30
    )
    {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    Serial.println();

    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.println("WiFi CONNECTED!");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
    }
    else
    {
        Serial.println("WiFi CONNECTION FAILED!");
        Serial.println("Watch will continue offline.");
    }
}


// ============================================================
//                            TIME
// ============================================================

void setupTime()
{
    Serial.println("Synchronizing Indian Standard Time...");

    configTime(
        GMT_OFFSET_SEC,
        DAYLIGHT_OFFSET_SEC,
        NTP_SERVER_1,
        NTP_SERVER_2
    );

    struct tm timeinfo;
    int attempts = 0;

    while (
        !getLocalTime(&timeinfo) &&
        attempts < 20
    )
    {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    Serial.println();

    if (attempts < 20)
    {
        Serial.println("TIME SYNCHRONIZED!");
    }
    else
    {
        Serial.println("TIME SYNCHRONIZATION FAILED!");
    }
}


// ============================================================
//                       MAX30102 SETUP
// ============================================================

void setupMAX30102()
{
    Serial.println();
    Serial.println("Starting MAX30102...");

    Wire1.begin(
        MAX_SDA,
        MAX_SCL
    );

    if (
        !particleSensor.begin(
            Wire1,
            I2C_SPEED_FAST
        )
    )
    {
        Serial.println("MAX30102 NOT FOUND!");

        while (true)
        {
            delay(1000);
        }
    }

    Serial.println("MAX30102 FOUND!");

    // Stable MAX30102 configuration.
    // Use the library's normal 4-sample averaging so the
    // beat detector receives a cleaner pulse waveform.
    particleSensor.setup(
        0x7F,     // LED brightness
        4,        // sample averaging
        2,        // Red + IR
        100,      // sample rate
        411,      // pulse width
        4096      // ADC range
    );

    // Stronger optical signal for the small watch sensor.
    particleSensor.setPulseAmplitudeRed(0x7F);
    particleSensor.setPulseAmplitudeIR(0x7F);

    Serial.println("MAX30102 READY!");
    Serial.println("HR: beat interval + 5-reading smoothing + 60-second average");
    Serial.println("SpO2: MAXIM algorithm");
    Serial.println("OLED HR source: 1-min avg -> beat HR -> MAXIM HR");
    Serial.println("OLED SpO2 source: valid MAXIM SpO2");
    Serial.println("Serial will print raw IR/RED values.");
    Serial.println("Place finger firmly on sensor and keep still.");
}


// ============================================================
//                     RESET SPO2 BUFFER
// ============================================================

void resetSpO2Buffer()
{
    spo2SampleCount = 0;
    spo2Available = false;
    validSPO2 = 0;
    validHeartRate = 0;
}

// ------------------------------------------------------------
// HR HELPERS
// ------------------------------------------------------------

void resetHeartRate()
{
    lastBeat = 0;
    beatsPerMinute = 0.0;
    beatAvg = 0;

    hrHistoryCount = 0;
    hrHistoryIndex = 0;

    for (int i = 0; i < HR_HISTORY_SIZE; i++)
    {
        hrHistory[i] = 0.0;
    }

    hrMinuteStart = millis();
    hrMinuteBpmSum = 0;
    hrMinuteBpmCount = 0;
    oneMinuteAverageHR = 0;
    oneMinuteAverageValid = false;

    fingerStartTime = millis();
}

void addIBI(unsigned long ibi)
{
    if (ibi < MIN_IBI || ibi > MAX_IBI)
    {
        Serial.print("HR: ignored IBI = ");
        Serial.print(ibi);
        Serial.println(" ms");
        return;
    }

    float instantBPM =
        60000.0 / (float)ibi;

    Serial.print("BEAT: IBI=");
    Serial.print(ibi);
    Serial.print(" ms, BPM=");
    Serial.println(instantBPM, 1);

    // --------------------------------------------------------
    // Add this valid beat to the current 60-second window.
    // --------------------------------------------------------

    hrMinuteBpmSum += (unsigned long)round(instantBPM);
    hrMinuteBpmCount++;

    if (hrMinuteBpmCount > 0)
    {
        oneMinuteAverageHR =
            (int)round(
                (double)hrMinuteBpmSum /
                (double)hrMinuteBpmCount
            );

        if (
            oneMinuteAverageHR >= 40 &&
            oneMinuteAverageHR <= 200
        )
        {
            oneMinuteAverageValid = true;
        }
    }

    // --------------------------------------------------------
    // Store valid BPM.
    // --------------------------------------------------------

    hrHistory[hrHistoryIndex] = instantBPM;

    hrHistoryIndex =
        (hrHistoryIndex + 1) % HR_HISTORY_SIZE;

    if (hrHistoryCount < HR_HISTORY_SIZE)
    {
        hrHistoryCount++;
    }

    if (hrHistoryCount >= MIN_VALID_BEATS)
    {
        // Average the recent valid BPM readings.
        float sum = 0.0;

        for (int i = 0; i < hrHistoryCount; i++)
        {
            sum += hrHistory[i];
        }

        float averageBPM =
            sum / (float)hrHistoryCount;

        if (
            averageBPM >= 40.0 &&
            averageBPM <= 200.0
        )
        {
            beatsPerMinute = averageBPM;
            beatAvg = (int)round(averageBPM);

            Serial.print("SMOOTH HR = ");
            Serial.print(beatAvg);
            Serial.print(" BPM | 1-MIN AVG = ");

            if (oneMinuteAverageValid)
            {
                Serial.print(oneMinuteAverageHR);
                Serial.println(" BPM");
            }
            else
            {
                Serial.println("-- BPM");
            }
        }
    }
}

// ============================================================
//                    READ MAX30102
// ============================================================

void readMAX30102()
{
    particleSensor.check();

    while (particleSensor.available())
    {
        uint32_t redValue = particleSensor.getRed();
        uint32_t irValue = particleSensor.getIR();

        // --------------------------------------------------------
        // DEBUG RAW SENSOR SIGNAL
        // --------------------------------------------------------

        static unsigned long lastIRPrint = 0;

        if (millis() - lastIRPrint >= 250)
        {
            lastIRPrint = millis();

            Serial.print("IR=");
            Serial.print(irValue);
            Serial.print(" RED=");
            Serial.println(redValue);
        }

        // --------------------------------------------------------
        // FINGER DETECTION
        // --------------------------------------------------------

        if (irValue > IR_THRESHOLD)
        {
            if (!fingerDetected)
            {
                fingerDetected = true;

                resetHeartRate();

                Serial.println();
                Serial.println("==============================");
                Serial.println("MAX30102 FINGER DETECTED");
                Serial.println("Waiting for pulse...");
                Serial.println("==============================");
            }

            // ----------------------------------------------------
            // HEART RATE
            // ----------------------------------------------------

            if (
                millis() - fingerStartTime >= HR_WARMUP_TIME &&
                checkForBeat(irValue)
            )
            {
                unsigned long currentTime = millis();

                if (lastBeat != 0)
                {
                    unsigned long ibi =
                        currentTime - lastBeat;

                    addIBI(ibi);
                }

                lastBeat = currentTime;
            }

            // ----------------------------------------------------
            // SpO2 BUFFER
            // ----------------------------------------------------

            if (spo2SampleCount < SPO2_BUFFER_SIZE)
            {
                redBuffer[spo2SampleCount] = redValue;
                irBuffer[spo2SampleCount] = irValue;
                spo2SampleCount++;
            }

            // ----------------------------------------------------
            // SpO2 ONLY
            // ----------------------------------------------------

            if (spo2SampleCount >= SPO2_BUFFER_SIZE)
            {
                maxim_heart_rate_and_oxygen_saturation(
                    irBuffer,
                    SPO2_BUFFER_SIZE,
                    redBuffer,
                    &spo2,
                    &validSPO2,
                    &algorithmHeartRate,
                    &validHeartRate
                );

                if (
                    validSPO2 &&
                    spo2 >= 70 &&
                    spo2 <= 100
                )
                {
                    displayedSpO2 = (int)spo2;
                    spo2Available = true;
                }
                else
                {
                    spo2Available = false;
                }

                // If the MAXIM algorithm has a valid HR and the
                // beat detector has not produced a value yet, make it
                // available to the OLED as a fallback.
                if (
                    validHeartRate &&
                    algorithmHeartRate >= 40 &&
                    algorithmHeartRate <= 200 &&
                    beatAvg == 0
                )
                {
                    beatAvg = (int)algorithmHeartRate;

                    Serial.print("DISPLAY HR FALLBACK: ");
                    Serial.print(beatAvg);
                    Serial.println(" BPM");
                }

                for (int i = 50; i < 100; i++)
                {
                    redBuffer[i - 50] = redBuffer[i];
                    irBuffer[i - 50] = irBuffer[i];
                }

                spo2SampleCount = 50;
            }
        }
        else
        {
            if (fingerDetected)
            {
                Serial.println();
                Serial.println("MAX30102: FINGER REMOVED");
            }

            fingerDetected = false;

            resetHeartRate();
            resetSpO2Buffer();
        }

        particleSensor.nextSample();
    }

    // ------------------------------------------------------------
    // ONE-MINUTE WINDOW ROLLOVER
    // ------------------------------------------------------------

    if (
        fingerDetected &&
        hrMinuteStart != 0 &&
        millis() - hrMinuteStart >= 60000
    )
    {
        if (hrMinuteBpmCount > 0)
        {
            oneMinuteAverageHR =
                (int)round(
                    (double)hrMinuteBpmSum /
                    (double)hrMinuteBpmCount
                );

            if (
                oneMinuteAverageHR >= 40 &&
                oneMinuteAverageHR <= 200
            )
            {
                oneMinuteAverageValid = true;

                Serial.print("===== 60 SECOND HR AVERAGE: ");
                Serial.print(oneMinuteAverageHR);
                Serial.println(" BPM =====");
            }
        }

        // Start the next 60-second window.
        hrMinuteStart = millis();
        hrMinuteBpmSum = 0;
        hrMinuteBpmCount = 0;
    }

    // ------------------------------------------------------------
    // HR TIMEOUT
    // ------------------------------------------------------------

    if (
        fingerDetected &&
        lastBeat != 0 &&
        millis() - lastBeat > HR_TIMEOUT
    )
    {
        Serial.println("HR: no beat detected for 5 seconds");

        beatAvg = 0;
        beatsPerMinute = 0.0;

        hrHistoryCount = 0;
        hrHistoryIndex = 0;

        lastBeat = 0;
    }
}

// ============================================================
//                         DHT22
// ============================================================

void readEnvironment()
{
    if (
        millis() - lastDHTRead <
        DHT_INTERVAL
    )
    {
        return;
    }

    lastDHTRead = millis();

    float newHumidity = dht.readHumidity();
    float newTemperature = dht.readTemperature();

    if (
        !isnan(newTemperature) &&
        !isnan(newHumidity)
    )
    {
        temperatureC = newTemperature;
        humidity = newHumidity;
    }
}


// ============================================================
//                           GPS SETUP
// ============================================================

void setupGPS()
{
    Serial.println();
    Serial.println("Starting GPS...");

    GPS.begin(
        9600,
        SERIAL_8N1,
        GPS_RX,
        GPS_TX
    );

    gpsLine.reserve(120);

    Serial.println("GPS UART READY!");
}


// ============================================================
//                       GPS HELPERS
// ============================================================

double convertNMEACoordinate(
    const String& value,
    char direction
)
{
    if (value.length() < 4)
    {
        return 0.0;
    }

    double raw = value.toDouble();

    int degrees = (int)(raw / 100.0);
    double minutes = raw - (degrees * 100.0);

    double coordinate =
        degrees + (minutes / 60.0);

    if (
        direction == 'S' ||
        direction == 'W'
    )
    {
        coordinate = -coordinate;
    }

    return coordinate;
}


void parseGPSLine(const String& line)
{
    if (
        !line.startsWith("$GPGGA") &&
        !line.startsWith("$GNGGA")
    )
    {
        return;
    }

    // GGA:
    // 0 message
    // 1 UTC time
    // 2 latitude
    // 3 N/S
    // 4 longitude
    // 5 E/W
    // 6 fix quality
    // 7 satellites

    String fields[15];
    int fieldIndex = 0;
    int start = 0;

    for (
        int i = 0;
        i <= line.length() && fieldIndex < 15;
        i++
    )
    {
        if (
            i == line.length() ||
            line[i] == ','
        )
        {
            fields[fieldIndex++] =
                line.substring(start, i);

            start = i + 1;
        }
    }

    if (fieldIndex < 8)
    {
        return;
    }

    int fixQuality = fields[6].toInt();
    int satellites = fields[7].toInt();

    gpsSatellites = satellites;

    if (
        fixQuality > 0 &&
        fields[2].length() > 0 &&
        fields[3].length() > 0 &&
        fields[4].length() > 0 &&
        fields[5].length() > 0
    )
    {
        gpsLatitude =
            convertNMEACoordinate(
                fields[2],
                fields[3][0]
            );

        gpsLongitude =
            convertNMEACoordinate(
                fields[4],
                fields[5][0]
            );

        gpsLocationValid = true;
        gpsConnected = true;
    }
    else
    {
        gpsLocationValid = false;
        gpsConnected = true;
    }
}


// ============================================================
//                          READ GPS
// ============================================================

void readGPS()
{
    while (GPS.available())
    {
        char c = GPS.read();

        if (c == '\n')
        {
            if (gpsLine.length() > 0)
            {
                parseGPSLine(gpsLine);
            }

            gpsLine = "";
        }
        else if (c != '\r')
        {
            if (gpsLine.length() < 120)
            {
                gpsLine += c;
            }
            else
            {
                gpsLine = "";
            }
        }
    }

    // A UART response means the GPS module is alive.
    if (GPS.available())
    {
        gpsConnected = true;
    }
}


// ============================================================
//                          BATTERY
// ============================================================

void readBattery()
{
    if (
        millis() - lastBatteryRead <
        BATTERY_INTERVAL
    )
    {
        return;
    }

    lastBatteryRead = millis();

    int raw = analogRead(BATTERY_PIN);

    float adcVoltage =
        (raw / 4095.0) * 3.3;

    // Assumes a 1:1 voltage divider.
    // Calibrate this later against your real battery circuit.
    batteryVoltage = adcVoltage * 2.0;

    batteryPercent =
        map(
            (int)(batteryVoltage * 100),
            330,
            420,
            0,
            100
        );

    batteryPercent =
        constrain(
            batteryPercent,
            0,
            100
        );
}


// ============================================================
//                  BLE FUNCTION DECLARATIONS
// ============================================================

void sendBLETelemetry();
bool sendSOSPacket();


// ============================================================
//                       BLE CALLBACKS
// ============================================================

class SwasthyaSathiBLEServerCallbacks
    : public BLEServerCallbacks
{
    void onConnect(BLEServer* server)
    {
        bleDeviceConnected = true;
        phoneConnected = true;

        Serial.println();
        Serial.println("================================");
        Serial.println("       PHONE CONNECTED");
        Serial.println("================================");

        if (bleStatusCharacteristic != nullptr)
        {
            bleStatusCharacteristic->setValue(
                "PHONE_CONNECTED"
            );
        }
    }

    void onDisconnect(BLEServer* server)
    {
        bleDeviceConnected = false;
        phoneConnected = false;

        Serial.println();
        Serial.println("PHONE DISCONNECTED");
        Serial.println("BLE ADVERTISING AGAIN...");

        BLEDevice::startAdvertising();
    }
};


// ============================================================
//                    BLE COMMAND CALLBACK
// ============================================================

class SwasthyaSathiBLECommandCallbacks
    : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic* characteristic)
    {
        std::string rawCommand =
            characteristic->getValue();

        String command =
            String(rawCommand.c_str());

        command.trim();

        if (command.length() == 0)
        {
            return;
        }

        Serial.print("BLE COMMAND: ");
        Serial.println(command);

        if (command == "PING")
        {
            if (bleStatusCharacteristic != nullptr)
            {
                bleStatusCharacteristic->setValue(
                    "PONG"
                );
            }

            if (
                bleTelemetryCharacteristic != nullptr &&
                bleDeviceConnected
            )
            {
                bleTelemetryCharacteristic->setValue(
                    "{\"type\":\"pong\",\"device\":\"SWASTHYASATHI WATCH\"}"
                );

                bleTelemetryCharacteristic->notify();
            }
        }
        else if (command == "GET_STATUS")
        {
            sendBLETelemetry();
        }
        else if (command == "GET_SOS")
        {
            sendSOSPacket();
        }
    }
};


// ============================================================
//                     BLE JSON HELPERS
// ============================================================

String jsonNumberOrNull(
    double value,
    bool valid,
    int decimals
)
{
    if (!valid)
    {
        return "null";
    }

    return String(value, decimals);
}


String buildTelemetryJSON()
{
    String json = "{";

    json += "\"type\":\"telemetry\"";

    json += ",\"hr\":";

    int telemetryHR = getDisplayHeartRate();

    if (
        telemetryHR >= 40 &&
        telemetryHR <= 200
    )
    {
        json += String(telemetryHR);
    }
    else
    {
        json += "null";
    }

    json += ",\"spo2\":";
    if (spo2Available)
    {
        json += String(displayedSpO2);
    }
    else
    {
        json += "null";
    }

    json += ",\"temperature\":";
    if (!isnan(temperatureC))
    {
        json += String(temperatureC, 1);
    }
    else
    {
        json += "null";
    }

    json += ",\"humidity\":";
    if (!isnan(humidity))
    {
        json += String(humidity, 1);
    }
    else
    {
        json += "null";
    }

    json += ",\"battery\":";
    json += String(batteryPercent);

    json += ",\"batteryVoltage\":";
    json += String(batteryVoltage, 2);

    json += ",\"gps\":";
    json += gpsLocationValid ? "true" : "false";

    json += ",\"satellites\":";
    json += String(gpsSatellites);

    json += ",\"latitude\":";
    json += jsonNumberOrNull(
        gpsLatitude,
        gpsLocationValid,
        6
    );

    json += ",\"longitude\":";
    json += jsonNumberOrNull(
        gpsLongitude,
        gpsLocationValid,
        6
    );

    json += ",\"wifi\":";
    json +=
        WiFi.status() == WL_CONNECTED
        ? "true"
        : "false";

    json += ",\"ble\":";
    json += bleDeviceConnected
        ? "true"
        : "false";

    json += "}";

    return json;
}


String getCurrentTimestamp()
{
    struct tm timeinfo;

    if (!getLocalTime(&timeinfo, 100))
    {
        return "time_unknown";
    }

    char buffer[32];

    strftime(
        buffer,
        sizeof(buffer),
        "%Y-%m-%d %H:%M:%S",
        &timeinfo
    );

    return String(buffer);
}


String buildSOSJSON()
{
    String json = "{";

    json += "\"type\":\"sos\"";

    json += ",\"timestamp\":\"";
    json += getCurrentTimestamp();
    json += "\"";

    json += ",\"hr\":";
    if (
        fingerDetected &&
        oneMinuteAverageValid &&
        oneMinuteAverageHR >= 40 &&
        oneMinuteAverageHR <= 200
    )
    {
        json += String(oneMinuteAverageHR);
    }
    else if (
        fingerDetected &&
        beatAvg >= 40 &&
        beatAvg <= 200
    )
    {
        json += String(beatAvg);
    }
    else
    {
        json += "null";
    }

    json += ",\"spo2\":";
    if (spo2Available)
    {
        json += String(displayedSpO2);
    }
    else
    {
        json += "null";
    }

    json += ",\"temperature\":";
    if (!isnan(temperatureC))
    {
        json += String(temperatureC, 1);
    }
    else
    {
        json += "null";
    }

    json += ",\"humidity\":";
    if (!isnan(humidity))
    {
        json += String(humidity, 1);
    }
    else
    {
        json += "null";
    }

    json += ",\"battery\":";
    json += String(batteryPercent);

    json += ",\"gps\":";
    json += gpsLocationValid ? "true" : "false";

    json += ",\"satellites\":";
    json += String(gpsSatellites);

    json += ",\"latitude\":";
    json += jsonNumberOrNull(
        gpsLatitude,
        gpsLocationValid,
        6
    );

    json += ",\"longitude\":";
    json += jsonNumberOrNull(
        gpsLongitude,
        gpsLocationValid,
        6
    );

    json += "}";

    return json;
}


// ============================================================
//                       BLE SETUP
// ============================================================

void setupBLE()
{
    Serial.println();
    Serial.println("Starting SwasthyaSathi BLE...");

    BLEDevice::init(
        "SWASTHYASATHI WATCH"
    );

    bleServer =
        BLEDevice::createServer();

    bleServer->setCallbacks(
        new SwasthyaSathiBLEServerCallbacks()
    );

    bleService =
        bleServer->createService(
            SWASTHYASATHI_SERVICE_UUID
        );

    // --------------------------------------------------------
    // STATUS
    // --------------------------------------------------------

    bleStatusCharacteristic =
        bleService->createCharacteristic(
            STATUS_CHARACTERISTIC_UUID,
            BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_WRITE
        );

    bleStatusCharacteristic->setValue(
        "SWASTHYASATHI_READY"
    );

    // --------------------------------------------------------
    // TELEMETRY
    // --------------------------------------------------------

    bleTelemetryCharacteristic =
        bleService->createCharacteristic(
            TELEMETRY_CHARACTERISTIC_UUID,
            BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_NOTIFY
        );

    bleTelemetryCharacteristic->addDescriptor(
        new BLE2902()
    );

    bleTelemetryCharacteristic->setValue(
        "{\"type\":\"telemetry\",\"status\":\"ready\"}"
    );

    // --------------------------------------------------------
    // COMMAND
    // --------------------------------------------------------

    bleCommandCharacteristic =
        bleService->createCharacteristic(
            COMMAND_CHARACTERISTIC_UUID,
            BLECharacteristic::PROPERTY_WRITE |
            BLECharacteristic::PROPERTY_WRITE_NR
        );

    bleCommandCharacteristic->setCallbacks(
        new SwasthyaSathiBLECommandCallbacks()
    );

    // --------------------------------------------------------
    // START SERVICE
    // --------------------------------------------------------

    bleService->start();

    BLEAdvertising* advertising =
        BLEDevice::getAdvertising();

    advertising->addServiceUUID(
        SWASTHYASATHI_SERVICE_UUID
    );

    advertising->setScanResponse(true);

    advertising->setMinPreferred(0x06);
    advertising->setMaxPreferred(0x12);

    BLEDevice::startAdvertising();

    Serial.println("BLE READY!");
    Serial.println("Device name: SWASTHYASATHI WATCH");
    Serial.println("Waiting for phone...");
}


// ============================================================
//                       BLE TELEMETRY
// ============================================================

void sendBLETelemetry()
{
    if (
        !bleDeviceConnected ||
        bleTelemetryCharacteristic == nullptr
    )
    {
        return;
    }

    String json =
        buildTelemetryJSON();

    bleTelemetryCharacteristic->setValue(
        json.c_str()
    );

    bleTelemetryCharacteristic->notify();

    Serial.print("BLE TELEMETRY: ");
    Serial.println(json);
}


// ============================================================
//                           BLE SOS
// ============================================================

bool sendSOSPacket()
{
    if (
        !bleDeviceConnected ||
        bleTelemetryCharacteristic == nullptr
    )
    {
        Serial.println(
            "SOS NOT SENT - PHONE NOT CONNECTED"
        );

        return false;
    }

    String json =
        buildSOSJSON();

    bleTelemetryCharacteristic->setValue(
        json.c_str()
    );

    bleTelemetryCharacteristic->notify();

    Serial.println();
    Serial.println("==============================");
    Serial.println("       SOS SENT TO PHONE");
    Serial.println("==============================");
    Serial.println(json);

    return true;
}


// ============================================================
//                     BLE PERIODIC UPDATE
// ============================================================

void updateBLE()
{
    if (!bleDeviceConnected)
    {
        return;
    }

    if (
        millis() - lastBLETelemetry >=
        BLE_TELEMETRY_INTERVAL
    )
    {
        lastBLETelemetry = millis();

        sendBLETelemetry();
    }
}


// ============================================================
//                     BATTERY ICON
// ============================================================

void drawBattery(
    int x,
    int y
)
{
    display.drawRect(
        x,
        y,
        14,
        7,
        SH110X_WHITE
    );

    display.fillRect(
        x + 14,
        y + 2,
        2,
        3,
        SH110X_WHITE
    );

    int fill =
        map(
            batteryPercent,
            0,
            100,
            0,
            10
        );

    if (fill > 0)
    {
        display.fillRect(
            x + 2,
            y + 2,
            fill,
            3,
            SH110X_WHITE
        );
    }
}


// ============================================================
//                        WIFI ICON
// ============================================================

void drawWiFi()
{
    if (
        WiFi.status() ==
        WL_CONNECTED
    )
    {
        display.drawCircle(
            98,
            5,
            5,
            SH110X_WHITE
        );

        display.drawCircle(
            98,
            5,
            3,
            SH110X_WHITE
        );

        display.fillCircle(
            98,
            5,
            1,
            SH110X_WHITE
        );

        display.fillRect(
            91,
            6,
            14,
            5,
            SH110X_BLACK
        );
    }
}


// ============================================================
//                         BLE ICON
// ============================================================

void drawBLEIcon()
{
    if (!bleDeviceConnected)
    {
        return;
    }

    display.drawLine(
        105,
        1,
        105,
        9,
        SH110X_WHITE
    );

    display.drawLine(
        105,
        1,
        109,
        4,
        SH110X_WHITE
    );

    display.drawLine(
        109,
        4,
        105,
        7,
        SH110X_WHITE
    );

    display.drawLine(
        105,
        7,
        109,
        10,
        SH110X_WHITE
    );
}


// ============================================================
//                      STATUS BAR
// ============================================================

void drawStatusBar(
    const char* title
)
{
    display.setTextSize(1);

    display.setCursor(
        3,
        1
    );

    display.print(title);

    drawWiFi();
    drawBLEIcon();

    drawBattery(
        111,
        1
    );

    display.drawLine(
        0,
        10,
        128,
        10,
        SH110X_WHITE
    );
}


// ============================================================
//                     PAGE INDICATOR
// ============================================================

void drawPageIndicator()
{
    for (int i = 0; i < 4; i++)
    {
        int x =
            55 + i * 6;

        if (
            i ==
            (int)currentScreen
        )
        {
            display.fillCircle(
                x,
                62,
                2,
                SH110X_WHITE
            );
        }
        else
        {
            display.drawCircle(
                x,
                62,
                1,
                SH110X_WHITE
            );
        }
    }
}


// ============================================================
//                   HOME WALLPAPER
// ============================================================

void drawWallpaper()
{
    const int stars[][2] =
    {
        {5, 16},
        {18, 22},
        {30, 14},
        {44, 19},
        {61, 13},
        {75, 22},
        {89, 15},
        {105, 19},
        {119, 14}
    };

    for (
        unsigned int i = 0;
        i < sizeof(stars) / sizeof(stars[0]);
        i++
    )
    {
        bool twinkle =
            (
                animationFrame +
                i * 3
            ) % 12 < 9;

        if (twinkle)
        {
            display.drawPixel(
                stars[i][0],
                stars[i][1],
                SH110X_WHITE
            );
        }
    }

    // Moon
    display.drawCircle(
        105,
        29,
        8,
        SH110X_WHITE
    );

    display.fillCircle(
        101,
        25,
        8,
        SH110X_BLACK
    );

    // Mountains
    display.fillTriangle(
        0,
        53,
        20,
        34,
        40,
        53,
        SH110X_WHITE
    );

    display.fillTriangle(
        25,
        53,
        53,
        29,
        81,
        53,
        SH110X_WHITE
    );

    display.fillTriangle(
        66,
        53,
        93,
        35,
        128,
        53,
        SH110X_WHITE
    );

    // Snow caps / cutouts
    display.fillTriangle(
        53,
        29,
        47,
        35,
        59,
        35,
        SH110X_BLACK
    );

    display.fillTriangle(
        93,
        35,
        87,
        40,
        99,
        40,
        SH110X_BLACK
    );

    // Moving horizon points
    int offset =
        animationFrame % 128;

    for (int i = 0; i < 10; i++)
    {
        int x =
            (
                i * 17 +
                offset
            ) % 128;

        display.drawPixel(
            x,
            54 + (i % 2),
            SH110X_WHITE
        );
    }
}


// ============================================================
//                  HEARTBEAT WAVE
// ============================================================

void drawHeartbeatWave(
    int y,
    bool active
)
{
    int startX = 66;

    int wave[] =
    {
        0, 0, 0, -2,
        2, -9, 14, -4,
        0, 0, 0, 0
    };

    for (int i = 0; i < 11; i++)
    {
        int x1 =
            startX + i * 5;

        int x2 =
            startX + (i + 1) * 5;

        int y1 =
            y +
            (
                active
                ? wave[i]
                : 0
            );

        int y2 =
            y +
            (
                active
                ? wave[i + 1]
                : 0
            );

        display.drawLine(
            x1,
            y1,
            x2,
            y2,
            SH110X_WHITE
        );
    }
}


// ============================================================
//                    DISPLAY HEALTH HELPERS
// ============================================================

// Return the best currently available HR value.
//
// Priority:
// 1. Completed one-minute average
// 2. Smoothed beat-to-beat HR
// 3. MAXIM algorithm HR
int getDisplayHeartRate()
{
    if (
        oneMinuteAverageValid &&
        oneMinuteAverageHR >= 40 &&
        oneMinuteAverageHR <= 200
    )
    {
        return oneMinuteAverageHR;
    }

    if (
        beatAvg >= 40 &&
        beatAvg <= 200
    )
    {
        return beatAvg;
    }

    if (
        validHeartRate &&
        algorithmHeartRate >= 40 &&
        algorithmHeartRate <= 200
    )
    {
        return (int)algorithmHeartRate;
    }

    return 0;
}

bool hasDisplayHeartRate()
{
    return getDisplayHeartRate() > 0;
}

int getDisplaySpO2()
{
    if (
        spo2Available &&
        displayedSpO2 >= 70 &&
        displayedSpO2 <= 100
    )
    {
        return displayedSpO2;
    }

    return 0;
}

// ============================================================
//                         HOME
// ============================================================

void drawHome()
{
    display.clearDisplay();

    drawWallpaper();

    // Information panels.
    display.fillRect(
        0,
        0,
        128,
        12,
        SH110X_BLACK
    );

    display.fillRect(
        0,
        12,
        128,
        40,
        SH110X_BLACK
    );

    display.fillRect(
        0,
        50,
        128,
        14,
        SH110X_BLACK
    );

    display.setTextSize(1);

    display.setCursor(4, 2);
    display.print("SWASTHYASATHI");

    drawWiFi();
    drawBLEIcon();

    drawBattery(
        111,
        2
    );

    struct tm timeinfo;

    if (getLocalTime(&timeinfo, 50))
    {
        int hour = timeinfo.tm_hour;

        const char* ampm =
            hour >= 12
            ? "PM"
            : "AM";

        int displayHour =
            hour % 12;

        if (displayHour == 0)
        {
            displayHour = 12;
        }

        char timeString[8];

        snprintf(
            timeString,
            sizeof(timeString),
            "%02d:%02d",
            displayHour,
            timeinfo.tm_min
        );

        display.setTextSize(2);

        display.setCursor(
            18,
            15
        );

        display.print(timeString);

        display.setTextSize(1);

        display.setCursor(
            82,
            19
        );

        display.print(ampm);

        const char* months[] =
        {
            "JAN", "FEB", "MAR",
            "APR", "MAY", "JUN",
            "JUL", "AUG", "SEP",
            "OCT", "NOV", "DEC"
        };

        display.setCursor(
            43,
            34
        );

        display.printf(
            "%02d %s %04d",
            timeinfo.tm_mday,
            months[timeinfo.tm_mon],
            timeinfo.tm_year + 1900
        );

        display.setCursor(
            57,
            43
        );

        display.print("IST");
    }

    display.drawLine(
        0,
        51,
        128,
        51,
        SH110X_WHITE
    );

    display.setTextSize(1);

    display.setCursor(
        3,
        54
    );

    display.print("HR ");

    int homeHR = getDisplayHeartRate();

    if (homeHR > 0)
    {
        display.print(homeHR);
        display.print(" BPM");
    }
    else
    {
        display.print("-- BPM");
    }

    display.setCursor(
        72,
        54
    );

    display.print("T ");

    if (!isnan(temperatureC))
    {
        display.print(
            temperatureC,
            0
        );

        display.print("C");
    }
    else
    {
        display.print("--");
    }

    drawPageIndicator();

    display.display();
}


// ============================================================
//                         HEALTH
// ============================================================

void drawHealth()
{
    display.clearDisplay();

    drawStatusBar("HEALTH");

    display.setTextSize(1);

    display.setCursor(
        4,
        16
    );

    display.print("HEART RATE");

    display.setTextSize(2);

    display.setCursor(
        8,
        26
    );

    int healthHR = getDisplayHeartRate();

    if (healthHR > 0)
    {
        display.print(healthHR);
    }
    else
    {
        display.print("--");
    }

    display.setTextSize(1);
    display.print(" BPM");

    drawHeartbeatWave(
        34,
        fingerDetected
    );

    display.drawLine(
        0,
        44,
        128,
        44,
        SH110X_WHITE
    );

    display.setCursor(
        4,
        49
    );

    display.print("SpO2");

    display.setCursor(
        78,
        49
    );

    int healthSpO2 = getDisplaySpO2();

    if (healthSpO2 > 0)
    {
        display.print(healthSpO2);
        display.print("%");
    }
    else
    {
        display.print("--%");
    }

    drawPageIndicator();

    display.display();
}


// ============================================================
//                      ENVIRONMENT
// ============================================================

void drawEnvironment()
{
    display.clearDisplay();

    drawStatusBar("ENVIRONMENT");

    display.setCursor(
        4,
        16
    );

    display.print("TEMPERATURE");

    display.setTextSize(2);

    display.setCursor(
        34,
        26
    );

    if (!isnan(temperatureC))
    {
        display.print(
            temperatureC,
            1
        );

        display.setTextSize(1);
        display.print(" C");
    }
    else
    {
        display.print("--.- C");
    }

    display.drawLine(
        0,
        44,
        128,
        44,
        SH110X_WHITE
    );

    display.setTextSize(1);

    display.setCursor(
        4,
        49
    );

    display.print("HUMIDITY");

    display.setCursor(
        94,
        49
    );

    if (!isnan(humidity))
    {
        display.print(
            humidity,
            0
        );

        display.print("%");
    }
    else
    {
        display.print("--%");
    }

    drawPageIndicator();

    display.display();
}


// ============================================================
//                         LOCATION
// ============================================================

void drawLocation()
{
    display.clearDisplay();

    drawStatusBar("LOCATION");

    display.setTextSize(1);

    display.setCursor(
        4,
        17
    );

    display.print("GPS");

    display.setCursor(
        54,
        17
    );

    if (gpsLocationValid)
    {
        display.print("FIXED");
    }
    else if (gpsConnected)
    {
        display.print("SEARCHING");
    }
    else
    {
        display.print("OFFLINE");
    }

    display.setCursor(
        4,
        31
    );

    display.print("SATELLITES");

    display.setCursor(
        99,
        31
    );

    if (gpsSatellites > 0)
    {
        display.print(
            gpsSatellites
        );
    }
    else
    {
        display.print("--");
    }

    display.drawLine(
        0,
        42,
        128,
        42,
        SH110X_WHITE
    );

    display.setCursor(
        4,
        47
    );

    if (gpsLocationValid)
    {
        display.print("LOCATION READY");
    }
    else
    {
        display.print("WAITING FOR GPS");
    }

    // Keep the small GPS coordinates available to the app
    // through BLE rather than overcrowding the OLED.
    drawPageIndicator();

    display.display();
}


// ============================================================
//                    SCREEN TRANSITION
// ============================================================

void transitionTo(
    Screen newScreen
)
{
    previousScreen =
        currentScreen;

    currentScreen =
        newScreen;

    for (
        int x = 0;
        x <= 128;
        x += 16
    )
    {
        display.clearDisplay();

        display.fillRect(
            0,
            0,
            x,
            64,
            SH110X_WHITE
        );

        display.display();

        delay(10);
    }

    drawCurrentScreen();
}


// ============================================================
//                    DRAW CURRENT SCREEN
// ============================================================

void drawCurrentScreen()
{
    switch (currentScreen)
    {
        case HOME_SCREEN:
            drawHome();
            break;

        case HEALTH_SCREEN:
            drawHealth();
            break;

        case ENVIRONMENT_SCREEN:
            drawEnvironment();
            break;

        case LOCATION_SCREEN:
            drawLocation();
            break;
    }
}


// ============================================================
//                       NEXT SCREEN
// ============================================================

void nextScreen()
{
    Screen next =
        (Screen)(
            (
                (int)currentScreen + 1
            ) % 4
        );

    transitionTo(next);
}


// ============================================================
//                    SOS PROGRESS SCREEN
// ============================================================

void drawSOSProgress(
    unsigned long heldTime
)
{
    display.clearDisplay();

    display.drawRoundRect(
        1,
        1,
        126,
        62,
        5,
        SH110X_WHITE
    );

    display.setTextSize(2);

    display.setCursor(
        44,
        5
    );

    display.print("SOS");

    display.setTextSize(1);

    display.setCursor(
        23,
        25
    );

    display.print("KEEP HOLDING");

    display.drawRect(
        10,
        39,
        108,
        9,
        SH110X_WHITE
    );

    int progress =
        map(
            min(
                heldTime,
                SOS_HOLD_TIME
            ),
            0,
            SOS_HOLD_TIME,
            0,
            104
        );

    display.fillRect(
        12,
        41,
        progress,
        5,
        SH110X_WHITE
    );

    float remaining =
        (
            SOS_HOLD_TIME -
            min(
                heldTime,
                SOS_HOLD_TIME
            )
        ) / 1000.0;

    display.setCursor(
        48,
        53
    );

    display.print(
        remaining,
        1
    );

    display.print("s");

    display.display();
}


// ============================================================
//                         SOS RESULT
// ============================================================

void drawSOSResult()
{
    display.clearDisplay();

    display.drawRoundRect(
        2,
        2,
        124,
        60,
        5,
        SH110X_WHITE
    );

    if (sosWasSent)
    {
        display.setTextSize(2);

        display.setCursor(
            21,
            8
        );

        display.print("SOS SENT");

        display.setTextSize(1);

        display.setCursor(
            17,
            34
        );

        display.print("BLE TO PHONE");

        display.setCursor(
            20,
            47
        );

        display.print("WAIT FOR APP");
    }
    else
    {
        display.setTextSize(2);

        display.setCursor(
            13,
            8
        );

        display.print("SOS ALERT");

        display.setTextSize(1);

        display.setCursor(
            14,
            34
        );

        display.print("PHONE NOT CONNECTED");

        display.setCursor(
            28,
            48
        );

        display.print("NOT SENT");
    }

    display.display();
}


// ============================================================
//                        TRIGGER SOS
// ============================================================

void triggerSOS()
{
    Serial.println();
    Serial.println("==============================");
    Serial.println("          !!! SOS !!!");
    Serial.println("==============================");

    // Send actual BLE SOS packet only if a phone is connected.
    sosWasSent =
        sendSOSPacket();

    sosMessageActive = true;
    sosMessageStart = millis();

    drawSOSResult();
}


// ============================================================
//                      TOUCH HANDLER
// ============================================================

void handleTouch()
{
    bool touchState =
        digitalRead(TOUCH_PIN);

    // --------------------------------------------------------
    // TOUCH START
    // --------------------------------------------------------

    if (
        touchState == HIGH &&
        !touchPressed &&
        millis() - lastTouchRelease >
        TOUCH_DEBOUNCE
    )
    {
        touchPressed = true;
        longPressTriggered = false;
        touchStartTime = millis();

        Serial.println("TOUCH START");
    }

    // --------------------------------------------------------
    // HOLD
    // --------------------------------------------------------

    if (
        touchState == HIGH &&
        touchPressed
    )
    {
        unsigned long held =
            millis() - touchStartTime;

        if (
            held >= SOS_PREVIEW_TIME &&
            !longPressTriggered
        )
        {
            drawSOSProgress(held);
        }

        if (
            held >= SOS_HOLD_TIME &&
            !longPressTriggered
        )
        {
            longPressTriggered = true;

            triggerSOS();
        }
    }

    // --------------------------------------------------------
    // RELEASE
    // --------------------------------------------------------

    if (
        touchState == LOW &&
        touchPressed
    )
    {
        unsigned long duration =
            millis() - touchStartTime;

        touchPressed = false;
        lastTouchRelease = millis();

        if (
            !longPressTriggered &&
            duration >= 50 &&
            duration < 600
        )
        {
            Serial.println("SHORT TAP");
            nextScreen();
        }
        else if (!longPressTriggered)
        {
            Serial.println("SOS CANCELLED");
            drawCurrentScreen();
        }
    }
}


// ============================================================
//                    ANIMATION UPDATE
// ============================================================

void updateAnimation()
{
    if (
        millis() - lastAnimationFrame <
        ANIMATION_INTERVAL
    )
    {
        return;
    }

    lastAnimationFrame = millis();
    animationFrame++;
}


// ============================================================
//                           SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);

    delay(1000);

    Serial.println();
    Serial.println("================================");
    Serial.println(" SWASTHYASATHI WATCH OS OLED");
    Serial.println("================================");

    // --------------------------------------------------------
    // OLED
    // --------------------------------------------------------

    Wire.begin(
        OLED_SDA,
        OLED_SCL
    );

    if (
        !display.begin(
            0x3C,
            true
        )
    )
    {
        Serial.println("OLED NOT FOUND!");

        while (true)
        {
            delay(1000);
        }
    }

    Serial.println("OLED FOUND!");

    display.setRotation(2);
    display.setTextColor(SH110X_WHITE);

    // --------------------------------------------------------
    // BOOT
    // --------------------------------------------------------

    bootAnimation();

    // --------------------------------------------------------
    // TOUCH
    // --------------------------------------------------------

    pinMode(
        TOUCH_PIN,
        INPUT
    );

    // --------------------------------------------------------
    // DHT22
    // --------------------------------------------------------

    dht.begin();

    // --------------------------------------------------------
    // WIFI
    // --------------------------------------------------------

    connectWiFi();

    // --------------------------------------------------------
    // TIME
    // --------------------------------------------------------

    if (
        WiFi.status() ==
        WL_CONNECTED
    )
    {
        setupTime();
    }

    // --------------------------------------------------------
    // MAX30102
    // --------------------------------------------------------

    setupMAX30102();

    // --------------------------------------------------------
    // GPS
    // --------------------------------------------------------

    setupGPS();

    // --------------------------------------------------------
    // BATTERY
    // --------------------------------------------------------

    analogReadResolution(12);

    pinMode(
        BATTERY_PIN,
        INPUT
    );

    // --------------------------------------------------------
    // BLE
    // --------------------------------------------------------

    setupBLE();

    // --------------------------------------------------------
    // HOME
    // --------------------------------------------------------

    currentScreen =
        HOME_SCREEN;

    drawHome();

    Serial.println();
    Serial.println("================================");
    Serial.println("       WATCH OS READY");
    Serial.println("================================");
    Serial.println("SHORT TAP = NEXT SCREEN");
    Serial.println("HOLD 3 SEC = SOS");
    Serial.println("BLE = SWASTHYASATHI WATCH");
    Serial.println("SpO2 = MAXIM ALGORITHM");
}


// ============================================================
//                            LOOP
// ============================================================

void loop()
{
    // --------------------------------------------------------
    // MAX30102
    // --------------------------------------------------------

    readMAX30102();

    // --------------------------------------------------------
    // DHT22
    // --------------------------------------------------------

    readEnvironment();

    // --------------------------------------------------------
    // GPS
    // --------------------------------------------------------

    readGPS();

    // --------------------------------------------------------
    // BATTERY
    // --------------------------------------------------------

    readBattery();

    // --------------------------------------------------------
    // BLE
    // --------------------------------------------------------

    updateBLE();

    // --------------------------------------------------------
    // TOUCH
    // --------------------------------------------------------

    handleTouch();

    // --------------------------------------------------------
    // ANIMATION
    // --------------------------------------------------------

    updateAnimation();

    // --------------------------------------------------------
    // SOS RESULT TIMEOUT
    // --------------------------------------------------------

    if (
        sosMessageActive &&
        millis() - sosMessageStart >
        SOS_MESSAGE_DURATION
    )
    {
        sosMessageActive = false;
        drawCurrentScreen();
    }

    // --------------------------------------------------------
    // OLED REFRESH
    // --------------------------------------------------------

    static unsigned long lastDisplayUpdate = 0;

    if (
        !touchPressed &&
        !sosMessageActive &&
        millis() - lastDisplayUpdate >= 100
    )
    {
        drawCurrentScreen();

        lastDisplayUpdate = millis();
    }

    delay(5);
}
