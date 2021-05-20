#include <map>
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include <../.pio/libdeps/esp32dev/Adafruit Unified Sensor/Adafruit_Sensor.h>
#include <../.pio/libdeps/esp32dev/Adafruit BME680 Library/Adafruit_BME680.h>

#include <WiFiCredentials.h>
#include <WiFi.h>
#include <WebServer.h>

#include <../.pio/libdeps/esp32dev/U8g2/src/U8g2lib.h>

#define SEALEVELPRESSURE_HPA (1013.25)

const int BME680_ADDR = 0x77;
const int BME680_POLLING_DELAY = 1200;

/*
BME680 I2C wiring:
SCL - D22
SDA - D21
VDD - 3V3
GND - GND
*/

Adafruit_BME680 bme;
//map<string, float> sensorData;
std::map<std::string, float> sensorData;

WebServer webServer(80);


/*
SH1106 18x64 1.3" OLED SPI wiring:
LED 3V3 (!)
SCK G16
SDA (MOSI) G23
AO (DC) G2
Reset G4
CS G17
GND GND
VCC 3V3
*/

// bus, *rotation, cs, dc, reset
//U8G2_SH1106_128X64_NONAME_1_4W_HW_SPI u8g2(U8G2_R0, 17, 2, 4);
// bus, *rotation, clock, data, cs, dc, reset
U8G2_SH1106_128X64_NONAME_1_4W_SW_SPI u8g2(U8G2_R0, 16, 23, 17, 2, 4);


void checkSensor() {
    Serial.println(F("BME680 test"));
    bool status = bme.begin(BME680_ADDR); 

    if (!status) {
        Serial.println("Could not find a valid BME680 sensor, check wiring!");
        while (1);
    }

    Serial.println(F("BME680 OK"));
    
    Serial.println(F("Set up oversampling and filter initialization"));
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150);  // 320*C for 150 ms
}


void pollSensor() {
    sensorData["Temperature, °C"] = bme.readTemperature();
    sensorData["Temperature, °F"] = sensorData["Temperature, °C"] * 1.8f + 32.0f;
    sensorData["Pressure, hPa"] = bme.readPressure() / 100.0f;
    sensorData["Pressure, mmHg"] = sensorData["Pressure, hPa"] * 0.75f;
    sensorData["Altitude, m"] = bme.readAltitude(SEALEVELPRESSURE_HPA);
    sensorData["Humidity, %"] = bme.readHumidity();
    sensorData["Gas, KOhms"] = bme.readGas() / 1000.0f;
}


void printSensor() {
    for (auto it : sensorData) {
        Serial.print(it.first.c_str());
        Serial.print(": ");
        Serial.println(it.second);
    }
}


String stringifySensor() {
    String out = "<ul>";
    for (auto it : sensorData) {
        out += "<li>";
        //
        out += it.first.c_str();
        out += ": ";
        out += String(it.second);
        //
        out += "</li>";
    }
    out += "</ul>";
    return out;
}


void forceConnectToWiFi() {
    WiFi.setHostname("ESP32_BME680");
    Serial.print("Connecting to ");
    Serial.println(AP_SSID);
    WiFi.begin(AP_SSID, AP_PASSWORD);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println();
    Serial.println("WiFi was connected succesfully!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}


void webHandleRoot() {
    Serial.println("Connected client");
    String out = 
    "<!DOCTYPE html> <html>\n"
    "    <!-- To convert it into C++ code, use tools like tomeko.net -->\n"
    "    <head>\n"
    "        <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n"
    "        <meta http-equiv=\"Content-Type\" content=\"text/html; charset=utf-8\" />\n"
    "        <meta http-equiv=\"refresh\" content=\"10\">\n"
    "        <title>BME680 | ESP32 Web Server</title>\n"
    "    </head>\n"
    "    <body>\n"
    "        <div>\n"
    "            <h1>ESP32 Web Server</h1>\n"
    "            <div>\n"
    "                BME680 data:\n"
    "                <!-- Payload placeholder -->\n"
    ;
    out += stringifySensor();
    out +=
    "            </div>\n"
    "        </div>\n"
    "    </body>\n"
    "</html>\n"
    ""
    ;
    webServer.send(200, "text/html", out); 
}


void webHandleJSON() {
    Serial.println("Sending JSON");
    String out = "{";
    for (auto it : sensorData) {
        out += "\"";
        out += it.first.c_str();
        out += "\": ";
        out += String(it.second);
        out += ", ";
    }
    out = out.substring(0, out.length() - 2) + "}";
    webServer.send(200, "application/json", out);
}


void setupWebServer() {
    webServer.on("/", HTTP_GET, webHandleRoot);
    webServer.on("/json", HTTP_GET, webHandleJSON);
}


void drawSensor() {
    u8g2.firstPage();
    int lineSpacing = 17;
    do {
        int nextCursorPosition = 10;
        for (auto it : sensorData) {
            String s = it.first.c_str();

            if (! s.endsWith("%") && ! s.endsWith("C") && ! s.endsWith("mmHg") && ! s.endsWith("KOhms")) {
                continue;
            }
            u8g2.setFont(u8g2_font_prospero_nbp_tf);
            u8g2.drawUTF8(0, nextCursorPosition, s.c_str());
            u8g2.drawUTF8(93, nextCursorPosition, String(it.second).c_str());
            nextCursorPosition += lineSpacing;
        }
    } while (u8g2.nextPage());
}


void setup() {
    Serial.begin(9600);
    checkSensor();
    forceConnectToWiFi();
    webServer.begin();
    setupWebServer();

    u8g2.begin();
}


void loop() { 
    pollSensor();
    // printSensor();
    delay(BME680_POLLING_DELAY);
    webServer.handleClient();

    drawSensor();
}
