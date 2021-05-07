using namespace std; 

#include <unordered_map>
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
//#include <../.pio/libdeps/esp32dev/TFT_eSPI/TFT_eSPI.h>
//#include <setup_display.h>
#include <../.pio/libdeps/esp32dev/Adafruit Unified Sensor/Adafruit_Sensor.h>
#include <../.pio/libdeps/esp32dev/Adafruit BME280 Library/Adafruit_BME280.h>

#include <WiFiCredentials.h>
#include <WiFi.h>
#include <WebServer.h>


#define SEALEVELPRESSURE_HPA (1013.25)

const int DISPLAY_HEIGHT = 128;
const int DISPLAY_WIDTH = 128;
const int BME280_ADDR = 0x76;
const int BME280_POLLING_DELAY = 1200;

/*
BME280 I2C wiring:
SCL - D22
SDA - D21
VDD - 3V3
GND - GND
*/

Adafruit_BME280 bme;
unordered_map<string, float> sensorData;
WebServer webServer(80);

void checkSensor() {
    Serial.println(F("BME280 test"));
    bool status = bme.begin(BME280_ADDR);  
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1);
    }
}

void pollSensor() {
    sensorData["Temperature, °C"] = bme.readTemperature();
    sensorData["Temperature, °F"] = sensorData["Temperature, °C"] * 1.8f + 32.0f;
    sensorData["Pressure, hPa"] = bme.readPressure() / 100.0f;
    sensorData["Pressure, mmHg"] = sensorData["Pressure, hPa"] * 0.75f;
    sensorData["Altitude, m"] = bme.readAltitude(SEALEVELPRESSURE_HPA);
    sensorData["Humidity, %"] = bme.readHumidity();
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
    WiFi.setHostname("ESP32_BME280");
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
    "        <title>BME280 | ESP32 Web Server</title>\n"
    "    </head>\n"
    "    <body>\n"
    "        <div>\n"
    "            <h1>ESP32 Web Server</h1>\n"
    "            <div>\n"
    "                BME280 data:\n"
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

void setup() {
    Serial.begin(9600);
    checkSensor();
    forceConnectToWiFi();
    webServer.begin();
    setupWebServer();
}

void loop() { 
    pollSensor();
    // printSensor();
    delay(BME280_POLLING_DELAY);
    webServer.handleClient();
}
