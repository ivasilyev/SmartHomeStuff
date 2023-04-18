#include <map>
#include <string>

#include <unordered_map>
#include <Arduino.h>
#include <SPI.h>
#include <User_Setup.h>
#include <WiFiCredentials.h>
#include <WiFi.h>
#include <Wire.h>
#include <WebServer.h>

#include <../.pio/libdeps/esp32dev/Adafruit Unified Sensor/Adafruit_Sensor.h>
#include <../.pio/libdeps/esp32dev/Adafruit BME280 Library/Adafruit_BME280.h>
#include <../.pio/libdeps/esp32dev/TFT_eSPI/TFT_eSPI.h>

#ifdef __cplusplus
    extern "C" {
#endif
    uint8_t temprature_sens_read();
#ifdef __cplusplus
}
#endif
uint8_t temprature_sens_read();

#define PROMETHEUS_NAMESPACE "iot"

const int DISPLAY_HEIGHT = 128;
const int DISPLAY_WIDTH = 128;

/*
ST7735 SPI wiring:
LED 3V3 (!)
SCK G16
SDA (MOSI) G23
AO (DC) G2
Reset G4
CS G17
GND GND
VCC 3V3
*/

const int BME280_ADDR = 0x76;
const int BME280_POLLING_DELAY = 3000;
const float SEALEVELPRESSURE_HPA = 1013.25;
const std::vector<std::string> DISPLAY_METRICS = {
    "air_humidity_percent",
    "air_pressure_mmhg",
    "air_temperature_celsius",
};

/*
BME280 I2C wiring:
SCL - D22
SDA - D21
VDD - 3V3
GND - GND
*/

WebServer webServer(80);

// Copy contents of User_Setup.h.bak into '.pio/libdeps/esp32dev/TFT_eSPI/User_Setup.h'
TFT_eSPI tft = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h

Adafruit_BME280 bme;

std::map<std::string, std::map<std::string, std::string>> sensorData;

void _populateSensorData(std::string name, std::string help, std::string type) {
    sensorData[name] = std::map<std::string, std::string> {
        {"help", help},
        {"type", type},
        {"value", ""}
    };
}

void populateSensorData() {
    std::string g = "gauge";

    _populateSensorData("air_temperature_celsius", "Temperature, °C", g);
    _populateSensorData("air_temperature_fahrenheit", "Temperature, °F", g);
    _populateSensorData("air_pressure_hpa", "Atmospheric Pressure, hPa", g);
    _populateSensorData("air_pressure_mmhg", "Pressure, mmHg", g);
    _populateSensorData("altitude_m", "Altitude, m", g);
    _populateSensorData("air_humidity_percent", "Humidity, %", g);

    _populateSensorData("system_up_time_ms", "System uptime", g);
    _populateSensorData("memory_total_heap_size", "Total heap memory size", g);
    _populateSensorData("memory_free_heap_size_bytes", "Free memory size", g);
    _populateSensorData("cpu_frequency_mhz", "CPU frequency", g);
    _populateSensorData("cpu_temperature_celsius", "CPU temperature, °C", g);
    _populateSensorData("cpu_temperature_fahrenheit", "CPU temperature, °F", g);
    _populateSensorData("sketch_size_bytes", "Sketch size", g);
    _populateSensorData("flash_size_bytes", "Flash size", g);
    _populateSensorData("available_size_bytes", "Available size", g);
}

void setSensorData(std::string name, float value) {
    sensorData[name]["value"] = std::to_string(value);
}


std::map<String, String> getSensorHelpAndValue(std::string name) {
    return std::map<String, String> { 
        {"help", sensorData[name]["help"].c_str()}, 
        {"value", sensorData[name]["value"].c_str()}
    };
}


void checkSensor() {
    Serial.println(F("BME280 test"));
    bool status = bme.begin(BME280_ADDR);  
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1);
    }
    Serial.println(F("BME280 OK"));
}


String metricsBuffer;


std::string handleMetric(std::string name) {
    /*
    Example Prometheus exporter output:
    # HELP go_goroutines Number of goroutines that currently exist.
    # TYPE go_goroutines gauge
    go_goroutines 10
    */
    std::string out;
    std::string _name = PROMETHEUS_NAMESPACE "_" + name;
    out += "# HELP " + _name + " " + sensorData[name]["help"] + "\n";
    out += "# TYPE " + _name + " " + sensorData[name]["type"] + "\n";
    out += _name + " " + sensorData[name]["value"] + "\n";
    return out;
}


void updateMetricsBuffer() {
    std::string out;
    for (auto pair : sensorData) {
        out += handleMetric(pair.first);
    }
    metricsBuffer = out.c_str();
}


void pollSensor() {
    float celsius = bme.readTemperature();
    setSensorData("air_temperature_celsius", celsius);
    setSensorData("air_temperature_fahrenheit", celsius * 1.8f + 32.0f);

    float pascal = bme.readPressure();
    setSensorData("air_pressure_hpa", pascal / 100.0f);
    setSensorData("air_pressure_mmhg", pascal * 7.5006f / 1000.0f);

    setSensorData("altitude_m", bme.readAltitude(SEALEVELPRESSURE_HPA));
    setSensorData("air_humidity_percent", bme.readHumidity());

    setSensorData("system_up_time_ms", millis());
    setSensorData("memory_total_heap_size", ESP.getHeapSize());
    setSensorData("memory_free_heap_size_bytes", ESP.getFreeHeap());
    setSensorData("cpu_frequency_mhz", getCpuFrequencyMhz());
    
    float fahrenheit = temprature_sens_read();
    setSensorData("cpu_temperature_fahrenheit", fahrenheit);
    setSensorData("cpu_temperature_celsius", (fahrenheit - 32.0f) / 1.8f);

    int sketch_size = ESP.getSketchSize();
    int flash_size =  ESP.getFreeSketchSpace();
    int available_size = flash_size - sketch_size;
    setSensorData("sketch_size_bytes", sketch_size);
    setSensorData("flash_size_bytes", flash_size);
    setSensorData("available_size_bytes", available_size);

    updateMetricsBuffer();
}


void printSensor() {
    for (auto pair : sensorData) {
        auto values = getSensorHelpAndValue(pair.first);
        Serial.print(values["help"]);
        Serial.print(": ");
        Serial.println(values["value"]);
    }
}


String sensorToTag() {
    String out = "<ul>";
    for (auto pair : sensorData) {
        auto values = getSensorHelpAndValue(pair.first);
        out += "<li>";
        out += values["help"];
        out += ": ";
        out += values["value"];
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
    out += sensorToTag();
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
    for (auto pair : sensorData) {
        auto values = getSensorHelpAndValue(pair.first);
        out += "\"";
        out += values["help"];
        out += "\": ";
        out += values["value"];
        out += ", ";
    }
    out = out.substring(0, out.length() - 2) + "}";
    webServer.send(200, "application/json", out);
}

void webHandleMetrics() {
    Serial.println("Sending metrics");
    webServer.send(200, "text/plain", metricsBuffer);
}


void setupWebServer() {
    webServer.on("/", HTTP_GET, webHandleRoot);
    webServer.on("/json", HTTP_GET, webHandleJSON);
    webServer.on("/metrics", HTTP_GET, webHandleMetrics);
}

void testDisplay() {
    tft.init();
    tft.begin();
    tft.fillScreen(TFT_RED);
    delay(BME280_POLLING_DELAY / 5);
    tft.fillScreen(TFT_GREEN);
    delay(BME280_POLLING_DELAY / 5);
    tft.fillScreen(TFT_BLUE);
    tft.drawRect(0,0,128,128,TFT_GREEN);
    delay(BME280_POLLING_DELAY);
    tft.fillScreen(TFT_BLACK);
}


void drawSensor() {
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(0, 1);
    String help;
    std::string value;

    for (auto name : DISPLAY_METRICS) {
        help = sensorData[name]["help"].c_str();
        // s = s.substring(s.indexOf(",") + 2, s.length());
        help.replace("°", "`");
        tft.setTextFont(2.4);
        tft.setTextColor(TFT_YELLOW);
        tft.println(help);

        value = sensorData[name]["value"];
        value = value.substr(0, value.find(".")+3);
        tft.setTextFont(4.9);
        tft.setTextColor(TFT_WHITE);
        tft.println(value.c_str());
    }
}


void setup() {
    Serial.begin(9600);
    populateSensorData();
    checkSensor();
    forceConnectToWiFi();
    webServer.begin();
    setupWebServer();
    testDisplay();
    //tft.setWindow(20, 20, 148, 148);
    //tft.setRotation(0);
    //tft.setTextColor(TFT_GREEN);  // Adding a black background colour erases previous text automatically
}


void loop() { 
    pollSensor();
    // printSensor();
    delay(BME280_POLLING_DELAY);
    webServer.handleClient();
    drawSensor();
}
