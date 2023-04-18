#include <map>
#include <string>
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include <../.pio/libdeps/esp32dev/Adafruit Unified Sensor/Adafruit_Sensor.h>
#include <../.pio/libdeps/esp32dev/Adafruit BME680 Library/Adafruit_BME680.h>

#include <WiFiCredentials.h>
#include <WiFi.h>
#include <WebServer.h>

#include <../.pio/libdeps/esp32dev/U8g2/src/U8g2lib.h>

#define PROMETHEUS_NAMESPACE "iot"

#ifdef __cplusplus
    extern "C" {
#endif
    uint8_t temprature_sens_read();
#ifdef __cplusplus
}
#endif
uint8_t temprature_sens_read();


const int BME680_ADDR = 0x77;
const int BME680_POLLING_DELAY = 3000;
const std::vector<std::string> DISPLAY_METRICS = {
    "gas_resistance_kohm",
    "air_humidity_percent",
    "air_pressure_mmhg",
    "air_temperature_celsius",
};


/*
BME680 I2C wiring:
SCL - D22
SDA - D21
VDD - 3V3
GND - GND
*/

Adafruit_BME680 bme;
//map<string, float> sensorData;
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
    _populateSensorData("air_pressure_mmhg", "Pressure, mmHg", g);
    _populateSensorData("air_humidity_percent", "Humidity, %", g);
    _populateSensorData("gas_resistance_kohm", "Gas, KOhms", g);

    // From https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/Esp.h
    _populateSensorData("system_up_time_ms", "System uptime in ms", g);
    _populateSensorData("cpu_frequency_mhz", "CPU frequency in MHz", g);
    _populateSensorData("cpu_temperature_celsius", "CPU temperature, °C", g);
    _populateSensorData("memory_heap_used_bytes", "Total heap memory size in bytes", g);
    _populateSensorData("memory_heap_free_bytes", "Free memory size in bytes", g);
    _populateSensorData("memory_psram_total_bytes", "Total SPI RAM size in bytes", g);
    _populateSensorData("memory_psram_free_bytes", "Free SPI RAM size in bytes", g);
    _populateSensorData("memory_flash_used_bytes", "Sketch size in bytes", g);
    _populateSensorData("memory_flash_total_bytes", "Flash size in bytes", g);
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
    bme.setGasHeater(320, 150);  // 320 °C for 150 ms
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
    setSensorData("air_temperature_celsius", bme.readTemperature());
    setSensorData("air_pressure_mmhg", bme.readPressure() * 7.5006f / 1000.0f);
    setSensorData("air_humidity_percent", bme.readHumidity());
    setSensorData("gas_resistance_kohm", bme.readGas() / 1000.0f);

    setSensorData("system_up_time_ms", millis());
    setSensorData("cpu_frequency_mhz", getCpuFrequencyMhz());
    setSensorData("cpu_temperature_celsius", (temprature_sens_read() - 32.0f) / 1.8f);
    setSensorData("memory_heap_used_bytes", ESP.getHeapSize());
    setSensorData("memory_heap_free_bytes", ESP.getFreeHeap());
    setSensorData("memory_psram_total_bytes", ESP.getPsramSize());
    setSensorData("memory_psram_free_bytes", ESP.getFreePsram());
    setSensorData("memory_flash_used_bytes", ESP.getSketchSize());
    setSensorData("memory_flash_total_bytes", ESP.getFreeSketchSpace());

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


void drawSensor() {
    u8g2.firstPage();
    int lineSpacing = 17;
    do {
        int nextCursorPosition = 10;
        for (auto i : DISPLAY_METRICS) {
            auto values = getSensorHelpAndValue(i);

            u8g2.setFont(u8g2_font_prospero_nbp_tf);
            u8g2.drawUTF8(0, nextCursorPosition, values["help"].c_str());
            u8g2.drawUTF8(93, nextCursorPosition, values["value"].c_str());
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
    populateSensorData();
}


void loop() { 
    pollSensor();
    // printSensor();
    delay(BME680_POLLING_DELAY);
    webServer.handleClient();

    drawSensor();
}
