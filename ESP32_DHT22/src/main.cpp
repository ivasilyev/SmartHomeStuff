#include <map>
#include <string>

#include <Arduino.h>
#include <DHT.h>
#include <WiFiCredentials.h>
#include <WiFi.h>
#include <WebServer.h>

#ifdef __cplusplus
    extern "C" {
#endif
    uint8_t temprature_sens_read();
#ifdef __cplusplus
}
#endif
uint8_t temprature_sens_read();

#define DHTPIN 4
#define DHTTYPE DHT22  // DHT 22  (AM2302), AM2321
#define PROMETHEUS_NAMESPACE "iot"
#define POLLING_DELAY 3000


DHT dht(DHTPIN, DHTTYPE);

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
    _populateSensorData("air_humidity_percent", "Humidity, %", g);
    
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

void checkSensor() {
    Serial.println(F("DHT22 test"));
    dht.begin(); 

    float h = dht.readHumidity();
    float t = dht.readTemperature(false);
    float f = dht.readTemperature(true);

    // Check if any reads failed and exit early (to try again).
    while (isnan(h) || isnan(t) || isnan(f)) {
      Serial.println(F("Could not find a valid DHT22 sensor!"));
    }

    Serial.println(F("DHT22 OK"));
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
    setSensorData("air_temperature_celsius", dht.readTemperature(false));
    setSensorData("air_humidity_percent", dht.readHumidity());
 
    // System metrics start
    setSensorData("system_up_time_ms", millis());
    setSensorData("cpu_frequency_mhz", getCpuFrequencyMhz());
    setSensorData("cpu_temperature_celsius", (temprature_sens_read() - 32.0f) / 1.8f);
    setSensorData("memory_heap_used_bytes", ESP.getHeapSize());
    setSensorData("memory_heap_free_bytes", ESP.getFreeHeap());
    setSensorData("memory_psram_total_bytes", ESP.getPsramSize());
    setSensorData("memory_psram_free_bytes", ESP.getFreePsram());
    setSensorData("memory_flash_used_bytes", ESP.getSketchSize());
    setSensorData("memory_flash_total_bytes", ESP.getFreeSketchSpace());
    // System metrics end

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
    WiFi.setHostname("ESP32_DHT22");
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
    "        <title>DHT22 | ESP32 Web Server</title>\n"
    "    </head>\n"
    "    <body>\n"
    "        <div>\n"
    "            <h1>ESP32 Web Server</h1>\n"
    "            <div>\n"
    "                DHT22 data:\n"
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


void setup() {
    Serial.begin(9600);
    checkSensor();
    forceConnectToWiFi();
    webServer.begin();
    setupWebServer();
    populateSensorData();
}


void loop() { 
    pollSensor();
    // printSensor();
    delay(POLLING_DELAY);
    webServer.handleClient();
}
