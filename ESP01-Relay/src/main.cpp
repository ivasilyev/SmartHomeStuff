#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WiFiCredentials.h>
#include <BuiltWebPage.h>

const uint8_t RELAY_PIN = 0;
const uint8_t LED_PIN = 2;

bool isRelayEnabled = false;
String changedAt = "N/A";

ESP8266WebServer webServer(80);

void forceConnectToWiFi() {
    // WiFi.hostname("ESP01_Relay_1");
    WiFi.mode(WIFI_STA);
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

void relayOn() {
    digitalWrite(RELAY_PIN, LOW);
    digitalWrite(LED_PIN, LOW);
    isRelayEnabled = true;
}

void relayOff() {
    digitalWrite(RELAY_PIN, HIGH);
    digitalWrite(LED_PIN, LOW);
    isRelayEnabled = false;
}

void toggleRelay() {
    if (isRelayEnabled) {
        relayOff();
    }
    else {
        relayOn();
    }
}

void blinkLED() {
    digitalWrite(LED_PIN, LOW);
    digitalWrite(LED_PIN, HIGH);
    digitalWrite(LED_PIN, LOW);
}

static void webHandleNotFound() {
      String message = "File Not Found\n\n";
      message += "URI: ";
      message += webServer.uri();
      message += "\nMethod: ";
      message += (webServer.method() == HTTP_GET) ? "GET" : "POST";
      message += "\nArguments: ";
      message += webServer.args();
      message += "\n";
      for (uint8_t i = 0; i < webServer.args(); i++) {
        message += " " + webServer.argName(i) + ": " + webServer.arg(i) + "\n";
      }
      webServer.send(404, "text/plain", message);
}


void webHandleRoot() {
    Serial.println("Connected client");

    webServer.send(200, "text/html", WEBPAGE);
}

void webDebugArgs() {
    String message = "Number of args received:";
    message += webServer.args();
    message += "\n";

    for (int i = 0; i < webServer.args(); i++) {
        message += "Arg " + (String)i + " -> ";
        message += webServer.argName(i) + ": ";
        message += webServer.arg(i) + "\n";
    }

    webServer.send(200, "text/plain", message);
}

void webSendJSON() {
    Serial.println("Sending JSON");

    DynamicJsonDocument doc(1024);
    doc["type"] = "relay";
    doc["board"] = "esp-01";
    doc["isTurnedOn"] = isRelayEnabled;
    doc["changedAt"] = changedAt;
    String out;
    serializeJson(doc, out);
    webServer.send(200, "application/json", out);
}

void webHandleSetState() {
    Serial.println("Connected client");
    digitalWrite(LED_PIN, LOW);
    digitalWrite(LED_PIN, HIGH);

    if (webServer.arg("isTurnedOn") != "") {
        if (webServer.arg("isTurnedOn") == "true") {
            relayOn();
        }
        else if (webServer.arg("isTurnedOn") == "false") {
            relayOff();
        }
        if (webServer.arg("changedAt") != "") {
            changedAt = webServer.arg("changedAt");
        }
    }

    digitalWrite(LED_PIN, LOW);
    webSendJSON();
}

void setup() {
    delay(1000);
    pinMode(RELAY_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    relayOff();

    Serial.begin(115200);
    forceConnectToWiFi();

    webServer.on("/", webHandleRoot);
    webServer.on("/set-state", webHandleSetState);
    webServer.on("/get-state", webSendJSON);
    webServer.onNotFound(webHandleNotFound);
    webServer.begin();
    Serial.println("Web server online");
}

void loop() {
    webServer.handleClient();
}
