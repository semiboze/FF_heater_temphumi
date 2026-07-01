#include <Arduino.h>
#include <WiFi.h>

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    Serial.println();
    Serial.print("制御用ESP32のMACアドレス: ");
    Serial.println(WiFi.macAddress());
}

void loop() {
    // 何もしません
}