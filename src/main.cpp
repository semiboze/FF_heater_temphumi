#include <Arduino.h>
#include <WiFi.h>

void setup() {
    Serial.begin(115200);
    delay(3000); // 起動ログが出終わるまで待機
    
    // WiFi.mode(WIFI_STA);
    WiFi.begin(); // WiFiスタックを起動させる
    Serial.println();
    Serial.println("\n--- MACアドレス取得用 ---");
    Serial.print("制御用ESP32のMACアドレス: ");
    Serial.println(WiFi.macAddress());
    Serial.println("--------------------------");
}

void loop() {
}