#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <DHT.h>

// ==================== 設定エリア ====================
#define SLEEP_DURATION_SEC 60

uint8_t broadcastAddress[] = {0xC0, 0x49, 0xEF, 0xD1, 0x63, 0x40};
#define PIN_DHT_ROOM 13
#define PIN_DHT_DUCT 14
#define PIN_BATTERY  34 

// ==================== データ構造 ====================
typedef struct {
    uint8_t version;
    float roomTemp;
    float roomHum;
    float ductTemp;
    float ductHum;
    float batteryVoltage;
} SensorData;

SensorData sensorData;
DHT dhtRoom(PIN_DHT_ROOM, DHT11);
DHT dhtDuct(PIN_DHT_DUCT, DHT11);
#define LED_PIN 2

// ==================== 関数定義 ====================
void blink(int count) {
  pinMode(LED_PIN, OUTPUT);
  for(int i = 0; i < count; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
  delay(500);
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("\r\nLast Packet Send Status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
    
    blink(2); // スリープ直前の2回点滅
    
    Serial.println("Going to sleep...");
    esp_deep_sleep_start();
}

void setup() {
    Serial.begin(115200);
    
    blink(3); // 起床時の3回点滅

    dhtRoom.begin();
    dhtDuct.begin();

    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    
    // コールバック登録（必須）
    esp_now_register_send_cb(OnDataSent);

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);

    peerInfo.channel = 1;  
    peerInfo.encrypt = false;
    
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
        Serial.println("Failed to add peer");
        return;
    }
    Serial.println("Peer added successfully");
    delay(2000);

    sensorData.version = 1;
    sensorData.roomTemp = dhtRoom.readTemperature();
    sensorData.roomHum = dhtRoom.readHumidity();
    sensorData.ductTemp = dhtDuct.readTemperature();
    sensorData.ductHum = dhtDuct.readHumidity();
    
    int adcVal = analogRead(PIN_BATTERY);
    sensorData.batteryVoltage = (adcVal * 3.3 / 4095.0) * 2.0; 

    // 送信より先にスリープタイマーを設定
    esp_sleep_enable_timer_wakeup(SLEEP_DURATION_SEC * 1000000);

    // 送信実行
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &sensorData, sizeof(sensorData));
    
    if (result == ESP_OK) {
        Serial.println("Sent with success");
    } else {
        Serial.println("Error sending the data");
    }
}

void loop() {
}