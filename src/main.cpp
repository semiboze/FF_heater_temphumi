#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <DHT.h>

// ==================== 設定エリア ====================
// 送信間隔(秒) - テスト時は短く、運用時は60秒などに設定
#define SLEEP_DURATION_SEC 10

// 親機（制御側）のMACアドレス
uint8_t broadcastAddress[] = {0xC0, 0x49, 0xEF, 0xD1, 0x63, 0x40};
// ピン定義
#define PIN_DHT_ROOM 13
#define PIN_DHT_DUCT 14
#define PIN_BATTERY  34  // 電圧監視用ADCピン(分圧回路推奨)

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

// 送信結果のコールバック
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("\r\nLast Packet Send Status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
    
    // 送信が完了したら(成功・失敗に関わらず)スリープへ
    Serial.println("Going to sleep...");
    esp_deep_sleep_start();
}

void setup() {
    Serial.begin(115200);
    
    // DHT初期化
    dhtRoom.begin();
    delay(500);
    dhtDuct.begin();

    // WiFiをSTAモードにしてESP-NOW初期化
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // 送信成功コールバック登録
    esp_now_register_send_cb(OnDataSent);

    // ピア（親機）の登録
    esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
        Serial.println("Failed to add peer");
        return;
    }

    // データ計測
    sensorData.version = 1;
    sensorData.roomTemp = dhtRoom.readTemperature();
    sensorData.roomHum = dhtRoom.readHumidity();
    sensorData.ductTemp = dhtDuct.readTemperature();
    sensorData.ductHum = dhtDuct.readHumidity();
    
    // バッテリー電圧計測 (0-4095を3.3V換算、分圧比を考慮して調整必要)
    // 分圧比(例: 1/2なら2倍にするなど)を調整してください
    int adcVal = analogRead(PIN_BATTERY);
    sensorData.batteryVoltage = (adcVal * 3.3 / 4095.0) * 2.0; 

    // 送信
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &sensorData, sizeof(sensorData));
    
    if (result == ESP_OK) {
        Serial.println("Sent with success");
    } else {
        Serial.println("Error sending the data");
    }

    // スリープ設定
    esp_sleep_enable_timer_wakeup(SLEEP_DURATION_SEC * 1000000);
}

void loop() {
    // スリープ運用のため、ここは空でOK
}