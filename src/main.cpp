#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <DHT.h>
#include <atomic>
#include <math.h>

// ==================== 設定エリア ====================
#define SLEEP_DURATION_SEC 60
constexpr uint8_t ESPNOW_CHANNEL = 1;
constexpr uint8_t SENSOR_READ_ATTEMPTS = 3;
// DHTライブラリの読み取りキャッシュ（2秒）より長く待って再測定する。
constexpr uint32_t SENSOR_READ_INTERVAL_MS = 2100;
constexpr uint32_t SEND_CALLBACK_TIMEOUT_MS = 3000;

uint8_t broadcastAddress[] = {0xC0, 0x49, 0xEF, 0xD1, 0x63, 0x40};
#define PIN_DHT_ROOM 13
#define PIN_DHT_DUCT 14
#define PIN_BATTERY  34

// ==================== データ構造 ====================
// version 1 のバイナリ形式を維持。取得できない項目は NAN を送る。
typedef struct {
    uint8_t version;
    float roomTemp;
    float roomHum;
    float ductTemp;
    float ductHum;
    float batteryVoltage;
} SensorData;
static_assert(sizeof(SensorData) == 24, "SensorData version 1 layout changed");

SensorData sensorData = {1, NAN, NAN, NAN, NAN, NAN};
DHT dhtRoom(PIN_DHT_ROOM, DHT11);
DHT dhtDuct(PIN_DHT_DUCT, DHT11);
#define LED_PIN 2

std::atomic<bool> sendCompleted{false};
std::atomic<esp_now_send_status_t> sendStatus{ESP_NOW_SEND_FAIL};

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

void sleepNow() {
    blink(2); // スリープ直前の2回点滅（Wi-Fiコールバック外で実行）
    Serial.println("Going to sleep...");
    Serial.flush();
    esp_deep_sleep_start();
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    (void)mac_addr;
    // Wi-Fiタスクでは結果の保存だけを行う。
    sendStatus.store(status);
    sendCompleted.store(true);
}

void readSensors() {
    // 起床後は必ず NAN から開始し、前回値や 0 度で故障を隠さない。
    sensorData.roomTemp = NAN;
    sensorData.roomHum = NAN;
    sensorData.ductTemp = NAN;
    sensorData.ductHum = NAN;

    // センサーの起動と、ライブラリの初回測定を待つ。
    delay(SENSOR_READ_INTERVAL_MS);
    for (uint8_t attempt = 0; attempt < SENSOR_READ_ATTEMPTS; attempt++) {
        // 同じ起床内で正常に読めたセンサーは保持し、失敗した側だけ再測定。
        if (!isfinite(sensorData.roomTemp)) {
            sensorData.roomTemp = dhtRoom.readTemperature();
            sensorData.roomHum = dhtRoom.readHumidity();
        }
        if (!isfinite(sensorData.ductTemp)) {
            sensorData.ductTemp = dhtDuct.readTemperature();
            sensorData.ductHum = dhtDuct.readHumidity();
        }
        // 湿度の取得成否は、温度の取得成否に影響させない。
        if (isfinite(sensorData.roomTemp) && isfinite(sensorData.ductTemp)) {
            break;
        }
        if (attempt + 1 < SENSOR_READ_ATTEMPTS) {
            delay(SENSOR_READ_INTERVAL_MS);
        }
    }

    // 非有限値はすべて NAN に統一し、親機が項目ごとに異常を判別できるようにする。
    if (!isfinite(sensorData.roomTemp)) sensorData.roomTemp = NAN;
    if (!isfinite(sensorData.roomHum)) sensorData.roomHum = NAN;
    if (!isfinite(sensorData.ductTemp)) sensorData.ductTemp = NAN;
    if (!isfinite(sensorData.ductHum)) sensorData.ductHum = NAN;
}

void setup() {
    Serial.begin(115200);
    // 初期化・送信に失敗した場合も必ず60秒後に再起床する。
    esp_sleep_enable_timer_wakeup(uint64_t(SLEEP_DURATION_SEC) * 1000000ULL);

    blink(3); // 起床時の3回点滅

    dhtRoom.begin();
    dhtDuct.begin();
    readSensors();

    int adcVal = analogRead(PIN_BATTERY);
    sensorData.batteryVoltage = (adcVal * 3.3 / 4095.0) * 2.0;

    if (!WiFi.mode(WIFI_STA) ||
        esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
        Serial.println("Failed to set Wi-Fi channel");
        sleepNow();
        return;
    }
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        sleepNow();
        return;
    }

    if (esp_now_register_send_cb(OnDataSent) != ESP_OK) {
        Serial.println("Failed to register ESP-NOW callback");
        sleepNow();
        return;
    }

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);

    peerInfo.channel = ESPNOW_CHANNEL;
    peerInfo.ifidx = WIFI_IF_STA;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK){
        Serial.println("Failed to add peer");
        sleepNow();
        return;
    }

    // 両温度が NAN でも送信し、親機にセンサー故障を明示する。
    sendCompleted.store(false);
    esp_err_t result = esp_now_send(broadcastAddress,
                                   reinterpret_cast<uint8_t *>(&sensorData),
                                   sizeof(sensorData));
    if (result != ESP_OK) {
        Serial.printf("Error sending the data: %d\n", result);
        sleepNow();
        return;
    }

    const uint32_t sendStartedAt = millis();
    while (!sendCompleted.load() &&
           uint32_t(millis() - sendStartedAt) < SEND_CALLBACK_TIMEOUT_MS) {
        delay(10);
    }
    if (sendCompleted.load()) {
        Serial.println(sendStatus.load() == ESP_NOW_SEND_SUCCESS
                           ? "Delivery Success" : "Delivery Fail");
    } else {
        Serial.println("ESP-NOW send callback timed out");
    }
    sleepNow();
}

void loop() {
}
