#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <DHT.h>
#include <atomic>
#include <Preferences.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "SensorConfig.h"
#include "EspNowWire.h"
#include "EspNowKeys.h"
#include <esp_sleep.h>
#include <esp_mac.h>
#include <math.h>

// ==================== 設定エリア ====================
constexpr uint8_t ESPNOW_CHANNEL = 1;
constexpr uint8_t SENSOR_READ_ATTEMPTS = 3;
// DHTライブラリの読み取りキャッシュ（2秒）より長く待って再測定する。
constexpr uint32_t SENSOR_READ_INTERVAL_MS = 2100;
constexpr uint32_t SEND_CALLBACK_TIMEOUT_MS = 3000;

uint8_t parentMac[] = {0xC0, 0x49, 0xEF, 0xD1, 0x63, 0x40};
uint8_t nowRoot[16] = {};
Preferences nowPrefs;
uint64_t sensorNonce = 0, parentNonce = 0;
constexpr uint32_t HANDSHAKE_TIMEOUT_MS = 3000;
constexpr uint32_t CONFIG_WINDOW_MS = 1500;

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


struct StoredSettings {
    int16_t roomTenths = 0;
    int16_t ductTenths = 0;
    uint16_t sleepSeconds = 60;
    uint16_t marker = 0x5353;
};
struct RadioRx {
    uint8_t bytes[sizeof(ffnow::ConfigSecureFrame)];
    uint8_t size;
};
StoredSettings sensorSettings;
QueueHandle_t configQueue = nullptr;
Preferences sensorPrefs;

bool validStored(const StoredSettings& saved) {
    ffheater::ConfigFrame frame{};
    frame.roomTenths = saved.roomTenths;
    frame.ductTenths = saved.ductTenths;
    frame.sleepSeconds = saved.sleepSeconds;
    return saved.marker == 0x5353 && ffheater::validConfig(frame);
}
void loadSettings() {
    sensorPrefs.begin("sensor-config", false);
    StoredSettings saved{};
    if (sensorPrefs.getBytesLength("settings") == sizeof(saved) &&
        sensorPrefs.getBytes("settings", &saved, sizeof(saved)) == sizeof(saved) &&
        validStored(saved)) sensorSettings = saved;
}
bool saveSettings(const StoredSettings& next) {
    if (!validStored(next)) return false;
    if (memcmp(&next, &sensorSettings, sizeof(next)) == 0) return true;
    if (sensorPrefs.putBytes("settings", &next, sizeof(next)) != sizeof(next)) return false;
    StoredSettings readback{};
    if (sensorPrefs.getBytes("settings", &readback, sizeof(readback)) != sizeof(readback) ||
        memcmp(&readback, &next, sizeof(next)) != 0) return false;
    sensorSettings = next;
    return true;
}
void OnConfigRecv(const uint8_t* mac, const uint8_t* bytes, int len) {
    if (!configQueue || !ffnow::allowedMac(mac, parentMac) || !bytes ||
        len <= 0 || len > int(sizeof(RadioRx::bytes))) return;
    RadioRx received{};
    received.size = uint8_t(len);
    memcpy(received.bytes, bytes, len);
    xQueueSend(configQueue, &received, 0);
}
std::atomic<bool> sendCompleted{false};
std::atomic<esp_now_send_status_t> sendStatus{ESP_NOW_SEND_FAIL};
void replyToConfig(const ffnow::ConfigSecureFrame& incoming) {
    ffnow::ConfigSecureFrame reply{};
    reply.kind = ffnow::REPORT;
    reply.sensorNonce = sensorNonce;
    reply.parentNonce = parentNonce;
    reply.config.op = ffheater::CONFIG_REPORT;
    reply.config.requestId = incoming.config.requestId;
    reply.config.status = ffheater::CONFIG_OK;
    if (incoming.config.op == ffheater::CONFIG_SET) {
        if (!ffheater::validConfig(incoming.config)) {
            reply.config.status = ffheater::CONFIG_BAD_VALUE;
        } else {
            StoredSettings next{};
            next.roomTenths = incoming.config.roomTenths;
            next.ductTenths = incoming.config.ductTenths;
            next.sleepSeconds = incoming.config.sleepSeconds;
            if (!saveSettings(next)) reply.config.status = ffheater::CONFIG_SAVE_ERROR;
        }
    }
    reply.config.roomTenths = sensorSettings.roomTenths;
    reply.config.ductTenths = sensorSettings.ductTenths;
    reply.config.sleepSeconds = sensorSettings.sleepSeconds;
    sendCompleted.store(false);
    esp_now_send(parentMac, reinterpret_cast<const uint8_t*>(&reply), sizeof(reply));
    const uint32_t started = millis();
    while (!sendCompleted.load() && millis() - started < 300) delay(5);
}

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

    // Preserve NaN; only finite temperatures receive calibration offsets.
    if (isfinite(sensorData.roomTemp)) sensorData.roomTemp += sensorSettings.roomTenths / 10.0f;
    if (isfinite(sensorData.ductTemp)) sensorData.ductTemp += sensorSettings.ductTenths / 10.0f;

    // 非有限値はすべて NAN に統一し、親機が項目ごとに異常を判別できるようにする。
    if (!isfinite(sensorData.roomTemp)) sensorData.roomTemp = NAN;
    if (!isfinite(sensorData.roomHum)) sensorData.roomHum = NAN;
    if (!isfinite(sensorData.ductTemp)) sensorData.ductTemp = NAN;
    if (!isfinite(sensorData.ductHum)) sensorData.ductHum = NAN;
}

// Deep-sleep timer wakeups skip maintenance delay to preserve battery life.
// Cold boot/reset opens a 15-second serial configuration window.
String maintenanceLine;
bool nextRestart = false;
void sensorNowCommand(const String& cmd) {
    if (cmd == "NOW STATUS") {
        Serial.print("Key present: "); Serial.println(nowPrefs.getBytesLength("root") == 16 ? "YES" : "NO");
        Serial.print("Parent STA MAC: ");
        for (int i=0;i<6;++i) { if(i) Serial.print(':'); if (parentMac[i] < 16) Serial.print('0'); Serial.print(parentMac[i], HEX); }
        Serial.println();
    } else if (cmd.startsWith("NOW SET ")) {
        uint8_t key[16] = {};
        if (!ffnow::parseHex(cmd.c_str() + 8, 16, key)) {
            Serial.println("ERROR: key must be exactly 32 hex characters"); return;
        }
        if (!ffnow::saveKey(nowPrefs, key)) {
            Serial.println("ERROR: NVS save failed"); return;
        }
        memcpy(nowRoot, key, sizeof(key));
        memset(key, 0, sizeof(key));
        Serial.println("Key saved. Restarting sensor (key will never be printed here).");
        nextRestart = true;
    } else if (cmd.startsWith("NOW PARENT ")) {
        uint8_t mac[6] = {};
        if (!ffnow::parseMac(cmd.c_str() + 11, mac)) {
            Serial.println("ERROR: enter MAC as AA:BB:CC:DD:EE:FF"); return;
        }
        if (nowPrefs.putBytes("parent", mac, 6) != 6) {
            Serial.println("ERROR: NVS MAC save failed"); return;
        }
        memcpy(parentMac, mac, 6);
        Serial.println("Parent MAC saved. Restarting sensor.");
        nextRestart = true;
    } else if (cmd.length()) Serial.println("Commands: NOW STATUS / NOW SET <32-hex-key> / NOW PARENT <MAC>");
}
void pollSensorSerial() {
    while (Serial.available()) {
        char c = char(Serial.read());
        if (c == '\r') continue;
        if (c == '\n') { sensorNowCommand(maintenanceLine); maintenanceLine = ""; }
        else if (c >= 32 && c <= 126 && maintenanceLine.length() < 90) maintenanceLine += c;
    }
    if (nextRestart) { Serial.flush(); delay(120); ESP.restart(); }
}
bool sendAndWait(const uint8_t* payload, size_t size, uint32_t timeoutMs) {
    sendCompleted.store(false);
    sendStatus.store(ESP_NOW_SEND_FAIL);
    if (esp_now_send(parentMac, payload, size) != ESP_OK) return false;
    const uint32_t started = millis();
    while (!sendCompleted.load() && millis() - started < timeoutMs) delay(5);
    return sendCompleted.load() && sendStatus.load() == ESP_NOW_SEND_SUCCESS;
}
void setup() {
    Serial.begin(115200);
    loadSettings();
    nowPrefs.begin("ffnow-sec", false);
    if (nowPrefs.getBytesLength("parent") == 6) {
        uint8_t savedMac[6] = {};
        if (nowPrefs.getBytes("parent", savedMac, 6) == 6 && ffnow::validUnicastMac(savedMac))
            memcpy(parentMac, savedMac, 6);
        else { Serial.println("ERROR: invalid parent MAC in NVS");
            esp_sleep_enable_timer_wakeup(300000000ULL); esp_deep_sleep_start(); }
    }
    const bool keyReady = ffnow::loadKey(nowPrefs, nowRoot);
    uint8_t ownMac[6] = {};
    esp_read_mac(ownMac, ESP_MAC_WIFI_STA);
    Serial.printf("Sensor STA MAC=%02X:%02X:%02X:%02X:%02X:%02X\n",
        ownMac[0],ownMac[1],ownMac[2],ownMac[3],ownMac[4],ownMac[5]);
    if (esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_TIMER) {
        Serial.println("USB maintenance: NOW STATUS / NOW SET <32HEX> / NOW PARENT <MAC>");
        const uint32_t windowMs = keyReady ? 15000 : 120000;
        const uint32_t windowStart = millis();
        while (millis() - windowStart < windowMs) { pollSensorSerial(); delay(10); }
    }
    if (!ffnow::loadKey(nowPrefs, nowRoot)) {
        Serial.println("No ESP-NOW key. Sleeping 5min; USB reset reopens key entry.");
        esp_sleep_enable_timer_wakeup(300000000ULL);
        esp_deep_sleep_start();
    }
    configQueue = xQueueCreate(4, sizeof(RadioRx));
    // The configured interval applies even on initialization/measurement failure.
    esp_sleep_enable_timer_wakeup(uint64_t(sensorSettings.sleepSeconds) * 1000000ULL);

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

    if (!configQueue || esp_now_register_recv_cb(OnConfigRecv) != ESP_OK) {
        Serial.println("Failed to register config receiver");
        sleepNow();
        return;
    }
    if (esp_now_register_send_cb(OnDataSent) != ESP_OK) {
        Serial.println("Failed to register ESP-NOW callback");
        sleepNow();
        return;
    }

    uint8_t pmk[16] = {}, lmk[16] = {};
    if (!ffnow::deriveKeys(nowRoot, pmk, lmk) || esp_now_set_pmk(pmk) != ESP_OK) {
        Serial.println("ESP-NOW key setup failed"); sleepNow(); return;
    }
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, parentMac, 6);
    peerInfo.channel = ESPNOW_CHANNEL;
    peerInfo.ifidx = WIFI_IF_STA;
    peerInfo.encrypt = true;
    memcpy(peerInfo.lmk, lmk, 16);
    memset(pmk, 0, sizeof(pmk)); memset(lmk, 0, sizeof(lmk));
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Encrypted peer add failed"); sleepNow(); return;
    }
    // Send hello first. Only accept a temperature packet after a one-use,
    // freshly generated parent challenge. This prevents captured frame replay.
    sensorNonce = ffnow::nonce();
    ffnow::HelloFrame hello{};
    hello.sensorNonce = sensorNonce;
    if (!sendAndWait(reinterpret_cast<const uint8_t*>(&hello), sizeof(hello), SEND_CALLBACK_TIMEOUT_MS)) {
        Serial.println("Encrypted HELLO delivery failed"); sleepNow(); return;
    }
    const uint32_t start = millis();
    bool challenged = false;
    while (millis() - start < HANDSHAKE_TIMEOUT_MS) {
        RadioRx rx{};
        if (xQueueReceive(configQueue, &rx, pdMS_TO_TICKS(25)) != pdTRUE) continue;
        ffnow::ChallengeFrame challenge{};
        if (!ffnow::parse(rx.bytes, rx.size, challenge, ffnow::CHALLENGE) ||
            !challenge.parentNonce || challenge.sensorNonce != sensorNonce) continue;
        parentNonce = challenge.parentNonce;
        challenged = true;
        break;
    }
    if (!challenged) { Serial.println("No valid challenge; sample not sent"); sleepNow(); return; }
    ffnow::SampleFrame sample{};
    sample.sensorNonce = sensorNonce;
    sample.parentNonce = parentNonce;
    memcpy(sample.sensorData, &sensorData, sizeof(sensorData));
    if (!sendAndWait(reinterpret_cast<const uint8_t*>(&sample), sizeof(sample), SEND_CALLBACK_TIMEOUT_MS)) {
        Serial.println("Encrypted sample delivery failed"); sleepNow(); return;
    }
    Serial.println("Encrypted sample transmitted");
    // Sensor settings accepted only during this wakeup with the matching nonce.
    const uint32_t listenAt = millis();
    while (millis() - listenAt < CONFIG_WINDOW_MS) {
        RadioRx rx{};
        if (xQueueReceive(configQueue, &rx, pdMS_TO_TICKS(25)) != pdTRUE) continue;
        ffnow::ConfigSecureFrame incoming{};
        if (!ffnow::parse(rx.bytes, rx.size, incoming, ffnow::CONFIG) ||
            !ffnow::matchesChallenge(incoming.sensorNonce, incoming.parentNonce,
                sensorNonce, parentNonce) ||
            incoming.config.magic != ffheater::CONFIG_MAGIC ||
            incoming.config.version != ffheater::CONFIG_VERSION ||
            (incoming.config.op != ffheater::CONFIG_GET &&
             incoming.config.op != ffheater::CONFIG_SET)) continue;
        replyToConfig(incoming);
        break;
    }
    esp_sleep_enable_timer_wakeup(uint64_t(sensorSettings.sleepSeconds) * 1000000ULL);
    sleepNow();
}

void loop() {
}
