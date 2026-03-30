/*
 * ELRS Head Tracker -> SBUS Bridge
 *
 * Emulates an ELRS Backpack to receive MSP_ELRS_BACKPACK_SET_PTR
 * messages from a VRx module via ESP-NOW, converts pan/tilt/roll into
 * RC channel values, and outputs them as an SBUS stream on Serial1.
 *
 * Requires ESP32-family (ESP32, S2, S3, C3, C6) — uses ESP-IDF APIs
 * (ESP-NOW, mbedtls, HardwareSerial pin remapping) not available on ESP8266.
 *
 * Protocol details derived from:
 *   github.com/ExpressLRS/Backpack   (ESP-NOW + MSP framing)
 *   github.com/ExpressLRS/ExpressLRS (SBUS output + CRSF channel encoding)
 */
#include <ESPmDNS.h>
#include <NetworkUdp.h>
#include <ArduinoOTA.h>

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <mbedtls/md5.h>
#include "msp.h"
#include "sbus.h"

// ======================== OPTIONS ====================================
#define WIFI_SSID        "XXXXXXXXXXXXX"
#define WIFI_PASS        "XXXXXXXXXXXXX"
#define BINDING_PHRASE   "XXXXXXXXXXXXX"  // Must match your ELRS / Backpack setup
#define PTR_CH_START     1                   // First channel for Pan (Tilt=CH2, Roll=CH3)
#define DEBUG_MODE                         // Uncomment to enable Serial debug output
#define ESP_NUM_CH       3
// ======================== PIN ASSIGNMENT =============================
#define SBUS_TX_PIN      21                   // Serial1 TX GPIO for SBUS output, select any pin

static_assert(PTR_CH_START >= 1 && PTR_CH_START <= 14, "PTR_CH_START must be 1-14 (needs 3 channels)");

// ======================== DEBUG MACROS ===============================
#ifdef DEBUG_MODE
  #define DBG_INIT()  Serial.begin(115200)
  #define DBG(...)    Serial.printf(__VA_ARGS__)
#else
  #define DBG_INIT()
  #define DBG(...)
#endif

// ======================== CONSTANTS =================================
constexpr uint32_t SBUS_INTERVAL_MS = 20;    // 50 Hz SBUS frame rate
constexpr uint32_t PTR_TIMEOUT_MS   = 1000;  // Failsafe after 1 s silence

// ======================== GLOBALS ===================================
uint8_t  uid[6];
uint16_t channels[SBUS_NUM_CHANNELS];
MspParser msp;

uint32_t last_ota_time = 0;

// Written from ESP-NOW callback, read from loop — guarded by ptrMux
portMUX_TYPE ptrMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint16_t ptrCh[ESP_NUM_CH];
volatile uint32_t lastPtrMs;
volatile bool     sendHTEnable;   // Flag: VRx requested cached packets
volatile uint32_t ptrPacketCount; // Total PTR packets received
uint32_t lastSbusMs;
uint32_t lastEnableMs;
bool     wasActive;
bool     progressUpdate = false;

// ======================== UID FROM BINDING PHRASE ====================
// Replicates ELRS build system: MD5('-DMY_BINDING_PHRASE="<phrase>"')[0:6]
void generateUID(const char *phrase, uint8_t *out) {
    char buf[128];
    snprintf(buf, sizeof(buf), "-DMY_BINDING_PHRASE=\"%s\"", phrase);

    uint8_t hash[16];
    mbedtls_md5_context ctx;
    mbedtls_md5_init(&ctx);
    mbedtls_md5_starts(&ctx);
    mbedtls_md5_update(&ctx, (const uint8_t *)buf, strlen(buf));
    mbedtls_md5_finish(&ctx, hash);
    mbedtls_md5_free(&ctx);

    memcpy(out, hash, 6);
    out[0] &= ~0x01;  // Ensure unicast (clear LSB of first byte)
}

// ======================== SEND MSP VIA ESP-NOW =======================
void sendMspEspNow(uint16_t function, const uint8_t *data, uint16_t len) {
    uint8_t frame[32];
    uint8_t frameLen = mspBuildCommand(frame, sizeof(frame), function, data, len);
    if (frameLen) esp_now_send(uid, frame, frameLen);
}

void sendHeadTrackingEnable() {
    uint8_t enable = 1;
    sendMspEspNow(MSP_ELRS_SET_HEAD_TRACKING, &enable, 1);
    DBG("Sent SET_HEAD_TRACKING enable\n");
}

void sanitizeCh(uint16_t *localPtr) {
    
    int base = PTR_CH_START - 1;

    for (int i = 0; i < ESP_NUM_CH; i++) {
        if (localPtr[i] < CRSF_CHANNEL_MIN) { localPtr[i]=CRSF_CHANNEL_MIN;}
        if (localPtr[i] > CRSF_CHANNEL_MAX) { localPtr[i]=CRSF_CHANNEL_MAX;}

        if (channels[base + i] == CRSF_CHANNEL_MIN && localPtr[i] == CRSF_CHANNEL_MAX) {
            localPtr[i] = CRSF_CHANNEL_MIN;
        }
        if (channels[base + i] == CRSF_CHANNEL_MAX && localPtr[i] == CRSF_CHANNEL_MIN) {
            localPtr[i] = CRSF_CHANNEL_MAX;
        }

    }    
}


// ======================== ESP-NOW RECEIVE CALLBACK ===================
void onEspNowRecv(const esp_now_recv_info_t *info,
                         const uint8_t *data, int len) {
    const uint8_t *mac = info->src_addr;
    if (memcmp(mac, uid, 6) != 0) return;

    // Each ESP-NOW packet is one complete MSP frame
    msp.reset();
    for (int i = 0; i < len; i++) {
        if (msp.feed(data[i])) {
            if (msp.function == MSP_ELRS_SET_PTR && msp.size == 6) {
                portENTER_CRITICAL(&ptrMux);
                ptrCh[0] = min(msp.payloadU16(0), (uint16_t)2047);  // Pan
                ptrCh[1] = min(msp.payloadU16(2), (uint16_t)2047);  // Tilt
                ptrCh[2] = min(msp.payloadU16(4), (uint16_t)2047);  // Roll
                lastPtrMs = millis();
                portEXIT_CRITICAL(&ptrMux);
                ptrPacketCount+1;

            }
            else if (msp.function == MSP_ELRS_REQU_VTX_PKT) {
                sendHTEnable = true;
                DBG("VRx requested cached state\n");
            }
        }
    }
}

// ======================== SETUP ======================================
void setup() {
    DBG_INIT();
    DBG("\nESPnow -> SBUS Bridge\n");
    uint32_t startSetup = millis();

    //OTA Client for 120s
    setupOta();
    bool handleUpdate = true;
    while (handleUpdate) {
        ArduinoOTA.handle();
        if (millis() - startSetup > 120000) {
            if (!progressUpdate) {
                handleUpdate = false;
            }
            
        }
    }
    WiFi.disconnect();

    generateUID(BINDING_PHRASE, uid);
    DBG("UID: %02X:%02X:%02X:%02X:%02X:%02X\n",
        uid[0], uid[1], uid[2], uid[3], uid[4], uid[5]);

    for (int i = 0; i < SBUS_NUM_CHANNELS; i++)
        channels[i] = CRSF_CHANNEL_MID;

    // SBUS output on Serial1 — 100 kbaud, 8E2, non-inverted (idle-high), TX-only
    Serial1.begin(100000, SERIAL_8E2, -1, SBUS_TX_PIN, false);

    setupEspnow();

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, uid, 6);
    peer.channel = 0;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) != ESP_OK)
        DBG("ESP-NOW add peer failed\n");
    esp_now_register_recv_cb(onEspNowRecv);

    DBG("SBUS TX on GPIO %d  |  Channels CH%d-CH%d (Pan/Tilt/Roll)\n",
        SBUS_TX_PIN, PTR_CH_START, PTR_CH_START + 2);
    DBG("Waiting for head tracker...\n");
}


// ======================== LOOP =======================================
void loop() {

    uint32_t now = millis();

    // Respond to VRx requesting cached packets
    if (sendHTEnable) {
        sendHTEnable = false;
        sendHeadTrackingEnable();
        lastEnableMs = now;
    }

    // Snapshot shared state under lock
    portENTER_CRITICAL(&ptrMux);
    uint16_t localPtr[3] = { ptrCh[0], ptrCh[1], ptrCh[2] };
    uint32_t localPtrMs  = lastPtrMs;
    portEXIT_CRITICAL(&ptrMux);

    // Periodically send enable until we receive PTR data
    bool active = localPtrMs > 0 && ((now - localPtrMs) < PTR_TIMEOUT_MS || localPtrMs > now);
    if (!active && (now - lastEnableMs >= 1000)) {
        sendHeadTrackingEnable();
        lastEnableMs = now;
    }

    // Log state transitions
    if (active != wasActive) {
        wasActive = active;
        if (active)
            DBG("Head tracker connected (packets: %lu)\n", ptrPacketCount);
        else if (localPtrMs > 0) {
            DBG("Head tracker lost — failsafe (last packet %lums ago)\n", (now - localPtrMs));
            }
    }

    // SBUS output
    if (now - lastSbusMs >= SBUS_INTERVAL_MS) {
        lastSbusMs = now;

        if (active) {
            int base = PTR_CH_START - 1;
            
            sanitizeCh(localPtr);

            channels[base + 0] = localPtr[0];
            channels[base + 1] = localPtr[1];
            channels[base + 2] = localPtr[2];

            DBG("Ch0 %u ", channels[0]);
            DBG("Ch1 %u ", channels[1]);
            DBG("Ch2 %u ", channels[2]);
            DBG("Ch3 %u\n", channels[3]);

        }

        uint8_t flags = active ? 0 : (SBUS_FLAG_SIGLOSS | SBUS_FLAG_FAILSAFE);
        sbusWrite(Serial1, channels, flags);
    }
}


void setupEspnow() {
        // WiFi STA mode for ESP-NOW on channel 1 (hardcoded in all Backpack modules)
    WiFi.mode(WIFI_STA);
    WiFi.setTxPower(WIFI_POWER_8_5dBm);  // ~7 mW — plenty for short-range backpack link
    esp_wifi_set_protocol(WIFI_IF_STA,
        WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR);
    WiFi.begin("", "", 1);
    WiFi.disconnect();

    esp_wifi_set_mac(WIFI_IF_STA, uid);

    if (esp_now_init() != ESP_OK) {
        DBG("ESP-NOW init failed — restarting\n");
        ESP.restart();
    }

}

void setupOta() {

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    DBG("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }


  ArduinoOTA.setHostname("espnow-bridge");
  ArduinoOTA.setPassword("admin");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH) {
        type = "sketch";
      } else {  // U_SPIFFS
        type = "filesystem";
      }
      progressUpdate = true;
    })
    .onEnd([]() {
      progressUpdate = false;
      DBG("\nEnd");

    })
    .onProgress([](unsigned int progress, unsigned int total) {
      if (millis() - last_ota_time > 500) {
        DBG("Progress: %u%%\n", (progress / (total / 100)));
        last_ota_time = millis();
      }
      progressUpdate = true;
    })
    .onError([](ota_error_t error) {
      DBG("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) {
        DBG("Auth Failed");
      } else if (error == OTA_BEGIN_ERROR) {
        DBG("Begin Failed");
      } else if (error == OTA_CONNECT_ERROR) {
        DBG("Connect Failed");
      } else if (error == OTA_RECEIVE_ERROR) {
        DBG("Receive Failed");
      } else if (error == OTA_END_ERROR) {
        DBG("End Failed");
      }
    });

  ArduinoOTA.begin();

  DBG("Ready\n");
  Serial.print(WiFi.localIP());

}
