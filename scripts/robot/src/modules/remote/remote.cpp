#include "remote.h"

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <cstring>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <Preferences.h>

#include "../../../config.h"
#include "../schedule/schedule.h"
#include "../moving/moving.h"

// NVS namespace "runtime_cfg": lưu base_pwm, gain, ngưỡng khoảng cách, PWM tối thiểu.
static Preferences sRuntimePrefs;
// Giá trị khởi tạo lần đầu lấy từ config.h (hằng compile-time).
static RuntimeConfig sRuntimeConfig = {
    BASE_PWM,
    LEFT_GAIN,
    RIGHT_GAIN,
    WALL_DISTANCE_CM,
    END_DISTANCE_CM,
    MIN_START_PWM_L,
    MIN_START_PWM_R,
};

// Đọc khóa đã lưu trong NVS và nạp vào RAM (chế độ chỉ đọc).
void setupRuntimeConfig() {
    sRuntimePrefs.begin("runtime_cfg", true);
    sRuntimeConfig.basePwm = sRuntimePrefs.getInt("base_pwm", BASE_PWM);
    sRuntimeConfig.leftGain = sRuntimePrefs.getFloat("left_gain", LEFT_GAIN);
    sRuntimeConfig.rightGain = sRuntimePrefs.getFloat("right_gain", RIGHT_GAIN);
    sRuntimeConfig.wallDistanceCm = sRuntimePrefs.getFloat("wall_cm", WALL_DISTANCE_CM);
    sRuntimeConfig.endDistanceCm = sRuntimePrefs.getFloat("end_cm", END_DISTANCE_CM);
    sRuntimeConfig.minStartPwmL = sRuntimePrefs.getInt("min_l", MIN_START_PWM_L);
    sRuntimeConfig.minStartPwmR = sRuntimePrefs.getInt("min_r", MIN_START_PWM_R);
    sRuntimePrefs.end();
}

// Trả về cấu hình đang dùng trong RAM.
const RuntimeConfig& getRuntimeConfig() {
    return sRuntimeConfig;
}

// Gán toàn bộ struct (sau khi nhận update_config từ WebSocket).
void setRuntimeConfig(const RuntimeConfig& cfg) {
    sRuntimeConfig = cfg;
}

// Ghi cấu hình hiện tại xuống NVS để giữ sau khi tắt nguồn.
void saveRuntimeConfig() {
    sRuntimePrefs.begin("runtime_cfg", false);
    sRuntimePrefs.putInt("base_pwm", sRuntimeConfig.basePwm);
    sRuntimePrefs.putFloat("left_gain", sRuntimeConfig.leftGain);
    sRuntimePrefs.putFloat("right_gain", sRuntimeConfig.rightGain);
    sRuntimePrefs.putFloat("wall_cm", sRuntimeConfig.wallDistanceCm);
    sRuntimePrefs.putFloat("end_cm", sRuntimeConfig.endDistanceCm);
    sRuntimePrefs.putInt("min_l", sRuntimeConfig.minStartPwmL);
    sRuntimePrefs.putInt("min_r", sRuntimeConfig.minStartPwmR);
    sRuntimePrefs.end();
}

// Phần WebSocket / HTTP nội bộ (ẩn linkage).
namespace {
constexpr const char* kApSsid = "Robot lau nha";
constexpr const char* kApPass = "12345678";

AsyncWebServer gServer(80);
AsyncWebSocket gWs("/ws");

bool gRemoteEnabled = true;

// --- Wi-Fi STA (lưu NVS namespace "config_wifi", khớp app: POST /config/wifi) ---
enum class WifiConnState { Idle, Requested, Pending };
WifiConnState sWifiConnState = WifiConnState::Idle;
String sWifiPendingSsid;
String sWifiPendingPass;
unsigned long sWifiConnStartMs = 0;
bool sSoftApUp = false;
static Preferences sWifiCfgPrefs;
constexpr unsigned long kWifiStaTimeoutMs = 20000;
static String sWifiPostAccum;

// UDP discovery — khớp app RobotUdpDiscovery (port 4210, "ROBOT_DISCOVER" → "ROBOT_HERE ...").
constexpr uint16_t kDiscoveryUdpPort = 4210;
constexpr char kDiscoveryRequest[] = "ROBOT_DISCOVER";

WiFiUDP sDiscoveryUdp;
bool sUdpDiscoveryStarted = false;

void stopUdpDiscoveryIfNeeded() {
    if (!sUdpDiscoveryStarted) {
        return;
    }
    sDiscoveryUdp.stop();
    sUdpDiscoveryStarted = false;
}

void startUdpDiscoveryIfNeeded() {
    if (sUdpDiscoveryStarted || WiFi.status() != WL_CONNECTED) {
        return;
    }
    if (sDiscoveryUdp.begin(kDiscoveryUdpPort)) {
        sUdpDiscoveryStarted = true;
        Serial.print(F("[REMOTE] UDP discovery lang nghe port "));
        Serial.println(kDiscoveryUdpPort);
    } else {
        Serial.println(F("[REMOTE] UDP discovery mo port that bai"));
    }
}

void handleUdpDiscoveryRequest() {
    if (!sUdpDiscoveryStarted || WiFi.status() != WL_CONNECTED) {
        return;
    }

    const int packetSize = sDiscoveryUdp.parsePacket();
    if (packetSize <= 0) {
        return;
    }

    char buffer[64];
    const int readLen = sDiscoveryUdp.read(buffer, static_cast<int>(sizeof(buffer)) - 1);
    if (readLen <= 0) {
        return;
    }
    buffer[readLen] = '\0';

    if (strcmp(buffer, kDiscoveryRequest) != 0) {
        return;
    }

    const IPAddress replyIp = sDiscoveryUdp.remoteIP();
    const uint16_t replyPort = sDiscoveryUdp.remotePort();

    const IPAddress localIp = WiFi.localIP();
    const String response = String("ROBOT_HERE ip=") + localIp.toString() + " http=80";

    sDiscoveryUdp.beginPacket(replyIp, replyPort);
    sDiscoveryUdp.print(response);
    sDiscoveryUdp.endPacket();

    Serial.print(F("[REMOTE] Da tra loi UDP discovery toi "));
    Serial.println(replyIp);
}

bool loadStoredWifiCredentials(String& ssid, String& pass) {
    sWifiCfgPrefs.begin("config_wifi", true);
    ssid = sWifiCfgPrefs.getString("ssid", "");
    pass = sWifiCfgPrefs.getString("pass", "");
    sWifiCfgPrefs.end();
    return ssid.length() > 0;
}

void persistWifiCredentials(const String& ssid, const String& pass) {
    sWifiCfgPrefs.begin("config_wifi", false);
    sWifiCfgPrefs.putString("ssid", ssid);
    sWifiCfgPrefs.putString("pass", pass);
    sWifiCfgPrefs.end();
}

void restartWebServer() {
    gServer.end();
    delay(10);
    gServer.begin();
}

void ensureSoftAp() {
    WiFi.mode(WIFI_AP_STA);
    if (!sSoftApUp) {
        WiFi.softAP(kApSsid, kApPass);
        sSoftApUp = true;
        Serial.print(F("[REMOTE] AP IP: "));
        Serial.println(WiFi.softAPIP());
    }
}

const char* robotStatusToCode(RobotStatus st) {
    switch (st) {
        case RobotStatus::SLEEPING:
            return "SLEEPING";
        case RobotStatus::WORKING:
            return "RUNNING";
        case RobotStatus::GOING_HOME:
            return "GOING_HOME";
        case RobotStatus::ERROR:
            return "ERROR";
        default:
            return "UNKNOWN";
    }
}

void sendJson(AsyncWebSocketClient* client, JsonDocument& doc) {
    String payload;
    serializeJson(doc, payload);
    if (client != nullptr) {
        client->text(payload);
    } else {
        gWs.textAll(payload);
    }
}

void sendAck(AsyncWebSocketClient* client, const char* action, bool ok, const char* message) {
    StaticJsonDocument<256> doc;
    doc["type"] = "ack";
    doc["action"] = action;
    doc["ok"] = ok;
    doc["message"] = message;
    sendJson(client, doc);
}

void sendError(AsyncWebSocketClient* client, const char* message) {
    StaticJsonDocument<192> doc;
    doc["type"] = "error";
    doc["message"] = message;
    sendJson(client, doc);
}

void sendStatusPayload(AsyncWebSocketClient* client, const char* type) {
    StaticJsonDocument<256> doc;
    doc["type"] = type;
    doc["status"] = robotStatusToCode(g_robotStatus);
    doc["busy"] = (g_robotStatus == RobotStatus::WORKING || g_robotStatus == RobotStatus::GOING_HOME);
    doc["remoteEnabled"] = gRemoteEnabled;
    sendJson(client, doc);
}

void processWifiConnectionTask() {
    if (sWifiConnState == WifiConnState::Requested) {
        stopUdpDiscoveryIfNeeded();
        WiFi.mode(WIFI_AP_STA);
        if (!sSoftApUp) {
            WiFi.softAP(kApSsid, kApPass);
            sSoftApUp = true;
        }
        WiFi.begin(sWifiPendingSsid.c_str(), sWifiPendingPass.c_str());
        sWifiConnStartMs = millis();
        sWifiConnState = WifiConnState::Pending;
        Serial.printf("[REMOTE] Dang ket noi Wi-Fi STA: %s\n", sWifiPendingSsid.c_str());
        return;
    }

    if (sWifiConnState != WifiConnState::Pending) {
        return;
    }

    if (WiFi.status() == WL_CONNECTED) {
        persistWifiCredentials(sWifiPendingSsid, sWifiPendingPass);
        WiFi.softAPdisconnect(true);
        sSoftApUp = false;
        WiFi.mode(WIFI_STA);
        restartWebServer();
        sWifiConnState = WifiConnState::Idle;
        Serial.print(F("[REMOTE] Wi-Fi STA OK, IP: "));
        Serial.println(WiFi.localIP());
        startUdpDiscoveryIfNeeded();
        sendStatusPayload(nullptr, "status");
        return;
    }

    if (millis() - sWifiConnStartMs >= kWifiStaTimeoutMs) {
        stopUdpDiscoveryIfNeeded();
        WiFi.disconnect(false, false);
        sWifiConnState = WifiConnState::Idle;
        Serial.println(F("[REMOTE] Ket noi Wi-Fi STA that bai, giu che do AP."));
        ensureSoftAp();
        restartWebServer();
        sendStatusPayload(nullptr, "status");
    }
}

void sendConfigPayload(AsyncWebSocketClient* client) {
    StaticJsonDocument<320> doc;
    doc["type"] = "config";
    JsonObject cfg = doc.createNestedObject("config");
    const RuntimeConfig& runtime = getRuntimeConfig();
    cfg["basePwm"] = runtime.basePwm;
    cfg["leftGain"] = runtime.leftGain;
    cfg["rightGain"] = runtime.rightGain;
    cfg["wallDistanceCm"] = runtime.wallDistanceCm;
    cfg["endDistanceCm"] = runtime.endDistanceCm;
    cfg["minStartPwmL"] = runtime.minStartPwmL;
    cfg["minStartPwmR"] = runtime.minStartPwmR;
    // Bí danh tương thích bản app mobile cũ (goSpeed / wallStopDistanceCm).
    cfg["goSpeed"] = runtime.basePwm;
    cfg["turnSpeed"] = runtime.basePwm;
    cfg["wallStopDistanceCm"] = runtime.wallDistanceCm;
    sendJson(client, doc);
}

void handleUpdateConfig(AsyncWebSocketClient* client, JsonObject cfg) {
    RuntimeConfig next = getRuntimeConfig();
    if (cfg["basePwm"].is<int>()) next.basePwm = cfg["basePwm"].as<int>();
    if (cfg["goSpeed"].is<int>()) next.basePwm = cfg["goSpeed"].as<int>();
    if (cfg["leftGain"].is<float>() || cfg["leftGain"].is<int>()) next.leftGain = cfg["leftGain"].as<float>();
    if (cfg["rightGain"].is<float>() || cfg["rightGain"].is<int>()) next.rightGain = cfg["rightGain"].as<float>();
    if (cfg["wallDistanceCm"].is<float>() || cfg["wallDistanceCm"].is<int>()) next.wallDistanceCm = cfg["wallDistanceCm"].as<float>();
    if (cfg["wallStopDistanceCm"].is<float>() || cfg["wallStopDistanceCm"].is<int>()) next.wallDistanceCm = cfg["wallStopDistanceCm"].as<float>();
    if (cfg["endDistanceCm"].is<float>() || cfg["endDistanceCm"].is<int>()) next.endDistanceCm = cfg["endDistanceCm"].as<float>();
    if (cfg["minStartPwmL"].is<int>()) next.minStartPwmL = cfg["minStartPwmL"].as<int>();
    if (cfg["minStartPwmR"].is<int>()) next.minStartPwmR = cfg["minStartPwmR"].as<int>();
    setRuntimeConfig(next);
    saveRuntimeConfig();
    sendAck(client, "update_config", true, "Da luu config runtime");
    sendConfigPayload(client);
}

void handleCommand(AsyncWebSocketClient* client, const String& command) {
    if (command == "start_work") {
        gRemoteEnabled = true;
        g_robotStatus = RobotStatus::WORKING;
        sendAck(client, "command", true, "Robot se bat dau lam viec");
        sendStatusPayload(nullptr, "status");
        return;
    }

    if (command == "stop") {
        stop();
        g_robotStatus = RobotStatus::SLEEPING;
        sendAck(client, "command", true, "Robot da dung");
        sendStatusPayload(nullptr, "status");
        return;
    }

    if (command == "power_on") {
        gRemoteEnabled = true;
        if (g_robotStatus == RobotStatus::ERROR) {
            g_robotStatus = RobotStatus::SLEEPING;
        }
        sendAck(client, "command", true, "Robot da bat");
        sendStatusPayload(nullptr, "status");
        return;
    }

    if (command == "power_off") {
        gRemoteEnabled = false;
        stop();
        g_robotStatus = RobotStatus::SLEEPING;
        sendAck(client, "command", true, "Robot da tat");
        sendStatusPayload(nullptr, "status");
        return;
    }

    if (command == "go_home") {
        sendAck(client, "command", false, "Go home chua hoan thanh module");
        return;
    }

    sendError(client, "Lenh khong duoc ho tro");
}

void handleSocketTextMessage(AsyncWebSocketClient* client, const String& text) {
    DynamicJsonDocument doc(1024);
    const DeserializationError err = deserializeJson(doc, text);
    if (err) {
        sendError(client, "JSON khong hop le");
        return;
    }

    JsonObject root = doc.as<JsonObject>();
    const String type = root["type"] | "";
    if (type == "get_status") {
        sendStatusPayload(client, "status");
        return;
    }
    if (type == "get_config") {
        sendConfigPayload(client);
        return;
    }
    if (type == "command") {
        handleCommand(client, root["command"] | "");
        return;
    }
    if (type == "update_config") {
        if (!root["config"].is<JsonObject>()) {
            sendError(client, "Thieu object config");
            return;
        }
        handleUpdateConfig(client, root["config"].as<JsonObject>());
        return;
    }
    sendError(client, "Loai goi tin khong duoc ho tro");
}

void onWebSocketEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                      AwsEventType type, void* arg, uint8_t* data, size_t len) {
    (void)server;
    if (type == WS_EVT_CONNECT) {
        sendStatusPayload(client, "connected");
        sendConfigPayload(client);
        return;
    }
    if (type != WS_EVT_DATA) return;

    AwsFrameInfo* info = (AwsFrameInfo*)arg;
    if (info == nullptr || !info->final || info->index != 0 || info->len != len || info->opcode != WS_TEXT) {
        return;
    }

    String text;
    text.reserve(len);
    for (size_t i = 0; i < len; ++i) text += (char)data[i];
    handleSocketTextMessage(client, text);
}
}  // namespace (WebSocket + AP)

// Bật AP Wi‑Fi, HTTP /health, POST /config/wifi và WebSocket /ws.
void setupRemote() {
    setupRuntimeConfig();

    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(kApSsid, kApPass);
    sSoftApUp = true;

    gServer.on("/health", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(200, "application/json", "{\"status\":\"ok\"}");
    });

    gServer.on(
        "/config/wifi",
        HTTP_POST,
        [](AsyncWebServerRequest* request) {},
        nullptr,
        [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
            if (index == 0) {
                sWifiPostAccum = "";
            }
            for (size_t i = 0; i < len; ++i) {
                sWifiPostAccum += static_cast<char>(data[i]);
            }
            if (index + len < total) {
                return;
            }

            DynamicJsonDocument doc(512);
            const DeserializationError err = deserializeJson(doc, sWifiPostAccum);
            if (err) {
                request->send(400, "application/json", "{\"error\":\"bad_json\"}");
                return;
            }

            JsonObject root = doc.as<JsonObject>();
            if (!root["ssid"].is<String>() || !root["pass"].is<String>()) {
                request->send(400, "application/json", "{\"error\":\"missing_ssid_or_pass\"}");
                return;
            }

            const String ssid = root["ssid"].as<String>();
            const String pass = root["pass"].as<String>();
            if (ssid.length() == 0) {
                request->send(400, "application/json", "{\"error\":\"missing_ssid_or_pass\"}");
                return;
            }

            if (sWifiConnState != WifiConnState::Idle) {
                request->send(409, "application/json", "{\"error\":\"wifi_connect_in_progress\"}");
                return;
            }

            sWifiPendingSsid = ssid;
            sWifiPendingPass = pass;
            sWifiConnState = WifiConnState::Requested;
            request->send(202, "application/json", "{\"ok\":true,\"status\":\"queued\"}");
        });

    gWs.onEvent(onWebSocketEvent);
    gServer.addHandler(&gWs);
    gServer.begin();

    Serial.println(F("[REMOTE] Module initialized"));
    Serial.print(F("[REMOTE] AP IP: "));
    Serial.println(WiFi.softAPIP());

    String bootSsid;
    String bootPass;
    if (loadStoredWifiCredentials(bootSsid, bootPass)) {
        Serial.println(F("[REMOTE] Tim thay Wi-Fi da luu, dang thu ket noi STA"));
        sWifiPendingSsid = bootSsid;
        sWifiPendingPass = bootPass;
        sWifiConnState = WifiConnState::Requested;
    }
}

// Xử lý kết nối STA + dọn client WS; gọi thường xuyên từ loop().
void pollRemote() {
    processWifiConnectionTask();
    if (WiFi.status() == WL_CONNECTED) {
        startUdpDiscoveryIfNeeded();
    }
    handleUdpDiscoveryRequest();
    gWs.cleanupClients();
}

// Tra cứu cờ bật/tắt điều khiển từ xa (power_on / power_off).
bool remoteShouldRunScheduler() {
    return gRemoteEnabled;
}
