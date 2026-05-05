// ========================================
// MAIN_DRAFT.CPP - Continuous Power Mode
// Bản sao của main.cpp KHÔNG sử dụng chế độ xung (pulsed power)
// Động cơ chạy ở chế độ động lực liên tục thường để so sánh hiệu năng
// ========================================

#include <Preferences.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <math.h>

// AsyncTCP tuning for ESP32 Arduino: smaller queue, pinned task core and safer stack.
#define CONFIG_ASYNC_TCP_RUNNING_CORE 1
#define CONFIG_ASYNC_TCP_QUEUE_SIZE 32
#define CONFIG_ASYNC_TCP_STACK_SIZE 12288

#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncJson.h>
#include <ArduinoJson.h>

#pragma region PIN
// ================= CẤU HÌNH CHÂN =================
const int ENA = 13, IN1 = 12, IN2 = 14; 
const int ENB = 25, IN3 = 27, IN4 = 26; 
const int TRIG_R = 32, ECHO_R = 33;
const int TRIG_M = 4,  ECHO_M = 5;
const int TRIG_L = 18, ECHO_L = 19;
const int ENCODER_LEFT = 34;
const int ENCODER_RIGHT = 35;
const int MPU_ADDR = 0x68;

// ================= CẤU HÌNH CHÂN BỔ XUNG =================
const int signalPin = 15; 
#pragma endregion

#pragma region CONSTANTS

const float ANGLE_TOLERANCE = 2.0; // Mức sai số góc cho phép
const int DEFAULT_TURN_SPEED = 60;
const int DEFAULT_GO_SPEED = 60;
const float DEFAULT_WHEEL_RADIUS_CM = 3.4f;
const float DEFAULT_WALL_STOP_DISTANCE_CM = 20.0f;
const int DEFAULT_SLEEP_MINUTES = 10;
const float PULSES_PER_CM = 2.0; // (nghĩa là 20 xung = 10cm) Bao nhiêu xung Encoder thì xe đi được 1 cm?
const float ENCODER_PULSES_PER_REV = PULSES_PER_CM * (2.0f * PI * DEFAULT_WHEEL_RADIUS_CM);
const unsigned long TURN_TIMEOUT_BASE_MS = 1500;
const unsigned long TURN_TIMEOUT_PER_DEG_MS = 30;
const float GO_MAX_DISTANCE_CM = 2000.0f;
const float AUTO_WORK_DISTANCE_CM = 120.0f;
const float MIN_DISTANCE_BEFORE_WALL_CHECK_CM = 15.0f; // Quãng đường tối thiểu trước khi kiểm tra tường
const unsigned long STATUS_PUSH_INTERVAL_MS = 1000;
const unsigned long STRAIGHT_CONTROL_INTERVAL_MS = 40;
const float STRAIGHT_YAW_KP = 2.2f;
const float ENCODER_BALANCE_KP = 1.2f;
const float ROTATE_SPEED_KP = 1.8f;
const int ROTATE_MIN_SPEED = 35;
const int ROTATE_FINE_MIN_SPEED = 18;
const float ROTATE_FINE_ZONE_DEG = 12.0f;
const float ROTATE_ENCODER_BALANCE_KP = 2.0f;
const float GYRO_SCALE_DPS = 131.0f;
const unsigned long GYRO_WARMUP_MS = 2000;
const int GYRO_CALIB_SAMPLES = 2000;
const float GYRO_LPF_ALPHA = 0.38f;
const float GYRO_DEADBAND_DPS = 0.03f;
const float GYRO_MAX_VALID_DPS = 300.0f;
const float GYRO_STATIONARY_DPS = 0.10f;
const int GYRO_STATIONARY_COUNT_REQUIRED = 80;
const float GYRO_BIAS_ADAPT_ALPHA = 0.0010f;
const float GYRO_MOTION_ENTER_DPS = 0.70f;
const float GYRO_MOTION_EXIT_DPS = 0.25f;
const float DEFAULT_ROTATE_REVERSE_THRESHOLD_DEG = 8.0f;
const float DEFAULT_ROTATE_TOLERANCE_DEG = 2.0f;
const int DEFAULT_ROTATE_MAX_CORRECTIONS = 2;
const float MIN_TURN_ANGLE_DEG = 3.0f; // Góc quay tối thiểu để vượt ma sát tĩnh
const bool DEFAULT_PULSE_ENABLED = false; // [DRAFT] Tắt chế độ xung, dùng động lực liên tục
const float DEFAULT_PULSE_FREQ_HZ = 12.5f;
const float DEFAULT_PULSE_DUTY_PERCENT = 50.0f;
const float DEFAULT_PULSE_POWER_PERCENT = 100.0f;
const bool REFERENCE_WHEEL_IS_LEFT = true;
const bool LEFT_MOTOR_REVERSED = false;
const bool RIGHT_MOTOR_REVERSED = true;

const int MAX_RETRY_CONNECT_WIFI = 10; 
const int RETRY_CONNECT_WIFI_DELAY_MS = 500; 
const unsigned long WIFI_CONNECT_TIMEOUT_MS =
  (unsigned long)MAX_RETRY_CONNECT_WIFI * RETRY_CONNECT_WIFI_DELAY_MS;
const uint16_t DISCOVERY_UDP_PORT = 4210;
const char* DISCOVERY_REQUEST = "ROBOT_DISCOVER";

// ========== ZIGZAG NAVIGATION PARAMETERS ==========
const float ZIGZAG_ANGLE_OFFSET_M = 28.0f;      // M: Phần bù góc khi điều chỉnh hướng
const float ZIGZAG_DEVIATION_THRESHOLD_N = 5.0f; // N: Ngưỡng lệch góc cho phép
const float ZIGZAG_STRAIGHT_DISTANCE_K = 100.0f; // K: Quãng đường đi thẳng tối đa (cm)
const float ZIGZAG_OFFSET_DISTANCE_L = 30.0f;    // L: Độ dịch ngang khi chuyển hàng

enum RobotStatus {
  STARTED,
  OFF,
  WAIT_CONNECT,
  SLEEPING,
  BEGIN_JOB,
  RUNNING,
  GOING_HOME,
  LOW_BATTERY,
  ERROR
};

#pragma endregion

#pragma region VARIABLES

// ================= BIẾN TOÀN CỤC =================
volatile long pulseCountLeft = 0;
volatile long pulseCountRight = 0;
float gyroZ = 0, gyroOffsetZ = 0;
float mpuAngleZ = 0.0f; // raw integrated yaw for precise rotation control
float currentYaw = 0, targetYaw = 0;
unsigned long lastGyroUs = 0;
unsigned long lastSerialTime = 0; // Để khống chế tốc độ in Serial
int gyroStationaryCount = 0;
bool gyroInMotion = false;
unsigned long gyroReadErrorCount = 0;

int state = 0; 
int turnDirection = 1; 

float distL, distM, distR;

// Quay góc
float targetAngle = 0.0;     // Góc mục tiêu cần xoay tới
bool isTurning = false;      // Cờ đánh dấu đang trong quá trình xoay
unsigned long lastDebugTime = 0; // Khống chế tốc độ in log
bool isDone = false;

// Hỗ trợ kết nối và lưu trạng thái
RobotStatus status; // STARTED|WAIT_CONNECT|SLEEPING|BEGIN_JOB|RUNNING|GOING_HOME|LOW_BATTERY|ERROR
Preferences pref;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

struct RobotRuntimeConfig {
  String ssid;
  String pass;
  int goSpeed;
  int turnSpeed;
  float wheelRadiusCm;
  float rotateReverseThresholdDeg;
  float rotateToleranceDeg;
  int rotateMaxCorrections;
  float wallStopDistanceCm;
  int sleepMinutes;
  bool pulseEnabled;
  float pulseFreqHz;
  float pulseDutyPercent;
  float pulsePowerPercent;

  float zigzagAngleOffsetM;
  float zigzagDeviationThresholdN;
  float zigzagStraightDistanceK;
  float zigzagOffsetDistanceL;
};

RobotRuntimeConfig activeConfig;
RobotRuntimeConfig stagedConfig;
bool hasPendingConfig = false;
unsigned long sleepStartMs = 0;
unsigned long lastStatusPushMs = 0;
bool stopRequested = false;

// Pulsed motor power (to reduce continuous heating / improve stability)
// [DRAFT] Tắt chế độ xung mặc định
bool g_usePulsedPower = false;
unsigned long g_pulseOnMs = 40;      // milliseconds motor enabled in each cycle
unsigned long g_pulsePeriodMs = 80;  // total cycle period in ms
unsigned long g_pulseLastToggleMs = 0;
bool g_pulseActive = true;
int g_targetLeftSpeed = 0;
int g_targetRightSpeed = 0;
int g_appliedLeftSpeed = 0;
int g_appliedRightSpeed = 0;
float g_pulsePowerScale = 1.0f;

enum WifiConnectState {
  WIFI_CONNECT_IDLE,
  WIFI_CONNECT_REQUESTED,
  WIFI_CONNECT_PENDING
};

WifiConnectState wifiConnectState = WIFI_CONNECT_IDLE;
String pendingSsid;
String pendingPass;
unsigned long wifiConnectStartMs = 0;
String wifiConnectMessage = "idle";
bool apStarted = false;
bool serverStarted = false;
bool staServerReady = false;
bool udpDiscoveryStarted = false;
WiFiUDP discoveryUdp;

enum MoveCommandType {
  MOVE_NONE,
  MOVE_GO,
  MOVE_ROTATE
};

MoveCommandType pendingMoveType = MOVE_NONE;
float pendingMoveValue = 0.0;
bool moveBusy = false;

// Forward declarations for helpers defined later in this file.
void sendStatusPayload(AsyncWebSocketClient* client, const char* type);
void sendConfigPayload(AsyncWebSocketClient* client);
void applyStagedConfigIfNeeded();
void onWebSocketEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                      AwsEventType type, void* arg, uint8_t* data, size_t len);
float normalizeAngle180(float angleDeg);
void readEncoderCounts(long& leftCount, long& rightCount);
float pulsesToDistanceCm(long pulses, float wheelRadiusCm);
long getReferenceWheelCount(long leftCount, long rightCount);
void updatePulseFromConfig(const RobotRuntimeConfig& cfg);

// Update motor pulsing each loop to implement pulsed-power mode
void updateMotorPulsing();

// ========== ZIGZAG NAVIGATION STATE ==========
float referenceYaw = 0.0f;          // Góc tham chiếu chung cho toàn bộ chu kỳ RUNNING
bool zigzagTurnLeft = true;         // true = quay trái, false = quay phải
int zigzagRowCount = 0;             // Số hàng đã đi (để tracking)
bool zigzagGoingHome = false;       // Cờ đánh dấu đang quay về nhà
float zigzagAngleOffsetM = ZIGZAG_ANGLE_OFFSET_M;
float zigzagDeviationThresholdN = ZIGZAG_DEVIATION_THRESHOLD_N;
float zigzagStraightDistanceK = ZIGZAG_STRAIGHT_DISTANCE_K;
float zigzagOffsetDistanceL = ZIGZAG_OFFSET_DISTANCE_L;

struct ZigzagPath {
  float startYaw;
  float distance;
  int direction; // 1: tiến, -1: lùi
};

const int MAX_ZIGZAG_HISTORY = 50;
ZigzagPath zigzagHistory[MAX_ZIGZAG_HISTORY];
int zigzagHistoryCount = 0;

#pragma endregion

#pragma region EMIT_STATUS_SIGNAL
void emitWaitConnectSignal() {
  // Còi và LED nháy mỗi nhịp 2 lần liên tiếp
  Serial.println("Còi kêu: chờ kết nối");
  digitalWrite(signalPin, HIGH);
  delay(100);
  digitalWrite(signalPin, LOW);
  delay(50);
  digitalWrite(signalPin, HIGH);
  delay(100);
  digitalWrite(signalPin, LOW);
  delay(1000);
}

void emitErrorSignal() {
  // Còi và LED nháy liên tục mỗi 1s
  Serial.println("Còi kêu: gặp lỗi");
  digitalWrite(signalPin, HIGH);
  delay(1000);
  digitalWrite(signalPin, LOW);
  delay(1000);
}

void emitSignal() {
  if (status == WAIT_CONNECT) {
    emitWaitConnectSignal();
  } else if (status == ERROR) {
    emitErrorSignal();
  }
}
#pragma endregion

#pragma region WIFI_MANAGE 

void startAccessPoint() {
  if (apStarted) {
    if (!serverStarted) {
      server.begin();
      serverStarted = true;
    }
    return;
  }

  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP("Robot lau nhà", "12345678");
  apStarted = true;
  staServerReady = false;
  if (!serverStarted) {
    server.begin();
    serverStarted = true;
  }
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.softAPIP());
}

void restartServerListener() {
  if (serverStarted) {
    server.end();
  }
  server.begin();
  serverStarted = true;
}

void stopUdpDiscoveryIfNeeded() {
  if (!udpDiscoveryStarted) {
    return;
  }

  discoveryUdp.stop();
  udpDiscoveryStarted = false;
}

void startUdpDiscoveryIfNeeded() {
  if (udpDiscoveryStarted || WiFi.status() != WL_CONNECTED) {
    return;
  }

  if (discoveryUdp.begin(DISCOVERY_UDP_PORT)) {
    udpDiscoveryStarted = true;
    Serial.print("UDP discovery ready on port ");
    Serial.println(DISCOVERY_UDP_PORT);
  } else {
    Serial.println("UDP discovery start failed");
  }
}

void handleUdpDiscoveryRequest() {
  if (!udpDiscoveryStarted || WiFi.status() != WL_CONNECTED) {
    return;
  }

  int packetSize = discoveryUdp.parsePacket();
  if (packetSize <= 0) {
    return;
  }

  char buffer[64];
  int readLen = discoveryUdp.read(buffer, sizeof(buffer) - 1);
  if (readLen <= 0) {
    return;
  }
  buffer[readLen] = '\0';

  if (strcmp(buffer, DISCOVERY_REQUEST) != 0) {
    return;
  }

  IPAddress localIp = WiFi.localIP();
  String response = String("ROBOT_HERE ip=") + localIp.toString() + " http=80";

  discoveryUdp.beginPacket(discoveryUdp.remoteIP(), discoveryUdp.remotePort());
  discoveryUdp.print(response);
  discoveryUdp.endPacket();
}

bool loadStoredWiFi(String& ssid, String& pass) {
  pref.begin("config_wifi", true);
  ssid = pref.getString("ssid", "");
  pass = pref.getString("pass", "");
  pref.end();
  return ssid.length() > 0;
}

void queueWiFiConnection(const String& ssid, const String& pass) {
  pendingSsid = ssid;
  pendingPass = pass;
  wifiConnectState = WIFI_CONNECT_REQUESTED;
  wifiConnectMessage = "queued";
  staServerReady = false;
  sendStatusPayload(nullptr, "status");
}

void processWiFiConnectionTask() {
  if (wifiConnectState == WIFI_CONNECT_REQUESTED) {
    // Run WiFi stack operations from loop context, not Async callback task.
    stopUdpDiscoveryIfNeeded();
    WiFi.begin(pendingSsid.c_str(), pendingPass.c_str());
    wifiConnectStartMs = millis();
    wifiConnectState = WIFI_CONNECT_PENDING;
    wifiConnectMessage = "connecting";
    Serial.printf("Đang kết nối tới: %s\n", pendingSsid.c_str());
    return;
  }

  if (wifiConnectState != WIFI_CONNECT_PENDING) {
    return;
  }

  if (WiFi.status() == WL_CONNECTED) {
    pref.begin("config_wifi", false);
    pref.putString("ssid", pendingSsid);
    pref.putString("pass", pendingPass);
    pref.end();

    activeConfig.ssid = pendingSsid;
    activeConfig.pass = pendingPass;

    wifiConnectState = WIFI_CONNECT_IDLE;
    wifiConnectMessage = "connected";
    status = SLEEPING;
    sleepStartMs = millis();

    if (apStarted) {
      WiFi.softAPdisconnect(true);
      apStarted = false;
    }
    WiFi.mode(WIFI_STA);
    restartServerListener();
    startUdpDiscoveryIfNeeded();
    staServerReady = true;

    Serial.println(F("WiFi đã kết nối thành công!"));
    Serial.print(F("Địa chỉ IP: "));
    Serial.println(WiFi.localIP());
    sendStatusPayload(nullptr, "status");
    sendConfigPayload(nullptr);
    return;
  }

  if (millis() - wifiConnectStartMs >= WIFI_CONNECT_TIMEOUT_MS) {
    WiFi.disconnect(false, false);
    wifiConnectState = WIFI_CONNECT_IDLE;
    wifiConnectMessage = "failed";
    stopUdpDiscoveryIfNeeded();
    staServerReady = false;
    status = WAIT_CONNECT;
    Serial.println(F("Kết nối thất bại, quay lại AP mode"));
    sendStatusPayload(nullptr, "status");
  }
}

void handleUpdateWifiConfig(AsyncWebServerRequest* request, JsonVariant& json) {
  JsonObject doc = json.as<JsonObject>();

  if (!doc["ssid"].is<String>() || !doc["pass"].is<String>()) {
    Serial.println("Dữ liệu cấu hình wifi không đầy đủ (phải bao gồm ssid và pass)");
    request->send(400, "application/json", "{\"error\":\"missing_ssid_or_pass\"}");
    return;
  }

  if (wifiConnectState != WIFI_CONNECT_IDLE) {
    request->send(409, "application/json", "{\"error\":\"wifi_connect_in_progress\"}");
    return;
  }

  const String ssid = doc["ssid"].as<String>();
  const String pass = doc["pass"].as<String>();
  queueWiFiConnection(ssid, pass);
  request->send(202, "application/json", "{\"ok\":true,\"status\":\"queued\"}");
}

bool connectWiFi(const char* ssid, const char* pass) {
  Serial.printf("Đang kết nối tới: %s\n", ssid);
  
  WiFi.mode(WIFI_STA); 
  WiFi.begin(ssid, pass);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < MAX_RETRY_CONNECT_WIFI) {
    delay(RETRY_CONNECT_WIFI_DELAY_MS);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(F("\nWiFi đã kết nối thành công!"));
    Serial.print(F("Địa chỉ IP: "));
    Serial.println(WiFi.localIP());
    return true;
  } else {
    Serial.println(F("\nKết nối thất bại!"));
    return false;
  }
}

#pragma endregion

#pragma region MOVE

void IRAM_ATTR countPulseLeft() {
  pulseCountLeft++;
}

void IRAM_ATTR countPulseRight() {
  pulseCountRight++;
}

void setupEncoderInputPin(int pin) {
  // ESP32 GPIO34..GPIO39 are input-only and do not support internal pull-up.
  if (pin >= 34 && pin <= 39) {
    pinMode(pin, INPUT);
  } else {
    pinMode(pin, INPUT_PULLUP);
  }
}

void mpuWriteRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission(true);
}

bool readMpu16(uint8_t reg, int16_t& value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  int got = Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)2, (uint8_t)true);
  if (got != 2 || Wire.available() < 2) {
    return false;
  }

  value = (int16_t)((Wire.read() << 8) | Wire.read());
  return true;
}

void initMpuSensor() {
  // Reset MPU and configure stable clock/filter settings for yaw integration.
  mpuWriteRegister(0x6B, 0x80);
  delay(100);
  mpuWriteRegister(0x6B, 0x01);
  delay(20);
  mpuWriteRegister(0x1A, 0x03);
  mpuWriteRegister(0x19, 0x04);
  mpuWriteRegister(0x1B, 0x00);
}

void calibrateGyroOffset() {
  long sumZ = 0;
  int validSamples = 0;

  for (int i = 0; i < 200; i++) {
    int16_t raw = 0;
    (void)readMpu16(0x47, raw);
    delay(2);
  }

  for (int i = 0; i < GYRO_CALIB_SAMPLES; i++) {
    int16_t raw = 0;
    if (readMpu16(0x47, raw)) {
      sumZ += raw;
      validSamples++;
    }
    delay(2);
  }

  if (validSamples > 0) {
    gyroOffsetZ = (float)sumZ / (float)validSamples;
  }

  gyroZ = 0.0f;
  mpuAngleZ = 0.0f;
  gyroStationaryCount = 0;
  gyroInMotion = false;
}

void initMoveHardware() {
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
  pinMode(TRIG_M, OUTPUT); pinMode(ECHO_M, INPUT);
  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);

  setupEncoderInputPin(ENCODER_LEFT);
  setupEncoderInputPin(ENCODER_RIGHT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT), countPulseLeft, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT), countPulseRight, FALLING);

  Wire.begin(21, 22);
  initMpuSensor();
  delay(GYRO_WARMUP_MS);
  calibrateGyroOffset();
  lastGyroUs = micros();
  // initialize motor pulsing state
  g_pulseLastToggleMs = millis();
  g_pulseActive = true;
  g_targetLeftSpeed = 0;
  g_targetRightSpeed = 0;
  g_appliedLeftSpeed = 0;
  g_appliedRightSpeed = 0;
}

void readEncoderCounts(long& leftCount, long& rightCount) {
  noInterrupts();
  leftCount = pulseCountLeft;
  rightCount = pulseCountRight;
  interrupts();
}

float pulsesToDistanceCm(long pulses, float wheelRadiusCm) {
  float radius = constrain(wheelRadiusCm, 1.0f, 20.0f);
  float circumference = 2.0f * PI * radius;
  if (ENCODER_PULSES_PER_REV <= 0.0f) {
    return 0.0f;
  }

  return ((float)pulses / ENCODER_PULSES_PER_REV) * circumference;
}

long getReferenceWheelCount(long leftCount, long rightCount) {
  return REFERENCE_WHEEL_IS_LEFT ? leftCount : rightCount;
}

float getDist(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long dur = pulseIn(echo, HIGH, 15000);
  if (dur == 0) {
    return 999.0;
  }

  return (dur * 0.0343) / 2;
}

void updateYaw() {
  int16_t rawZ = 0;
  if (!readMpu16(0x47, rawZ)) {
    gyroReadErrorCount++;
    return;
  }

  unsigned long nowUs = micros();
  float dt = (nowUs - lastGyroUs) / 1000000.0f;
  lastGyroUs = nowUs;

  if (dt <= 0.0f || dt > 0.05f) {
    return;
  }

  float gyroRawDps = (rawZ - gyroOffsetZ) / GYRO_SCALE_DPS;
  if (fabsf(gyroRawDps) > GYRO_MAX_VALID_DPS) {
    return;
  }

  // Keep raw integrated yaw without deadband for fine rotation convergence.
  mpuAngleZ += gyroRawDps * dt;

  gyroZ += GYRO_LPF_ALPHA * (gyroRawDps - gyroZ);

  float absRate = fabsf(gyroZ);
  if (!gyroInMotion && absRate >= GYRO_MOTION_ENTER_DPS) {
    gyroInMotion = true;
  } else if (gyroInMotion && absRate <= GYRO_MOTION_EXIT_DPS) {
    gyroInMotion = false;
  }

  if (!gyroInMotion && absRate <= GYRO_STATIONARY_DPS) {
    if (gyroStationaryCount < GYRO_STATIONARY_COUNT_REQUIRED + 50) {
      gyroStationaryCount++;
    }
    if (gyroStationaryCount >= GYRO_STATIONARY_COUNT_REQUIRED) {
      gyroOffsetZ = (1.0f - GYRO_BIAS_ADAPT_ALPHA) * gyroOffsetZ + GYRO_BIAS_ADAPT_ALPHA * rawZ;
    }
  } else {
    gyroStationaryCount = 0;
  }

  float gyroDpsForIntegration = gyroZ;
  if (fabsf(gyroDpsForIntegration) < GYRO_DEADBAND_DPS) {
    gyroDpsForIntegration = 0.0f;
  }

  currentYaw += gyroDpsForIntegration * dt;
}

void setMotors(int speedL, int speedR) {
  int leftCmd = LEFT_MOTOR_REVERSED ? -speedL : speedL;
  int rightCmd = RIGHT_MOTOR_REVERSED ? -speedR : speedR;

  // set direction pins immediately (direction must be stable during pulses)
  if (leftCmd >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    leftCmd = -leftCmd;
  }

  if (rightCmd >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    rightCmd = -rightCmd;
  }

  // update requested target speeds (applied later according to pulsing state)
  g_targetLeftSpeed = constrain(leftCmd, 0, 255);
  g_targetRightSpeed = constrain(rightCmd, 0, 255);

  if (!g_usePulsedPower) {
    analogWrite(ENA, g_targetLeftSpeed);
    analogWrite(ENB, g_targetRightSpeed);
    g_appliedLeftSpeed = g_targetLeftSpeed;
    g_appliedRightSpeed = g_targetRightSpeed;
  } else {
    unsigned long now = millis();
    int appliedLeft = (int)(g_targetLeftSpeed * g_pulsePowerScale);
    int appliedRight = (int)(g_targetRightSpeed * g_pulsePowerScale);
    appliedLeft = constrain(appliedLeft, 0, 255);
    appliedRight = constrain(appliedRight, 0, 255);
    if (g_pulseActive && (now - g_pulseLastToggleMs) < g_pulseOnMs) {
      analogWrite(ENA, appliedLeft);
      analogWrite(ENB, appliedRight);
      g_appliedLeftSpeed = appliedLeft;
      g_appliedRightSpeed = appliedRight;
    } else {
      analogWrite(ENA, 0);
      analogWrite(ENB, 0);
      g_appliedLeftSpeed = 0;
      g_appliedRightSpeed = 0;
    }
  }
}

void stopMotors() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, HIGH);
  analogWrite(ENA, 255); analogWrite(ENB, 255);
  delay(120);
  analogWrite(ENA, 0); analogWrite(ENB, 0);
  g_targetLeftSpeed = 0;
  g_targetRightSpeed = 0;
  g_appliedLeftSpeed = 0;
  g_appliedRightSpeed = 0;
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

// Non-blocking pulsing update; toggles motor outputs according to configured on/period durations
void updateMotorPulsing() {
  if (!g_usePulsedPower) {
    return;
  }

  unsigned long now = millis();
  unsigned long onMs = g_pulseOnMs;
  unsigned long offMs = (g_pulsePeriodMs > g_pulseOnMs) ? (g_pulsePeriodMs - g_pulseOnMs) : 0;
  unsigned long phaseDuration = g_pulseActive ? onMs : offMs;

  if (phaseDuration == 0) {
    int appliedLeft = (int)(g_targetLeftSpeed * g_pulsePowerScale);
    int appliedRight = (int)(g_targetRightSpeed * g_pulsePowerScale);
    appliedLeft = constrain(appliedLeft, 0, 255);
    appliedRight = constrain(appliedRight, 0, 255);
    analogWrite(ENA, appliedLeft);
    analogWrite(ENB, appliedRight);
    g_appliedLeftSpeed = appliedLeft;
    g_appliedRightSpeed = appliedRight;
    return;
  }

  if (now - g_pulseLastToggleMs >= phaseDuration) {
    g_pulseActive = !g_pulseActive;
    g_pulseLastToggleMs = now;
    if (g_pulseActive) {
      int appliedLeft = (int)(g_targetLeftSpeed * g_pulsePowerScale);
      int appliedRight = (int)(g_targetRightSpeed * g_pulsePowerScale);
      appliedLeft = constrain(appliedLeft, 0, 255);
      appliedRight = constrain(appliedRight, 0, 255);
      analogWrite(ENA, appliedLeft);
      analogWrite(ENB, appliedRight);
      g_appliedLeftSpeed = appliedLeft;
      g_appliedRightSpeed = appliedRight;
    } else {
      analogWrite(ENA, 0);
      analogWrite(ENB, 0);
      g_appliedLeftSpeed = 0;
      g_appliedRightSpeed = 0;
    }
  }
}

float normalizeAngle180(float angleDeg) {
  while (angleDeg > 180.0f) {
    angleDeg -= 360.0f;
  }
  while (angleDeg < -180.0f) {
    angleDeg += 360.0f;
  }
  return angleDeg;
}

void driveStraight(int speed) {
  int linearSpeed = constrain(abs(speed), 0, 255);
  // Đi thẳng: 2 bánh cùng chiều trong hệ quy chiếu robot.
  setMotors(linearSpeed, linearSpeed);
}

void rotateInPlace(bool clockwise, int speed) {
  int turnSpeed = constrain(abs(speed), 0, 255);
  // Quay tại chỗ: 2 bánh ngược chiều nhau, cùng tốc độ.
  if (clockwise) {
    setMotors(turnSpeed, -turnSpeed);
  } else {
    setMotors(-turnSpeed, turnSpeed);
  }
}

void quay(float degree) {
  if (degree > 360.0 || degree < -360.0) {
    degree = fmod(degree, 360.0);
  }
  
  // Bỏ qua góc quay quá nhỏ (không đủ lực để thắng ma sát tĩnh)
  if (abs(degree) < 0.5f) {
    Serial.printf("Turn angle %.2f° too small, skipping\n", degree);
    return;
  }

  // [DRAFT] Không bắt buộc bật xung - để chế độ hiện tại
  bool prevPulseEnabled = g_usePulsedPower;
  // g_usePulsedPower = true;  // REMOVED: không ép buộc bật xung
  float prevPulsePowerScale = g_pulsePowerScale;

  updateYaw();
  float startYaw = mpuAngleZ;
  unsigned long startMs = millis();
  unsigned long timeoutMs = TURN_TIMEOUT_BASE_MS + (unsigned long)(fabsf(degree) * TURN_TIMEOUT_PER_DEG_MS);
  bool clockwise = degree > 0.0f;
  float allowedErrorDeg = constrain(activeConfig.rotateToleranceDeg, 0.5f, 20.0f);
  int maxCorrections = constrain(activeConfig.rotateMaxCorrections, 0, 20);
  bool allowReverse = maxCorrections > 0;
  int correctionCount = 0;
  bool wasOvershot = false;
  float lastRemainingAbs = fabsf(degree);
  unsigned long lastProgressMs = millis();

  long prevLeftCount = 0;
  long prevRightCount = 0;
  readEncoderCounts(prevLeftCount, prevRightCount);

  while (true) {
    if (stopRequested) {
      stopMotors();
      break;
    }

    updateYaw();
    float turnedSigned = mpuAngleZ - startYaw;
    float remainingSigned = degree - turnedSigned;
    float remainingAbs = fabsf(remainingSigned);

    if (remainingAbs < (lastRemainingAbs - 0.10f)) {
      lastRemainingAbs = remainingAbs;
      lastProgressMs = millis();
    }

    if (remainingAbs <= allowedErrorDeg) {
      stopMotors();
      g_usePulsedPower = prevPulseEnabled;
      break;
    }

    bool overshot = (degree > 0.0f && remainingSigned < 0.0f) ||
                    (degree < 0.0f && remainingSigned > 0.0f);
    if (overshot) {
      if (!wasOvershot) {
        if (allowReverse && correctionCount < maxCorrections) {
          clockwise = remainingSigned > 0.0f;
          correctionCount++;
        } else {
          stopMotors();
          g_usePulsedPower = prevPulseEnabled;
          break;
        }
      }
      wasOvershot = true;
    } else {
      wasOvershot = false;
    }

    if (millis() - startMs >= timeoutMs) {
      stopMotors();
      status = ERROR;
      Serial.println("Rotate timeout");
      g_usePulsedPower = prevPulseEnabled;
      break;
    }

    int minRotateSpeed = remainingAbs <= ROTATE_FINE_ZONE_DEG ? ROTATE_FINE_MIN_SPEED : ROTATE_MIN_SPEED;
    if (remainingAbs <= ROTATE_FINE_ZONE_DEG) {
      // With pulsing enabled, keep enough torque in fine zone to avoid stalling.
      minRotateSpeed = max(minRotateSpeed, 28);
    }
    int referenceSpeed = (int)(remainingAbs * ROTATE_SPEED_KP);
    referenceSpeed = constrain(referenceSpeed, minRotateSpeed, activeConfig.turnSpeed);

    // Boost pulse power near target to overcome static friction
    if (remainingAbs <= ROTATE_FINE_ZONE_DEG) {
      g_pulsePowerScale = 1.0f;
    } else {
      g_pulsePowerScale = prevPulsePowerScale;
    }

    // If angle has not improved for a while, inject a stronger nudge pulse.
    if ((millis() - lastProgressMs) > 180UL && remainingAbs > allowedErrorDeg) {
      referenceSpeed = max(referenceSpeed, ROTATE_MIN_SPEED + 8);
      g_pulsePowerScale = 1.0f;
      g_pulseActive = true;
      g_pulseLastToggleMs = millis();
      lastProgressMs = millis();
    }

    long leftCount = 0;
    long rightCount = 0;
    readEncoderCounts(leftCount, rightCount);

    long referenceCountNow = getReferenceWheelCount(leftCount, rightCount);
    long referenceCountPrev = getReferenceWheelCount(prevLeftCount, prevRightCount);
    long followerCountNow = REFERENCE_WHEEL_IS_LEFT ? rightCount : leftCount;
    long followerCountPrev = REFERENCE_WHEEL_IS_LEFT ? prevRightCount : prevLeftCount;

    long deltaReference = abs(referenceCountNow - referenceCountPrev);
    long deltaFollower = abs(followerCountNow - followerCountPrev);
    float encoderError = (float)(deltaReference - deltaFollower);

    int followerSpeed = constrain((int)(referenceSpeed + encoderError * ROTATE_ENCODER_BALANCE_KP),
                                  ROTATE_FINE_MIN_SPEED,
                                  activeConfig.turnSpeed);

    if (REFERENCE_WHEEL_IS_LEFT) {
      if (clockwise) {
        setMotors(referenceSpeed, -followerSpeed);
      } else {
        setMotors(-referenceSpeed, followerSpeed);
      }
    } else {
      if (clockwise) {
        setMotors(followerSpeed, -referenceSpeed);
      } else {
        setMotors(-followerSpeed, referenceSpeed);
      }
    }

    prevLeftCount = leftCount;
    prevRightCount = rightCount;

    updateMotorPulsing();
    delay(4);
  }

  g_usePulsedPower = prevPulseEnabled;
  g_pulsePowerScale = prevPulsePowerScale;
}

void di_thang(float targetDistanceCm) {
  if (targetDistanceCm <= 0.0f) {
    return;
  }

  // [DRAFT] Không bắt buộc bật xung - để chế độ hiện tại
  bool prevPulseEnabled = g_usePulsedPower;
  // g_usePulsedPower = true;  // REMOVED: không ép buộc bật xung

  targetDistanceCm = constrain(targetDistanceCm, 0.0f, GO_MAX_DISTANCE_CM);
  unsigned long startMs = millis();
  unsigned long timeoutMs = 10000UL + (unsigned long)(targetDistanceCm * 250.0f);
  unsigned long lastControlMs = 0;

  updateYaw();
  float startYaw = currentYaw;

  long startLeftCount = 0;
  long startRightCount = 0;
  readEncoderCounts(startLeftCount, startRightCount);

  long startReferenceCount = getReferenceWheelCount(startLeftCount, startRightCount);
  long prevReferenceCount = startReferenceCount;
  long prevFollowerCount = REFERENCE_WHEEL_IS_LEFT ? startRightCount : startLeftCount;

  int referenceSpeed = constrain(activeConfig.goSpeed, 0, 255);
  if (REFERENCE_WHEEL_IS_LEFT) {
    setMotors(referenceSpeed, referenceSpeed);
  } else {
    setMotors(referenceSpeed, referenceSpeed);
  }

  while (true) {
    if (stopRequested) {
      g_usePulsedPower = prevPulseEnabled;
      break;
    }

    if (millis() - startMs >= timeoutMs) {
      Serial.println("Di thang timeout");
      status = ERROR;
      g_usePulsedPower = prevPulseEnabled;
      break;
    }

    distM = getDist(TRIG_M, ECHO_M);
    if (distM <= activeConfig.wallStopDistanceCm) {
      Serial.println("Vat can gan, dung lai");
      g_usePulsedPower = prevPulseEnabled;
      break;
    }

    long leftCount = 0;
    long rightCount = 0;
    readEncoderCounts(leftCount, rightCount);

    long referenceCount = getReferenceWheelCount(leftCount, rightCount);
    float traveledCm = pulsesToDistanceCm(abs(referenceCount - startReferenceCount), activeConfig.wheelRadiusCm);
    if (traveledCm >= targetDistanceCm) {
      g_usePulsedPower = prevPulseEnabled;
      break;
    }

    unsigned long now = millis();
    if (now - lastControlMs >= STRAIGHT_CONTROL_INTERVAL_MS) {
      lastControlMs = now;
      updateYaw();

      long followerCount = REFERENCE_WHEEL_IS_LEFT ? rightCount : leftCount;

      long deltaReference = referenceCount - prevReferenceCount;
      long deltaFollower = followerCount - prevFollowerCount;
      prevReferenceCount = referenceCount;
      prevFollowerCount = followerCount;

      float yawError = normalizeAngle180(startYaw - currentYaw);
      float encoderError = (float)(deltaReference - deltaFollower);

      float followerCorrection = (yawError * STRAIGHT_YAW_KP) + (encoderError * ENCODER_BALANCE_KP);
      int followerSpeed = constrain((int)(referenceSpeed + followerCorrection), 0, 255);

      if (REFERENCE_WHEEL_IS_LEFT) {
        setMotors(referenceSpeed, followerSpeed);
      } else {
        setMotors(followerSpeed, referenceSpeed);
      }
    }

    updateMotorPulsing();
    delay(10);
  }

  stopMotors();
  g_usePulsedPower = prevPulseEnabled;
}

bool queueMoveCommand(const String& type, float value) {
  if (status == OFF) {
    return false;
  }

  if (moveBusy || pendingMoveType != MOVE_NONE) {
    return false;
  }

  if (type == "go") {
    if (value <= 0.0f || value > GO_MAX_DISTANCE_CM) {
      return false;
    }
    pendingMoveType = MOVE_GO;
  } else if (type == "rotate") {
    if (value > 360.0f || value < -360.0f) {
      value = fmod(value, 360.0f);
    }
    if (fabsf(value) < 0.5f) {
      return false;
    }
    pendingMoveType = MOVE_ROTATE;
  } else {
    return false;
  }

  pendingMoveValue = value;
  if (status == SLEEPING) {
    status = BEGIN_JOB;
  }
  return true;
}

void processMoveCommandTask() {
  if (moveBusy || pendingMoveType == MOVE_NONE) {
    return;
  }

  applyStagedConfigIfNeeded();

  moveBusy = true;
  stopRequested = false;
  MoveCommandType commandType = pendingMoveType;
  float commandValue = pendingMoveValue;
  pendingMoveType = MOVE_NONE;
  pendingMoveValue = 0.0;

  status = RUNNING;
  sendStatusPayload(nullptr, "status");
  if (commandType == MOVE_GO) {
    di_thang(commandValue);
  } else if (commandType == MOVE_ROTATE) {
    quay(commandValue);
  }

  if (status != ERROR) {
    status = SLEEPING;
  }
  sleepStartMs = millis();
  sendStatusPayload(nullptr, "status");

  moveBusy = false;
}

#pragma endregion

#pragma region UTIL

bool extractJsonFromData(JsonDocument& doc, uint8_t *data, size_t len) {
  // Giải mã trực tiếp từ mảng byte nhận được
  DeserializationError error = deserializeJson(doc, data, len);

  if (error) {
    Serial.print(F("Lỗi parse JSON: "));
    Serial.println(error.f_str());
    return false;
  }
  return true;
}

const char* statusToCode(RobotStatus value) {
  switch (value) {
    case STARTED: return "STARTED";
    case OFF: return "OFF";
    case WAIT_CONNECT: return "WAIT_CONNECT";
    case SLEEPING: return "SLEEPING";
    case BEGIN_JOB: return "BEGIN_JOB";
    case RUNNING: return "RUNNING";
    case GOING_HOME: return "GOING_HOME";
    case LOW_BATTERY: return "LOW_BATTERY";
    case ERROR: return "ERROR";
    default: return "UNKNOWN";
  }
}

const char* statusToDescription(RobotStatus value) {
  switch (value) {
    case STARTED: return "Robot dang khoi dong";
    case OFF: return "Robot dang tat";
    case WAIT_CONNECT: return "Dang cho ket noi";
    case SLEEPING: return "Dang ngu";
    case BEGIN_JOB: return "Dang chuan bi lam viec";
    case RUNNING: return "Dang lam viec";
    case GOING_HOME: return "Dang quay ve home";
    case LOW_BATTERY: return "Pin thap";
    case ERROR: return "Robot gap loi";
    default: return "Khong xac dinh";
  }
}

void normalizeRuntimeConfig(RobotRuntimeConfig& cfg) {
  cfg.goSpeed = constrain(cfg.goSpeed, 0, 255);
  cfg.turnSpeed = constrain(cfg.turnSpeed, 0, 255);
  cfg.wheelRadiusCm = constrain(cfg.wheelRadiusCm, 1.0f, 20.0f);
  cfg.rotateReverseThresholdDeg = constrain(cfg.rotateReverseThresholdDeg, 0.5f, 90.0f);
  cfg.rotateToleranceDeg = constrain(cfg.rotateToleranceDeg, 0.5f, 20.0f);
  cfg.rotateMaxCorrections = constrain(cfg.rotateMaxCorrections, 0, 20);
  cfg.wallStopDistanceCm = constrain(cfg.wallStopDistanceCm, 5.0f, 200.0f);
  cfg.sleepMinutes = constrain(cfg.sleepMinutes, 1, 120);
  cfg.pulseFreqHz = constrain(cfg.pulseFreqHz, 1.0f, 200.0f);
  cfg.pulseDutyPercent = constrain(cfg.pulseDutyPercent, 5.0f, 95.0f);
  cfg.pulsePowerPercent = constrain(cfg.pulsePowerPercent, 10.0f, 100.0f);

  cfg.zigzagAngleOffsetM = constrain(cfg.zigzagAngleOffsetM, 0.0f, 45.0f);
  cfg.zigzagDeviationThresholdN = constrain(cfg.zigzagDeviationThresholdN, 1.0f, 30.0f);
  cfg.zigzagStraightDistanceK = constrain(cfg.zigzagStraightDistanceK, 20.0f, 500.0f);
  cfg.zigzagOffsetDistanceL = constrain(cfg.zigzagOffsetDistanceL, 10.0f, 100.0f);
}

RobotRuntimeConfig createDefaultRuntimeConfig() {
  RobotRuntimeConfig cfg;
  cfg.ssid = "";
  cfg.pass = "";
  cfg.goSpeed = DEFAULT_GO_SPEED;
  cfg.turnSpeed = DEFAULT_TURN_SPEED;
  cfg.wheelRadiusCm = DEFAULT_WHEEL_RADIUS_CM;
  cfg.rotateReverseThresholdDeg = DEFAULT_ROTATE_REVERSE_THRESHOLD_DEG;
  cfg.rotateToleranceDeg = DEFAULT_ROTATE_TOLERANCE_DEG;
  cfg.rotateMaxCorrections = DEFAULT_ROTATE_MAX_CORRECTIONS;
  cfg.wallStopDistanceCm = DEFAULT_WALL_STOP_DISTANCE_CM;
  cfg.sleepMinutes = DEFAULT_SLEEP_MINUTES;
  cfg.pulseEnabled = DEFAULT_PULSE_ENABLED;
  cfg.pulseFreqHz = DEFAULT_PULSE_FREQ_HZ;
  cfg.pulseDutyPercent = DEFAULT_PULSE_DUTY_PERCENT;
  cfg.pulsePowerPercent = DEFAULT_PULSE_POWER_PERCENT;

  cfg.zigzagAngleOffsetM = ZIGZAG_ANGLE_OFFSET_M;
  cfg.zigzagDeviationThresholdN = ZIGZAG_DEVIATION_THRESHOLD_N;
  cfg.zigzagStraightDistanceK = ZIGZAG_STRAIGHT_DISTANCE_K;
  cfg.zigzagOffsetDistanceL = ZIGZAG_OFFSET_DISTANCE_L;

  return cfg;
}

void loadRuntimeConfig() {
  activeConfig = createDefaultRuntimeConfig();

  pref.begin("config_wifi", true);
  activeConfig.ssid = pref.getString("ssid", "");
  activeConfig.pass = pref.getString("pass", "");
  pref.end();

  pref.begin("robot_runtime", true);
  activeConfig.goSpeed = pref.getInt("go_speed", DEFAULT_GO_SPEED);
  activeConfig.turnSpeed = pref.getInt("turn_speed", DEFAULT_TURN_SPEED);
  activeConfig.wheelRadiusCm = pref.getFloat("wheel_radius_cm", DEFAULT_WHEEL_RADIUS_CM);
  activeConfig.rotateReverseThresholdDeg = pref.getFloat("rotate_reverse_deg", DEFAULT_ROTATE_REVERSE_THRESHOLD_DEG);
  activeConfig.rotateToleranceDeg = pref.getFloat("rotate_tolerance_deg", DEFAULT_ROTATE_TOLERANCE_DEG);
  activeConfig.rotateMaxCorrections = pref.getInt("rotate_max_corr", DEFAULT_ROTATE_MAX_CORRECTIONS);
  activeConfig.wallStopDistanceCm = pref.getFloat("wall_distance", DEFAULT_WALL_STOP_DISTANCE_CM);
  activeConfig.sleepMinutes = pref.getInt("sleep_minutes", DEFAULT_SLEEP_MINUTES);
  activeConfig.pulseEnabled = pref.getBool("pulse_enabled", DEFAULT_PULSE_ENABLED);
  activeConfig.pulseFreqHz = pref.getFloat("pulse_freq_hz", DEFAULT_PULSE_FREQ_HZ);
  activeConfig.pulseDutyPercent = pref.getFloat("pulse_duty_pct", DEFAULT_PULSE_DUTY_PERCENT);
  activeConfig.pulsePowerPercent = pref.getFloat("pulse_power_pct", DEFAULT_PULSE_POWER_PERCENT);

  activeConfig.zigzagAngleOffsetM = pref.getFloat("zigzag_m", ZIGZAG_ANGLE_OFFSET_M);
  activeConfig.zigzagDeviationThresholdN = pref.getFloat("zigzag_n", ZIGZAG_DEVIATION_THRESHOLD_N);
  activeConfig.zigzagStraightDistanceK = pref.getFloat("zigzag_k", ZIGZAG_STRAIGHT_DISTANCE_K);
  activeConfig.zigzagOffsetDistanceL = pref.getFloat("zigzag_l", ZIGZAG_OFFSET_DISTANCE_L);

  pref.end();

  normalizeRuntimeConfig(activeConfig);
  updatePulseFromConfig(activeConfig);
}

void saveRuntimeConfig(const RobotRuntimeConfig& cfg) {
  pref.begin("robot_runtime", false);
  pref.putInt("go_speed", cfg.goSpeed);
  pref.putInt("turn_speed", cfg.turnSpeed);
  pref.putFloat("wheel_radius_cm", cfg.wheelRadiusCm);
  pref.putFloat("rotate_reverse_deg", cfg.rotateReverseThresholdDeg);
  pref.putFloat("rotate_tolerance_deg", cfg.rotateToleranceDeg);
  pref.putInt("rotate_max_corr", cfg.rotateMaxCorrections);
  pref.putFloat("wall_distance", cfg.wallStopDistanceCm);
  pref.putInt("sleep_minutes", cfg.sleepMinutes);
  pref.putBool("pulse_enabled", cfg.pulseEnabled);
  pref.putFloat("pulse_freq_hz", cfg.pulseFreqHz);
  pref.putFloat("pulse_duty_pct", cfg.pulseDutyPercent);
  pref.putFloat("pulse_power_pct", cfg.pulsePowerPercent);

  pref.putFloat("zigzag_m", cfg.zigzagAngleOffsetM);
  pref.putFloat("zigzag_n", cfg.zigzagDeviationThresholdN);
  pref.putFloat("zigzag_k", cfg.zigzagStraightDistanceK);
  pref.putFloat("zigzag_l", cfg.zigzagOffsetDistanceL);

  pref.end();

  if (cfg.ssid.length() > 0) {
    pref.begin("config_wifi", false);
    pref.putString("ssid", cfg.ssid);
    pref.putString("pass", cfg.pass);
    pref.end();
  }
}

void sendAck(AsyncWebSocketClient* client, const String& action, bool ok, const String& message) {
  StaticJsonDocument<256> doc;
  doc["type"] = "ack";
  doc["action"] = action;
  doc["ok"] = ok;
  doc["message"] = message;

  String payload;
  serializeJson(doc, payload);

  if (client != nullptr) {
    client->text(payload);
  } else {
    ws.textAll(payload);
  }
}

void sendError(AsyncWebSocketClient* client, const String& message) {
  StaticJsonDocument<256> doc;
  doc["type"] = "error";
  doc["message"] = message;

  String payload;
  serializeJson(doc, payload);
  if (client != nullptr) {
    client->text(payload);
  }
}

void sendStatusPayload(AsyncWebSocketClient* client, const char* type) {
  StaticJsonDocument<320> doc;
  doc["type"] = type;
  doc["status"] = statusToCode(status);
  doc["statusText"] = statusToDescription(status);
  doc["busy"] = moveBusy;
  doc["configPending"] = hasPendingConfig;
  doc["wifiState"] = wifiConnectMessage;

  String payload;
  serializeJson(doc, payload);

  if (client != nullptr) {
    client->text(payload);
  } else {
    ws.textAll(payload);
  }
}

void sendConfigPayload(AsyncWebSocketClient* client) {
  StaticJsonDocument<512> doc;
  doc["type"] = "config";
  JsonObject cfg = doc.createNestedObject("config");
  const RobotRuntimeConfig& currentConfig = hasPendingConfig ? stagedConfig : activeConfig;
  cfg["ssid"] = currentConfig.ssid;
  cfg["pass"] = currentConfig.pass;
  cfg["goSpeed"] = currentConfig.goSpeed;
  cfg["turnSpeed"] = currentConfig.turnSpeed;
  cfg["wheelRadiusCm"] = currentConfig.wheelRadiusCm;
  cfg["rotateReverseThresholdDeg"] = currentConfig.rotateReverseThresholdDeg;
  cfg["rotateToleranceDeg"] = currentConfig.rotateToleranceDeg;
  cfg["rotateMaxCorrections"] = currentConfig.rotateMaxCorrections;
  cfg["wallStopDistanceCm"] = currentConfig.wallStopDistanceCm;
  cfg["sleepMinutes"] = currentConfig.sleepMinutes;
  cfg["pulseEnabled"] = currentConfig.pulseEnabled;
  cfg["pulseFreqHz"] = currentConfig.pulseFreqHz;
  cfg["pulseDutyPercent"] = currentConfig.pulseDutyPercent;
  cfg["pulsePowerPercent"] = currentConfig.pulsePowerPercent;

  cfg["zigzagAngleOffsetM"] = currentConfig.zigzagAngleOffsetM;
  cfg["zigzagDeviationThresholdN"] = currentConfig.zigzagDeviationThresholdN;
  cfg["zigzagStraightDistanceK"] = currentConfig.zigzagStraightDistanceK;
  cfg["zigzagOffsetDistanceL"] = currentConfig.zigzagOffsetDistanceL; 

  String payload;
  serializeJson(doc, payload);

  if (client != nullptr) {
    client->text(payload);
  } else {
    ws.textAll(payload);
  } 
}

void applyStagedConfigIfNeeded() {
  if (!hasPendingConfig || moveBusy) {
    return;
  }

  activeConfig = stagedConfig;
  normalizeRuntimeConfig(activeConfig);
  updatePulseFromConfig(activeConfig);
  saveRuntimeConfig(activeConfig);
  hasPendingConfig = false;
  sendAck(nullptr, "update_config", true, "Cau hinh da duoc ap dung va luu bo nho");
  sendConfigPayload(nullptr);
}

bool queueMoveFromSocket(const String& type, float value) {
  applyStagedConfigIfNeeded();
  bool queued = queueMoveCommand(type, value);
  if (queued) {
    sendStatusPayload(nullptr, "status");
  }
  return queued;
}

void handleSocketUpdateConfig(AsyncWebSocketClient* client, JsonObject configObj) {
  RobotRuntimeConfig nextConfig = activeConfig;

  if (configObj["ssid"].is<String>()) {
    nextConfig.ssid = configObj["ssid"].as<String>();
  }
  if (configObj["pass"].is<String>()) {
    nextConfig.pass = configObj["pass"].as<String>();
  }
  if (configObj["goSpeed"].is<int>()) {
    nextConfig.goSpeed = configObj["goSpeed"].as<int>();
  }
  if (configObj["turnSpeed"].is<int>()) {
    nextConfig.turnSpeed = configObj["turnSpeed"].as<int>();
  }
  if (configObj["wheelRadiusCm"].is<float>() || configObj["wheelRadiusCm"].is<int>()) {
    nextConfig.wheelRadiusCm = configObj["wheelRadiusCm"].as<float>();
  }
  if (configObj["rotateReverseThresholdDeg"].is<float>() || configObj["rotateReverseThresholdDeg"].is<int>()) {
    nextConfig.rotateReverseThresholdDeg = configObj["rotateReverseThresholdDeg"].as<float>();
  }
  if (configObj["rotateToleranceDeg"].is<float>() || configObj["rotateToleranceDeg"].is<int>()) {
    nextConfig.rotateToleranceDeg = configObj["rotateToleranceDeg"].as<float>();
  }
  if (configObj["rotateMaxCorrections"].is<int>()) {
    nextConfig.rotateMaxCorrections = configObj["rotateMaxCorrections"].as<int>();
  }
  if (configObj["wallStopDistanceCm"].is<float>() || configObj["wallStopDistanceCm"].is<int>()) {
    nextConfig.wallStopDistanceCm = configObj["wallStopDistanceCm"].as<float>();
  }
  if (configObj["sleepMinutes"].is<int>()) {
    nextConfig.sleepMinutes = configObj["sleepMinutes"].as<int>();
  }
  if (configObj["pulseEnabled"].is<bool>()) {
    nextConfig.pulseEnabled = configObj["pulseEnabled"].as<bool>();
  }
  if (configObj["pulseFreqHz"].is<float>() || configObj["pulseFreqHz"].is<int>()) {
    nextConfig.pulseFreqHz = configObj["pulseFreqHz"].as<float>();
  }
  if (configObj["pulseDutyPercent"].is<float>() || configObj["pulseDutyPercent"].is<int>()) {
    nextConfig.pulseDutyPercent = configObj["pulseDutyPercent"].as<float>();
  }
  if (configObj["pulsePowerPercent"].is<float>() || configObj["pulsePowerPercent"].is<int>()) {
    nextConfig.pulsePowerPercent = configObj["pulsePowerPercent"].as<float>();
  }
  if (configObj["zigzagAngleOffsetM"].is<float>() || configObj["zigzagAngleOffsetM"].is<int>()) {
    nextConfig.zigzagAngleOffsetM = configObj["zigzagAngleOffsetM"].as<float>();
  }
  if (configObj["zigzagDeviationThresholdN"].is<float>() || configObj["zigzagDeviationThresholdN"].is<int>()) {
    nextConfig.zigzagDeviationThresholdN = configObj["zigzagDeviationThresholdN"].as<float>();
  }
  if (configObj["zigzagStraightDistanceK"].is<float>() || configObj["zigzagStraightDistanceK"].is<int>()) {
    nextConfig.zigzagStraightDistanceK = configObj["zigzagStraightDistanceK"].as<float>();
  }
  if (configObj["zigzagOffsetDistanceL"].is<float>() || configObj["zigzagOffsetDistanceL"].is<int>()) {
    nextConfig.zigzagOffsetDistanceL = configObj["zigzagOffsetDistanceL"].as<float>();
  }

  normalizeRuntimeConfig(nextConfig);
  stagedConfig = nextConfig;
  hasPendingConfig = true;

  // Persist ngay khi nhận cấu hình để tránh mất khi robot restart trước chu kỳ kế tiếp.
  saveRuntimeConfig(stagedConfig);

  sendAck(client, "update_config", true, "Da luu cau hinh, se ap dung o chu ky tiep theo");
  sendConfigPayload(client);
  sendStatusPayload(nullptr, "status");
}

void handleSocketCommand(AsyncWebSocketClient* client, const String& command) {
  if (command == "stop") {
    if (status == OFF) {
      sendAck(client, "command", false, "Robot dang OFF, hay bat len truoc");
      return;
    }

    stopRequested = true;
    pendingMoveType = MOVE_NONE;
    pendingMoveValue = 0.0f;
    stopMotors();
    status = SLEEPING;
    sleepStartMs = millis();
    sendAck(client, "command", true, "Robot da dung");
    sendStatusPayload(nullptr, "status");
    return;
  }

  if (command == "go_home") {
    if (status == OFF) {
      sendAck(client, "command", false, "Robot dang OFF, hay bat len truoc");
      return;
    }

    stopRequested = true;
    pendingMoveType = MOVE_NONE;
    pendingMoveValue = 0.0f;
    status = GOING_HOME;
    sendAck(client, "command", true, "Robot dang quay ve home");
    sendStatusPayload(nullptr, "status");
    return;
  }

  if (command == "start_work") {
    if (status == OFF) {
      sendAck(client, "command", false, "Robot dang OFF, hay bat len truoc");
      return;
    }
    
    // Khởi động zigzag navigation
    status = BEGIN_JOB;
    sendAck(client, "command", true, "Robot se bat dau lam viec zigzag");
    sendStatusPayload(nullptr, "status");
    return;
  }

  if (command == "power_off") {
    stopRequested = true;
    pendingMoveType = MOVE_NONE;
    pendingMoveValue = 0.0f;
    stopMotors();
    status = OFF;
    sendAck(client, "command", true, "Robot da tat");
    sendStatusPayload(nullptr, "status");
    return;
  }

  if (command == "power_on") {
    if (status == OFF) {
      status = (WiFi.status() == WL_CONNECTED) ? SLEEPING : WAIT_CONNECT;
      sleepStartMs = millis();
    }
    sendAck(client, "command", true, "Robot da bat");
    sendStatusPayload(nullptr, "status");
    return;
  }

  sendError(client, "Lenh khong duoc ho tro");
}

void handleSocketTextMessage(AsyncWebSocketClient* client, const String& text) {
  DynamicJsonDocument doc(1024);
  DeserializationError err = deserializeJson(doc, text);
  if (err) {
    sendError(client, "JSON khong hop le");
    return;
  }

  JsonObject root = doc.as<JsonObject>();
  String type = root["type"] | "";

  if (type == "get_status") {
    sendStatusPayload(client, "status");
    return;
  }

  if (type == "get_config") {
    sendConfigPayload(client);
    return;
  }

  if (type == "command") {
    String command = root["command"] | "";
    handleSocketCommand(client, command);
    return;
  }

  if (type == "debug_move") {
    String moveType = root["moveType"] | "";
    float value = root["value"] | 0.0f;
    bool queued = queueMoveFromSocket(moveType, value);
    sendAck(client, "debug_move", queued, queued ? "Da xep lenh" : "Robot ban hoac gia tri khong hop le");
    return;
  }

  if (type == "update_config") {
    if (!root["config"].is<JsonObject>()) {
      sendError(client, "Thieu object config");
      return;
    }
    handleSocketUpdateConfig(client, root["config"].as<JsonObject>());
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

  if (type != WS_EVT_DATA) {
    return;
  }

  AwsFrameInfo* info = (AwsFrameInfo*)arg;
  if (info == nullptr || !info->final || info->index != 0 || info->len != len || info->opcode != WS_TEXT) {
    return;
  }

  String text;
  text.reserve(len);
  for (size_t i = 0; i < len; ++i) {
    text += (char)data[i];
  }
  handleSocketTextMessage(client, text);
}

void pushStatusIfNeeded() {
  if (millis() - lastStatusPushMs < STATUS_PUSH_INTERVAL_MS) {
    return;
  }
  lastStatusPushMs = millis();
  sendStatusPayload(nullptr, "status");
}

#pragma endregion

#pragma region CONTROL_API

void handleControlStatus(AsyncWebServerRequest* request, JsonVariant& json) {
  JsonObject doc = json.as<JsonObject>();
  if (!doc["status"].is<int>()) {
    request->send(400, "application/json", "{\"error\":\"status_must_be_number\"}");
    return;
  }

  int nextStatus = doc["status"].as<int>();
  if (nextStatus < STARTED || nextStatus > ERROR) {
    request->send(400, "application/json", "{\"error\":\"status_out_of_range\"}");
    return;
  }

  status = static_cast<RobotStatus>(nextStatus);
  request->send(200, "application/json", "{\"ok\":true}");
}

void handleControlMove(AsyncWebServerRequest* request, JsonVariant& json) {
  JsonObject doc = json.as<JsonObject>();
  if (!doc["type"].is<String>() || !doc["value"].is<float>()) {
    request->send(400, "application/json", "{\"error\":\"type_or_value_invalid\"}");
    return;
  }

  String type = doc["type"].as<String>();
  float value = doc["value"].as<float>();

  if (!queueMoveCommand(type, value)) {
    request->send(409, "application/json", "{\"error\":\"robot_busy_or_invalid_type\"}");
    return;
  }

  request->send(202, "application/json", "{\"ok\":true,\"status\":\"queued\"}");
}

#pragma endregion

#pragma region HANDLER
void handleOnStartedStatus() {
  Serial.println("STARTED");
  status = WAIT_CONNECT;
  sendStatusPayload(nullptr, "status");
}

void handleOffStatus() {
  stopRequested = true;
  stopMotors();
  if (millis() - lastSerialTime > 2000) {
    Serial.println("OFF");
    lastSerialTime = millis();
  }
}

void handleWaitConnectStatus() {
  // Bắt đầu trạng thái phát điểm truy cập (nêu cần)
  if (!apStarted) {
    startAccessPoint();
  }
  status = WAIT_CONNECT;
  if (millis() - lastSerialTime > 2000) {
    Serial.println("WAIT");
    lastSerialTime = millis();
  }
  emitWaitConnectSignal();
}

void handleBeginJobStatus() {
  // if (!moveBusy && pendingMoveType == MOVE_NONE) {
  //   queueMoveCommand("go", AUTO_WORK_DISTANCE_CM);
  // }

  if (!moveBusy && pendingMoveType == MOVE_NONE) {
    // Thay vì đi thẳng đơn giản, chạy zigzag navigation
    status = RUNNING;
    sendStatusPayload(nullptr, "status");
    
    // Cập nhật config vào biến toàn cục
    zigzagAngleOffsetM = activeConfig.zigzagAngleOffsetM;
    zigzagDeviationThresholdN = activeConfig.zigzagDeviationThresholdN;
    zigzagStraightDistanceK = activeConfig.zigzagStraightDistanceK;
    zigzagOffsetDistanceL = activeConfig.zigzagOffsetDistanceL;
    
    runZigzagCycle();
  }
}

void handleRunningStatus() {
  if (millis() - lastSerialTime > 2000) {
    Serial.println("RUNNING");
    lastSerialTime = millis();
  }
}

void handleGoingHomeStatus() {
  Serial.println("GoingHome");
  stopRequested = true;
  stopMotors();
  status = SLEEPING;
  sleepStartMs = millis();
  sendStatusPayload(nullptr, "status");
}

void handleSleepingStatus() {
  if (sleepStartMs == 0) {
    sleepStartMs = millis();
  }

  if (millis() - lastSerialTime > 2000) {
    Serial.println("Sleeping");
    lastSerialTime = millis();
  }

  unsigned long sleepWindowMs = (unsigned long)activeConfig.sleepMinutes * 60UL * 1000UL;
  if (!moveBusy && pendingMoveType == MOVE_NONE && sleepWindowMs > 0 && millis() - sleepStartMs >= sleepWindowMs) {
    queueMoveCommand("go", AUTO_WORK_DISTANCE_CM);
    sleepStartMs = millis();
  }
}

void handleBatteryStatus() {
  Serial.println("Battery");
}

void handleErrorStatus() {
  Serial.println("Error");
}

void handleOnReceivedControlCommand() {
  Serial.println("Nhan duoc command");
}
#pragma endregion

#pragma region SETUP



void initVariables() {
  status = STARTED;
  sleepStartMs = millis();
  loadRuntimeConfig();
}

void configHTTPServer() {
  server.on("/health", HTTP_GET, [](AsyncWebServerRequest *request){
    String response = String("{\"status\":\"ok\",\"wifi\":\"") + wifiConnectMessage + "\"}";
    request->send(200, "application/json", response);
  });

  AsyncCallbackJsonWebHandler* wifiConfigHandler =
      new AsyncCallbackJsonWebHandler("/config/wifi", handleUpdateWifiConfig);
  wifiConfigHandler->setMethod(HTTP_POST);
  server.addHandler(wifiConfigHandler);

    AsyncCallbackJsonWebHandler* controlStatusHandler =
      new AsyncCallbackJsonWebHandler("/control/status", handleControlStatus);
    controlStatusHandler->setMethod(HTTP_POST);
    server.addHandler(controlStatusHandler);

    AsyncCallbackJsonWebHandler* controlMoveHandler =
      new AsyncCallbackJsonWebHandler("/control/move", handleControlMove);
    controlMoveHandler->setMethod(HTTP_POST);
    server.addHandler(controlMoveHandler);

  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);
}

#pragma endregion



void run() {
  switch (status) {
    case STARTED:
      handleOnStartedStatus();
      break;
    case OFF:
      handleOffStatus();
      break;
    case WAIT_CONNECT:
      handleWaitConnectStatus();
      break;
    case BEGIN_JOB:
      handleBeginJobStatus();
      break;
    case RUNNING:
      handleRunningStatus();
      break;
    case GOING_HOME:
      handleGoingHomeStatus();
      break;
    case SLEEPING:
      handleSleepingStatus();
      break;
    case LOW_BATTERY:
      handleBatteryStatus();
      break;
    case ERROR:
      handleErrorStatus();
      break;
    default:
      break;
  }
}

#pragma region ZIGZAG_NAVIGATION

void saveReferenceYaw() {
  updateYaw();
  referenceYaw = currentYaw;
  Serial.printf("Reference Yaw saved: %.2f°\n", referenceYaw);
}

float getYawDeviation() {
  updateYaw();
  return normalizeAngle180(currentYaw - referenceYaw);
}

bool checkYawDeviation() {
  float deviation = abs(getYawDeviation());
  return deviation > zigzagDeviationThresholdN;
}

void correctYawDeviation() {
  float deviation = getYawDeviation();
  if (abs(deviation) <= zigzagDeviationThresholdN) {
    return;
  }
  
  Serial.printf("Correcting yaw deviation: %.2f° (threshold: %.2f°)\n", deviation, zigzagDeviationThresholdN);
  
  // Quay về góc chính với công thức: góc quay = deviation + M (cùng dấu)
  // M là phần bù để bù hao hụt khi quay, phải cùng dấu với deviation
  // VD: deviation = 10° (lệch phải) → quay 10° + 28° = 38° (quay trái nhiều hơn để bù)
  //     deviation = -10° (lệch trái) → quay -10° + (-28°) = -38° (quay phải nhiều hơn để bù)
  float offsetSign = (deviation > 0) ? 1.0f : -1.0f;
  float correctionAngle = deviation + (offsetSign * zigzagAngleOffsetM);
  
  // Kiểm tra góc quay có đủ lớn không (phải >= MIN_TURN_ANGLE để thắng ma sát)
  if (abs(correctionAngle) < MIN_TURN_ANGLE_DEG) {
    Serial.printf("Correction angle %.2f° too small (< %.1f°), skipping turn\n", 
                  correctionAngle, MIN_TURN_ANGLE_DEG);
    return;
  }
  
  Serial.printf("Correction angle: %.2f° (deviation %.2f° + M*sign %.2f°)\n", 
                correctionAngle, deviation, offsetSign * zigzagAngleOffsetM);
  
  quay(correctionAngle);
  
  delay(500);
  
  Serial.println("Yaw correction completed");
}

void zigzagTurn90AndShift() {
  if (zigzagGoingHome) {
    return; // Không làm gì khi đang về nhà
  }
  
  stopMotors();
  delay(500);
  
  // Bước 1: Quay 90° + N (trái hoặc phải tùy lượt)
  // Lưu ý: N được cộng vào góc quay để bù hao hụt, nhưng MPU vẫn đo góc chuẩn
  float turnAngle = zigzagTurnLeft ? -(90.0f + zigzagDeviationThresholdN) : (90.0f + zigzagDeviationThresholdN);
  Serial.printf("Turning %s: %.2f° (90 + N)\n", zigzagTurnLeft ? "LEFT" : "RIGHT", turnAngle);
  quay(turnAngle);
  
  delay(1000);
  
  // Bước 2: Đi thẳng độ dịch L và kiểm tra vật cản
  Serial.printf("Shifting by %.1f cm\n", zigzagOffsetDistanceL);
  
  // Kiểm tra vật cản trước khi đi
  distM = getDist(TRIG_M, ECHO_M);
  if (distM <= activeConfig.wallStopDistanceCm) {
    Serial.println("Obstacle detected before shift! Initiating return home sequence...");
    
    // Quay 90 + N về phía ngược lại (huỷ lần quay trước)
    float reverseAngle = zigzagTurnLeft ? (90.0f + zigzagDeviationThresholdN) : -(90.0f + zigzagDeviationThresholdN);
    Serial.printf("Reversing turn: %.2f°\n", reverseAngle);
    quay(reverseAngle);
    delay(1000);
    
    // Quay 180° để đi ngược về
    Serial.println("Turning 180° to go back");
    quay(180.0f);
    delay(1000);
    
    // Đánh dấu đang đi về nhà
    zigzagGoingHome = true;
    Serial.println("Now going home mode");
    return;
  }
  
  // Đi thẳng quãng L
  di_thang(zigzagOffsetDistanceL);
  delay(1000);
  
  // Bước 3: Quay về hướng chính (referenceYaw)
  // Công thức: góc quay = độ lệch + N (N để bù hao hụt, cùng dấu với độ lệch)
  float deviationAfterShift = getYawDeviation();
  
  // Nếu độ lệch rất nhỏ (< MIN_TURN_ANGLE - N), không cần quay
  if (abs(deviationAfterShift) < (MIN_TURN_ANGLE_DEG - zigzagDeviationThresholdN)) {
    Serial.printf("Deviation after shift %.2f° is very small, skipping return turn\n", deviationAfterShift);
  } else {
    float signDeviation = (deviationAfterShift > 0) ? 1.0f : -1.0f;
    float returnAngle = deviationAfterShift + (signDeviation * zigzagDeviationThresholdN);
    
    // Đảm bảo góc quay >= MIN_TURN_ANGLE
    if (abs(returnAngle) < MIN_TURN_ANGLE_DEG) {
      returnAngle = signDeviation * MIN_TURN_ANGLE_DEG;
      Serial.printf("Return angle too small, using minimum: %.2f°\n", returnAngle);
    }
    
    Serial.printf("Returning to reference (%.2f°): turn %.2f° (deviation %.2f° + N*sign %.2f°)\n", 
                  referenceYaw, returnAngle, deviationAfterShift, signDeviation * zigzagDeviationThresholdN);
    quay(returnAngle);
  }
  
  delay(500);
  
  // Bước 4: Tiến thêm 15-20cm để THOÁT KHỎI VÙNG TƯỜNG
  // (Tránh bị kẹt trong vòng lặp quay liên tục)
  Serial.println("Advancing 18cm to clear turn zone...");
  di_thang(18.0f);
  delay(500);
  
  // Đổi hướng quay cho lần sau (luân phiên trái/phải)
  zigzagTurnLeft = !zigzagTurnLeft;
  zigzagRowCount++;
  
  Serial.printf("=== Zigzag row %d completed ===\n", zigzagRowCount);
}

void zigzagNavigate() {
  if (zigzagGoingHome) {
    Serial.println("Going home - retracing path");
    // TODO: Implement path retracing
    return;
  }
  
  // Bắt đầu di chuyển thẳng theo referenceYaw (góc tham chiếu toàn cục)
  updateYaw();
  float targetStraightYaw = referenceYaw; // Đi thẳng theo góc tham chiếu chung
  
  long startLeftCount = 0;
  long startRightCount = 0;
  readEncoderCounts(startLeftCount, startRightCount);
  long startReferenceCount = getReferenceWheelCount(startLeftCount, startRightCount);
  
  long prevReferenceCount = startReferenceCount;
  long prevFollowerCount = REFERENCE_WHEEL_IS_LEFT ? startRightCount : startLeftCount;
  
  int referenceSpeed = constrain(activeConfig.goSpeed, 0, 255);
  unsigned long lastControlMs = millis();
  unsigned long lastCheckMs = millis();
  
  // Bắt đầu di chuyển
  setMotors(referenceSpeed, referenceSpeed);
  
  while (true) {
    if (stopRequested) {
      stopMotors();
      return;
    }
    
    // Đọc quãng đường đã đi
    long leftCount = 0;
    long rightCount = 0;
    readEncoderCounts(leftCount, rightCount);
    long referenceCount = getReferenceWheelCount(leftCount, rightCount);
    float distanceTraveled = pulsesToDistanceCm(abs(referenceCount - startReferenceCount), activeConfig.wheelRadiusCm);
    
    // Kiểm tra vật cản mỗi 200ms, NHƯNG CHỈ SAU KHI ĐÃ ĐI >= 15cm
    // (Tránh quay liên tục khi vừa mới quay xong chưa kịp đi xa)
    if (millis() - lastCheckMs >= 200) {
      if (distanceTraveled >= MIN_DISTANCE_BEFORE_WALL_CHECK_CM) {
        distM = getDist(TRIG_M, ECHO_M);
        if (distM <= activeConfig.wallStopDistanceCm) {
          Serial.printf("Obstacle ahead at %.1f cm after traveling %.1f cm! Turning...\n", distM, distanceTraveled);
          stopMotors();
          delay(500);
          zigzagTurn90AndShift();
          return;
        }
      }
      lastCheckMs = millis();
    }
    
    // Kiểm tra độ lệch góc LỚN so với referenceYaw (góc tham chiếu toàn cục)
    // Lưu ý: PID đã tự động điều chỉnh nhỏ liên tục, đây là điều chỉnh lớn khi lệch quá mức
    updateYaw();
    float currentDeviation = normalizeAngle180(currentYaw - referenceYaw);
    if (abs(currentDeviation) > zigzagDeviationThresholdN) {
      Serial.printf("\n*** MAJOR DEVIATION: %.2f° > %.2f° ***\n", currentDeviation, zigzagDeviationThresholdN);
      stopMotors();
      delay(1000); // Dừng 1s
      correctYawDeviation();
      
      // Sau khi điều chỉnh, tiếp tục đi theo referenceYaw
      // KHÔNG reset startYaw = currentYaw vì sẽ làm mất hướng tham chiếu!
      updateYaw();
      targetStraightYaw = referenceYaw; // Vẫn giữ hướng tham chiếu
      
      // Reset encoder từ vị trí hiện tại
      readEncoderCounts(leftCount, rightCount);
      startReferenceCount = getReferenceWheelCount(leftCount, rightCount);
      prevReferenceCount = startReferenceCount;
      prevFollowerCount = REFERENCE_WHEEL_IS_LEFT ? rightCount : leftCount;
      
      setMotors(referenceSpeed, referenceSpeed);
      lastControlMs = millis();
      Serial.printf("Resuming straight movement toward reference yaw: %.2f°\n", referenceYaw);
      continue;
    }
    
    // Kiểm tra đã đi hết quãng đường K chưa
    if (distanceTraveled >= zigzagStraightDistanceK) {
      Serial.printf("Reached max distance K: %.1f cm\n", distanceTraveled);
      stopMotors();
      delay(500);
      zigzagTurn90AndShift();
      return;
    }
    
    // Điều chỉnh cân bằng 2 bánh mỗi 40ms để TỰ ĐỘNG duy trì hướng thẳng
    unsigned long now = millis();
    if (now - lastControlMs >= STRAIGHT_CONTROL_INTERVAL_MS) {
      lastControlMs = now;
      updateYaw();
      
      long followerCount = REFERENCE_WHEEL_IS_LEFT ? rightCount : leftCount;
      long deltaReference = referenceCount - prevReferenceCount;
      long deltaFollower = followerCount - prevFollowerCount;
      prevReferenceCount = referenceCount;
      prevFollowerCount = followerCount;
      
      // Điều chỉnh LIÊN TỤC để duy trì targetStraightYaw (= referenceYaw)
      // Đây là cơ chế "tự động điều chỉnh hướng" chính
      float yawError = normalizeAngle180(targetStraightYaw - currentYaw);
      float encoderError = (float)(deltaReference - deltaFollower);
      
      float followerCorrection = (yawError * STRAIGHT_YAW_KP) + (encoderError * ENCODER_BALANCE_KP);
      int followerSpeed = constrain((int)(referenceSpeed + followerCorrection), 0, 255);
      
      if (REFERENCE_WHEEL_IS_LEFT) {
        setMotors(referenceSpeed, followerSpeed);
      } else {
        setMotors(followerSpeed, referenceSpeed);
      }
      
      // Debug: In thông tin điều chỉnh mỗi 2s
      static unsigned long lastDebugPrint = 0;
      if (millis() - lastDebugPrint >= 2000) {
        Serial.printf("Auto-adjust: yawErr=%.1f° encErr=%.0f correction=%.1f\n", 
                      yawError, encoderError, followerCorrection);
        lastDebugPrint = millis();
      }
    }
    
    updateMotorPulsing();
    delay(10);
  }
}

void runZigzagCycle() {
  // Lưu góc tham chiếu khi bắt đầu chu kỳ
  saveReferenceYaw();
  zigzagTurnLeft = true;
  zigzagRowCount = 0;
  zigzagGoingHome = false;
  zigzagHistoryCount = 0;
  
  Serial.println("\n=== STARTING ZIGZAG NAVIGATION ===");
  Serial.printf("Reference Yaw: %.2f°\n", referenceYaw);
  Serial.printf("M (Angle Offset): %.1f°\n", zigzagAngleOffsetM);
  Serial.printf("N (Deviation Threshold): %.1f°\n", zigzagDeviationThresholdN);
  Serial.printf("K (Straight Distance): %.1f cm\n", zigzagStraightDistanceK);
  Serial.printf("L (Offset Distance): %.1f cm\n\n", zigzagOffsetDistanceL);
  
  while (!stopRequested && !zigzagGoingHome && status == RUNNING) {
    zigzagNavigate();
    delay(500);
  }
  
  stopMotors();
  
  if (zigzagGoingHome) {
    Serial.println("\n=== GOING HOME ===");
    status = GOING_HOME;
  } else {
    Serial.println("\n=== ZIGZAG CYCLE COMPLETED ===");
  }
}

#pragma endregion

void setup() {
  pinMode(signalPin, OUTPUT);
  
  Serial.begin(115200);

  initMoveHardware();

  initVariables();
  configHTTPServer();

  String storedSsid;
  String storedPass;
  if (loadStoredWiFi(storedSsid, storedPass)) {
    Serial.println("Da tim thay WiFi da luu, dang thu ket noi lai");
    queueWiFiConnection(storedSsid, storedPass);
    status = SLEEPING;
    sleepStartMs = millis();
  } else {
    startAccessPoint();
  }
}

void loop() {
  processWiFiConnectionTask();
  handleUdpDiscoveryRequest();
  processMoveCommandTask();
  updateMotorPulsing();
  ws.cleanupClients();
  pushStatusIfNeeded();

  // Safety: if STA is connected (including auto reconnect), ensure HTTP server is bound.
  if (WiFi.status() == WL_CONNECTED && !staServerReady) {
    WiFi.mode(WIFI_STA);
    restartServerListener();
    startUdpDiscoveryIfNeeded();
    staServerReady = true;
  }

  run();
}

void updatePulseFromConfig(const RobotRuntimeConfig& cfg) {
  g_usePulsedPower = cfg.pulseEnabled;

  float freqHz = constrain(cfg.pulseFreqHz, 1.0f, 200.0f);
  float dutyPercent = constrain(cfg.pulseDutyPercent, 5.0f, 95.0f);
  float powerPercent = constrain(cfg.pulsePowerPercent, 10.0f, 100.0f);

  float periodMsF = 1000.0f / freqHz;
  unsigned long periodMs = (unsigned long)max(1.0f, periodMsF);
  unsigned long onMs = (unsigned long)max(1.0f, periodMsF * (dutyPercent / 100.0f));
  if (onMs > periodMs) {
    onMs = periodMs;
  }

  g_pulsePeriodMs = periodMs;
  g_pulseOnMs = onMs;
  g_pulsePowerScale = powerPercent / 100.0f;
}