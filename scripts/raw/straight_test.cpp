#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>

const int ENA = 13;
const int IN1 = 12;
const int IN2 = 14;
const int ENB = 25;
const int IN3 = 27;
const int IN4 = 26;
const int ENCODER_LEFT = 34;
const int ENCODER_RIGHT = 35;
const int MPU_ADDR = 0x68;

const bool LEFT_MOTOR_REVERSED = false;
const bool RIGHT_MOTOR_REVERSED = true;
const bool REFERENCE_WHEEL_IS_LEFT = true;

const char *WIFI_SSID = "TP-Link_F482";
const char *WIFI_PASS = "10303115";
const char *FALLBACK_AP_SSID = "RobotTune";
const char *FALLBACK_AP_PASS = "12345678";

const unsigned long START_DELAY_MS = 5000;
const float TARGET_DISTANCE_CM = 50.0f;
const int MAX_PWM = 80;
const float PULSES_PER_CM = 2.0f;
const unsigned long RUN_TIMEOUT_MS = 25000;
const unsigned long REALIGN_TIMEOUT_MS = 3500;

const float GYRO_SCALE_DPS = 131.0f;
const int GYRO_CALIB_SAMPLES = 1500;
const float GYRO_LPF_ALPHA = 0.38f;
const float GYRO_DEADBAND_DPS = 0.03f;
const float GYRO_MAX_VALID_DPS = 300.0f;

const int START_YAW_SAMPLE_COUNT = 15;
const unsigned long START_YAW_SAMPLE_DELAY_MS = 5;

int tuneStartSpeed = 14;
float tuneMovingFactor = 0.55f;
float tuneSlowdownAfterCm = 1.0f;
int tuneBoostSpeed = 40;
unsigned long tuneBoostMs = 260;
unsigned long tuneStallCheckMs = 80;
int tuneStallPulseMinDelta = 1;
unsigned long tuneStallBoostMs = 120;
unsigned long tuneHeadingIntervalMs = 10;
float tuneYawKp = 1.2f;
float tuneYawDeadbandDeg = 0.35f;
int tuneMaxYawCorrection = 8;
float tuneYawCorrectionSign = -1.0f;
float tuneLeftMotorGain = 1.00f;
float tuneRightMotorGain = 1.00f;
float tuneRealignTriggerDeg = 10.0f;
float tuneRealignToleranceDeg = 2.0f;
int tuneRealignMinSpeed = 14;
int tuneRealignMaxSpeed = 30;

volatile long pulseCountLeft = 0;
volatile long pulseCountRight = 0;

unsigned long bootMs = 0;
unsigned long lastLogMs = 0;
bool autoStarted = false;
bool runRequested = false;
bool runActive = false;
bool stopRequested = false;
bool realignActive = false;

float gyroOffsetZ = 0.0f;
float gyroZDpsFiltered = 0.0f;
float currentYaw = 0.0f;
unsigned long lastGyroUs = 0;

float teleDistanceCm = 0.0f;
float teleYawErrDeg = 0.0f;
int teleAppliedSpeed = 0;
int teleLeftOut = 0;
int teleRightOut = 0;

WebServer server(80);

const char PAGE_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>Robot Straight Tune</title>
<style>
body{font-family:Segoe UI,Arial,sans-serif;background:#0f172a;color:#e2e8f0;margin:0;padding:16px}
.card{background:#111827;border:1px solid #334155;border-radius:12px;padding:12px;margin-bottom:12px}
.grid{display:grid;grid-template-columns:1fr 110px;gap:8px;align-items:center}
input{width:100%;padding:6px;border-radius:8px;border:1px solid #334155;background:#1e293b;color:#e2e8f0}
button{padding:10px 12px;border-radius:10px;border:none;background:#22c55e;color:#052e16;font-weight:700}
button.alt{background:#38bdf8;color:#082f49}
button.stop{background:#f43f5e;color:#450a0a}
.small{opacity:.8;font-size:12px}
</style>
</head>
<body>
<div class="card"><h3>Tinh Chinh Di Thang (Realtime)</h3><div id="state" class="small">dang tai...</div></div>
<div class="card small">
Tham so de hieu nhanh:
<br>- Toc do dau: toc do luc vua bat dau chay
<br>- He so giam toc: ti le toc do sau khi da lan banh
<br>- Toc do day luc / thoi gian day: xung tang luc de thoat i
<br>- Yaw Kp: do manh giu huong theo MPU
<br>- Yaw Sign: dao chieu bu huong (1 hoac -1)
<br>- Ti le luc trai/phai: can bang hai ben dong co
</div>
<div class="card">
<div class="grid"><label>Toc do dau (startSpeed)</label><input id="startSpeed" type="number"></div>
<div class="grid"><label>He so giam toc (movingFactor)</label><input id="movingFactor" type="number" step="0.01"></div>
<div class="grid"><label>Bat dau giam sau (cm)</label><input id="slowdownAfterCm" type="number" step="0.1"></div>
<div class="grid"><label>Toc do day luc (boostSpeed)</label><input id="boostSpeed" type="number"></div>
<div class="grid"><label>Thoi gian day luc (ms)</label><input id="boostMs" type="number"></div>
<div class="grid"><label>Chu ky check i (ms)</label><input id="stallCheckMs" type="number"></div>
<div class="grid"><label>Nguong xung toi thieu</label><input id="stallPulseMinDelta" type="number"></div>
<div class="grid"><label>Do dai xung day (ms)</label><input id="stallBoostMs" type="number"></div>
<div class="grid"><label>Do manh giu huong (yawKp)</label><input id="yawKp" type="number" step="0.01"></div>
<div class="grid"><label>Vung bo qua lech goc</label><input id="yawDeadbandDeg" type="number" step="0.01"></div>
<div class="grid"><label>Gioi han bu huong max</label><input id="maxYawCorrection" type="number"></div>
<div class="grid"><label>Chieu bu huong (1/-1)</label><input id="yawSign" type="number" step="1"></div>
<div class="grid"><label>Chu ky tinh huong (ms)</label><input id="headingIntervalMs" type="number"></div>
<div class="grid"><label>Ti le luc dong co trai</label><input id="leftMotorGain" type="number" step="0.01"></div>
<div class="grid"><label>Ti le luc dong co phai</label><input id="rightMotorGain" type="number" step="0.01"></div>
<div class="grid"><label>Nguong vao chinh huong (deg)</label><input id="realignTriggerDeg" type="number" step="0.1"></div>
<div class="grid"><label>Dung sai thoat chinh (deg)</label><input id="realignToleranceDeg" type="number" step="0.1"></div>
<div class="grid"><label>Toc do xoay min</label><input id="realignMinSpeed" type="number"></div>
<div class="grid"><label>Toc do xoay max</label><input id="realignMaxSpeed" type="number"></div>
</div>
<div class="card">
<button onclick="applyCfg()">Ap dung</button>
<button class="alt" onclick="startRun()">Bat dau</button>
<button class="stop" onclick="stopRun()">Dung</button>
</div>
<script>
async function j(url,opt){const r=await fetch(url,opt);return r.json();}
let isEditing = false;
let inputsBound = false;

function bindEditEvents(){
if(inputsBound) return;
inputsBound = true;
document.querySelectorAll('input').forEach(e=>{
e.addEventListener('focus',()=>{isEditing=true;});
e.addEventListener('blur',()=>{isEditing=false;});
});
}

function setVals(c){
for(const k in c){
const e=document.getElementById(k);
if(!e) continue;
if(document.activeElement===e) continue;
if(isEditing) continue;
e.value=c[k];
}
}
async function refresh(){
const s=await j('/api/state');
setVals(s.config);
document.getElementById('state').textContent=
`runActive=${s.runActive} runRequested=${s.runRequested} realign=${s.realignActive} d=${s.tele.distanceCm.toFixed(1)} yawErr=${s.tele.yawErrDeg.toFixed(2)} speed=${s.tele.appliedSpeed} L=${s.tele.leftOut} R=${s.tele.rightOut}`;
}
async function applyCfg(){
const fd=new URLSearchParams();
['startSpeed','movingFactor','slowdownAfterCm','boostSpeed','boostMs','stallCheckMs','stallPulseMinDelta','stallBoostMs','yawKp','yawDeadbandDeg','maxYawCorrection','yawSign','headingIntervalMs','leftMotorGain','rightMotorGain','realignTriggerDeg','realignToleranceDeg','realignMinSpeed','realignMaxSpeed']
.forEach(k=>fd.append(k,document.getElementById(k).value));
await j('/api/config',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:fd});
await refresh();
}
async function startRun(){await j('/api/start',{method:'POST'});}
async function stopRun(){await j('/api/stop',{method:'POST'});}
bindEditEvents();
setInterval(refresh,500);refresh();
</script>
</body></html>
)HTML";

void IRAM_ATTR countPulseLeft() { pulseCountLeft++; }
void IRAM_ATTR countPulseRight() { pulseCountRight++; }

void setupEncoderInputPin(int pin) {
  if (pin >= 34 && pin <= 39) pinMode(pin, INPUT);
  else pinMode(pin, INPUT_PULLUP);
}

void readEncoderCounts(long &leftCount, long &rightCount) {
  noInterrupts();
  leftCount = pulseCountLeft;
  rightCount = pulseCountRight;
  interrupts();
}

long getReferenceWheelCount(long leftCount, long rightCount) {
  return REFERENCE_WHEEL_IS_LEFT ? leftCount : rightCount;
}

float pulsesToDistanceCm(long pulses) {
  return (PULSES_PER_CM > 0.0f) ? ((float)pulses / PULSES_PER_CM) : 0.0f;
}

float normalizeAngle180(float angleDeg) {
  while (angleDeg > 180.0f) angleDeg -= 360.0f;
  while (angleDeg < -180.0f) angleDeg += 360.0f;
  return angleDeg;
}

void mpuWriteRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission(true);
}

bool readMpu16(uint8_t reg, int16_t &value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  int got = Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)2, (uint8_t)true);
  if (got != 2 || Wire.available() < 2) return false;
  value = (int16_t)((Wire.read() << 8) | Wire.read());
  return true;
}

void initMpuSensor() {
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
  if (validSamples > 0) gyroOffsetZ = (float)sumZ / (float)validSamples;
  gyroZDpsFiltered = 0.0f;
  currentYaw = 0.0f;
}

void updateYaw() {
  int16_t rawZ = 0;
  if (!readMpu16(0x47, rawZ)) return;
  unsigned long nowUs = micros();
  float dt = (nowUs - lastGyroUs) / 1000000.0f;
  lastGyroUs = nowUs;
  if (dt <= 0.0f || dt > 0.05f) return;

  float gyroRawDps = (rawZ - gyroOffsetZ) / GYRO_SCALE_DPS;
  if (fabsf(gyroRawDps) > GYRO_MAX_VALID_DPS) return;

  gyroZDpsFiltered += GYRO_LPF_ALPHA * (gyroRawDps - gyroZDpsFiltered);
  float gyroForIntegration = (fabsf(gyroZDpsFiltered) < GYRO_DEADBAND_DPS) ? 0.0f : gyroZDpsFiltered;
  currentYaw += gyroForIntegration * dt;
}

float captureStartYawStill() {
  float sumYaw = 0.0f;
  for (int i = 0; i < START_YAW_SAMPLE_COUNT; i++) {
    updateYaw();
    sumYaw += currentYaw;
    delay(START_YAW_SAMPLE_DELAY_MS);
  }
  return sumYaw / (float)START_YAW_SAMPLE_COUNT;
}

void setMotors(int speedL, int speedR) {
  int leftCmd = constrain((int)(speedL * tuneLeftMotorGain), -MAX_PWM, MAX_PWM);
  int rightCmd = constrain((int)(speedR * tuneRightMotorGain), -MAX_PWM, MAX_PWM);

  leftCmd = LEFT_MOTOR_REVERSED ? -leftCmd : leftCmd;
  rightCmd = RIGHT_MOTOR_REVERSED ? -rightCmd : rightCmd;

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

  analogWrite(ENA, map(leftCmd, 0, MAX_PWM, 0, 255));
  analogWrite(ENB, map(rightCmd, 0, MAX_PWM, 0, 255));
}

void stopMotors() { setMotors(0, 0); }

String stateJson() {
  String s = "{";
  s += "\"runActive\":" + String(runActive ? "true" : "false");
  s += ",\"runRequested\":" + String(runRequested ? "true" : "false");
  s += ",\"realignActive\":" + String(realignActive ? "true" : "false");
  s += ",\"tele\":{";
  s += "\"distanceCm\":" + String(teleDistanceCm, 2);
  s += ",\"yawErrDeg\":" + String(teleYawErrDeg, 2);
  s += ",\"appliedSpeed\":" + String(teleAppliedSpeed);
  s += ",\"leftOut\":" + String(teleLeftOut);
  s += ",\"rightOut\":" + String(teleRightOut);
  s += "},\"config\":{";
  s += "\"startSpeed\":" + String(tuneStartSpeed);
  s += ",\"movingFactor\":" + String(tuneMovingFactor, 2);
  s += ",\"slowdownAfterCm\":" + String(tuneSlowdownAfterCm, 2);
  s += ",\"boostSpeed\":" + String(tuneBoostSpeed);
  s += ",\"boostMs\":" + String((unsigned long)tuneBoostMs);
  s += ",\"stallCheckMs\":" + String((unsigned long)tuneStallCheckMs);
  s += ",\"stallPulseMinDelta\":" + String(tuneStallPulseMinDelta);
  s += ",\"stallBoostMs\":" + String((unsigned long)tuneStallBoostMs);
  s += ",\"yawKp\":" + String(tuneYawKp, 2);
  s += ",\"yawDeadbandDeg\":" + String(tuneYawDeadbandDeg, 2);
  s += ",\"maxYawCorrection\":" + String(tuneMaxYawCorrection);
  s += ",\"yawSign\":" + String(tuneYawCorrectionSign, 0);
  s += ",\"headingIntervalMs\":" + String((unsigned long)tuneHeadingIntervalMs);
  s += ",\"leftMotorGain\":" + String(tuneLeftMotorGain, 2);
  s += ",\"rightMotorGain\":" + String(tuneRightMotorGain, 2);
  s += ",\"realignTriggerDeg\":" + String(tuneRealignTriggerDeg, 2);
  s += ",\"realignToleranceDeg\":" + String(tuneRealignToleranceDeg, 2);
  s += ",\"realignMinSpeed\":" + String(tuneRealignMinSpeed);
  s += ",\"realignMaxSpeed\":" + String(tuneRealignMaxSpeed);
  s += "}}";
  return s;
}

void handleConfigPost() {
  if (server.hasArg("startSpeed")) tuneStartSpeed = constrain(server.arg("startSpeed").toInt(), 0, MAX_PWM);
  if (server.hasArg("movingFactor")) tuneMovingFactor = constrain(server.arg("movingFactor").toFloat(), 0.4f, 1.0f);
  if (server.hasArg("slowdownAfterCm")) tuneSlowdownAfterCm = constrain(server.arg("slowdownAfterCm").toFloat(), 0.0f, 50.0f);
  if (server.hasArg("boostSpeed")) tuneBoostSpeed = constrain(server.arg("boostSpeed").toInt(), 0, MAX_PWM);
  if (server.hasArg("boostMs")) tuneBoostMs = constrain((unsigned long)server.arg("boostMs").toInt(), 20UL, 2000UL);
  if (server.hasArg("stallCheckMs")) tuneStallCheckMs = constrain((unsigned long)server.arg("stallCheckMs").toInt(), 40UL, 2000UL);
  if (server.hasArg("stallPulseMinDelta")) tuneStallPulseMinDelta = constrain(server.arg("stallPulseMinDelta").toInt(), 0, 20);
  if (server.hasArg("stallBoostMs")) tuneStallBoostMs = constrain((unsigned long)server.arg("stallBoostMs").toInt(), 20UL, 2000UL);
  if (server.hasArg("yawKp")) tuneYawKp = constrain(server.arg("yawKp").toFloat(), 0.0f, 5.0f);
  if (server.hasArg("yawDeadbandDeg")) tuneYawDeadbandDeg = constrain(server.arg("yawDeadbandDeg").toFloat(), 0.0f, 5.0f);
  if (server.hasArg("maxYawCorrection")) tuneMaxYawCorrection = constrain(server.arg("maxYawCorrection").toInt(), 0, 30);
  if (server.hasArg("yawSign")) tuneYawCorrectionSign = (server.arg("yawSign").toInt() >= 0) ? 1.0f : -1.0f;
  if (server.hasArg("headingIntervalMs")) tuneHeadingIntervalMs = constrain((unsigned long)server.arg("headingIntervalMs").toInt(), 10UL, 300UL);
  if (server.hasArg("leftMotorGain")) tuneLeftMotorGain = constrain(server.arg("leftMotorGain").toFloat(), 0.50f, 1.50f);
  if (server.hasArg("rightMotorGain")) tuneRightMotorGain = constrain(server.arg("rightMotorGain").toFloat(), 0.50f, 1.50f);
  if (server.hasArg("realignTriggerDeg")) tuneRealignTriggerDeg = constrain(server.arg("realignTriggerDeg").toFloat(), 2.0f, 90.0f);
  if (server.hasArg("realignToleranceDeg")) tuneRealignToleranceDeg = constrain(server.arg("realignToleranceDeg").toFloat(), 0.5f, 20.0f);
  if (server.hasArg("realignMinSpeed")) tuneRealignMinSpeed = constrain(server.arg("realignMinSpeed").toInt(), 0, MAX_PWM);
  if (server.hasArg("realignMaxSpeed")) tuneRealignMaxSpeed = constrain(server.arg("realignMaxSpeed").toInt(), tuneRealignMinSpeed, MAX_PWM);
  server.send(200, "application/json", "{\"ok\":true}");
}

int computeRealignTurnCommand(float yawErrorDeg) {
  float absErr = fabsf(yawErrorDeg);
  int turnSpeed = constrain((int)(absErr * tuneYawKp), tuneRealignMinSpeed, tuneRealignMaxSpeed);
  int turnDir = (yawErrorDeg >= 0.0f) ? 1 : -1;
  if (tuneYawCorrectionSign < 0.0f) turnDir = -turnDir;
  return turnDir * turnSpeed;
}

void realignToStartYaw(float startYaw) {
  realignActive = true;
  stopMotors();
  delay(40);

  unsigned long realignStartMs = millis();
  unsigned long lastRealignLogMs = 0;

  while (!stopRequested) {
    server.handleClient();
    updateYaw();

    float err = normalizeAngle180(startYaw - currentYaw);
    if (fabsf(err) <= tuneRealignToleranceDeg) break;

    if (millis() - realignStartMs >= REALIGN_TIMEOUT_MS) {
      Serial.println("Realign timeout, thoat che do chinh huong");
      break;
    }

    int turnCmd = computeRealignTurnCommand(err);
    setMotors(-turnCmd, turnCmd);

    if (millis() - lastRealignLogMs >= 120) {
      Serial.print("Realign | err=");
      Serial.print(err, 2);
      Serial.print(" turn=");
      Serial.println(turnCmd);
      lastRealignLogMs = millis();
    }

    delay(8);
  }

  stopMotors();
  realignActive = false;
}

void setupWeb() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  unsigned long startMs = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startMs < 12000) {
    delay(300);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("WiFi connected. IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println();
    Serial.println("WiFi connect failed, fallback AP mode");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(FALLBACK_AP_SSID, FALLBACK_AP_PASS);
    Serial.print("Fallback AP IP: ");
    Serial.println(WiFi.softAPIP());
  }

  server.on("/", HTTP_GET, []() { server.send(200, "text/html", PAGE_HTML); });
  server.on("/api/state", HTTP_GET, []() { server.send(200, "application/json", stateJson()); });
  server.on("/api/config", HTTP_POST, handleConfigPost);
  server.on("/api/start", HTTP_POST, []() {
    stopRequested = false;
    runRequested = true;
    server.send(200, "application/json", "{\"ok\":true}");
  });
  server.on("/api/stop", HTTP_POST, []() {
    stopRequested = true;
    server.send(200, "application/json", "{\"ok\":true}");
  });

  server.begin();
  Serial.println("Web tune server ready on port 80");
}

void runStraightDistance(float targetDistanceCm) {
  runActive = true;
  stopRequested = false;

  noInterrupts();
  pulseCountLeft = 0;
  pulseCountRight = 0;
  interrupts();

  stopMotors();
  updateYaw();
  float startYaw = captureStartYawStill();

  int startSpeed = constrain(tuneStartSpeed, 0, MAX_PWM);
  int movingSpeed = constrain((int)(startSpeed * tuneMovingFactor + 0.5f), 0, MAX_PWM);
  int boostSpeed = constrain(tuneBoostSpeed, startSpeed, MAX_PWM);
  int appliedSpeed = boostSpeed;
  int baseSpeed = startSpeed;

  bool launchBoostDone = false;
  bool speedReduced = false;
  bool stallBoostActive = false;

  unsigned long launchBoostStartMs = millis();
  unsigned long stallBoostUntilMs = 0;
  unsigned long lastStallCheckMs = 0;
  unsigned long startMs = millis();

  long startLeft = 0;
  long startRight = 0;
  readEncoderCounts(startLeft, startRight);
  long startRef = getReferenceWheelCount(startLeft, startRight);
  long lastStallRef = startRef;

  float yawError = 0.0f;
  int yawCorrection = 0;

  setMotors(appliedSpeed, appliedSpeed);

  while (true) {
    server.handleClient();

    if (stopRequested) {
      Serial.println("Straight test: stopped by user");
      break;
    }

    long leftCount = 0;
    long rightCount = 0;
    readEncoderCounts(leftCount, rightCount);

    long refNow = getReferenceWheelCount(leftCount, rightCount);
    float traveledCm = pulsesToDistanceCm(abs(refNow - startRef));
    unsigned long now = millis();

    updateYaw();
    yawError = normalizeAngle180(startYaw - currentYaw);

    if (fabsf(yawError) >= tuneRealignTriggerDeg) {
      Serial.print("Yaw lech lon, vao che do chinh huong: ");
      Serial.println(yawError, 2);
      realignToStartYaw(startYaw);

      // Reset anti-stall timing after realign pause.
      readEncoderCounts(leftCount, rightCount);
      refNow = getReferenceWheelCount(leftCount, rightCount);
      lastStallRef = refNow;
      lastStallCheckMs = millis();
      stallBoostActive = false;
      yawCorrection = 0;
      continue;
    }

    float correction = yawError * tuneYawKp * tuneYawCorrectionSign;
    if (fabsf(yawError) < tuneYawDeadbandDeg) correction = 0.0f;
    int correctionLimit = min(tuneMaxYawCorrection, max(appliedSpeed - 2, 0));
    yawCorrection = constrain((int)correction, -correctionLimit, correctionLimit);

    if (!launchBoostDone && now - launchBoostStartMs >= tuneBoostMs) {
      launchBoostDone = true;
      baseSpeed = startSpeed;
      appliedSpeed = baseSpeed;
    }

    if (!speedReduced && traveledCm >= tuneSlowdownAfterCm) {
      baseSpeed = movingSpeed;
      if (!stallBoostActive) appliedSpeed = baseSpeed;
      speedReduced = true;
    }

    if (launchBoostDone && !stallBoostActive && now - lastStallCheckMs >= tuneStallCheckMs) {
      long pulseDelta = abs(refNow - lastStallRef);
      if (pulseDelta <= tuneStallPulseMinDelta) {
        stallBoostActive = true;
        stallBoostUntilMs = now + tuneStallBoostMs;
        appliedSpeed = boostSpeed;
      }
      lastStallRef = refNow;
      lastStallCheckMs = now;
    }

    if (stallBoostActive && now >= stallBoostUntilMs) {
      stallBoostActive = false;
      appliedSpeed = baseSpeed;
    }

    if (traveledCm >= targetDistanceCm) {
      Serial.println("Straight test: target reached");
      break;
    }

    if (now - startMs > RUN_TIMEOUT_MS) {
      Serial.println("Straight test: timeout");
      break;
    }

    int leftOut = constrain(appliedSpeed - yawCorrection, 0, MAX_PWM);
    int rightOut = constrain(appliedSpeed + yawCorrection, 0, MAX_PWM);
    setMotors(leftOut, rightOut);

    teleDistanceCm = traveledCm;
    teleYawErrDeg = yawError;
    teleAppliedSpeed = appliedSpeed;
    teleLeftOut = leftOut;
    teleRightOut = rightOut;

    if (now - lastLogMs >= 300) {
      Serial.print("Run | d=");
      Serial.print(traveledCm, 1);
      Serial.print(" yawErr=");
      Serial.print(yawError, 2);
      Serial.print(" speed=");
      Serial.print(appliedSpeed);
      Serial.print(" L=");
      Serial.print(leftOut);
      Serial.print(" R=");
      Serial.println(rightOut);
      lastLogMs = now;
    }

    delay(5);
  }

  stopMotors();
  runActive = false;
  runRequested = false;
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  setupEncoderInputPin(ENCODER_LEFT);
  setupEncoderInputPin(ENCODER_RIGHT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT), countPulseLeft, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT), countPulseRight, FALLING);

  Wire.begin(21, 22);
  initMpuSensor();
  calibrateGyroOffset();
  lastGyroUs = micros();

  setupWeb();

  stopMotors();
  bootMs = millis();

  Serial.println();
  Serial.println("=== STRAIGHT TEST BOOT ===");
  Serial.println("Wait 5s, then auto run once");
}

void loop() {
  server.handleClient();

  if (!autoStarted && millis() - bootMs >= START_DELAY_MS) {
    autoStarted = true;
    runRequested = true;
  }

  if (runRequested && !runActive) {
    runStraightDistance(TARGET_DISTANCE_CM);
    Serial.println("Straight test: done");
  }

  delay(2);
}
