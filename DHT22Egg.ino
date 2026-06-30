// Task 1.1: credentials ย้ายออกจากโค้ด ดู secrets.h
// Task 1.2: WiFiManager — ตั้ง WiFi ครั้งแรกผ่าน captive portal
// Task 1.3: Watchdog 30s + ArduinoOTA
// Task 2.1: Multi-stage profile (อ่านจาก Firebase /incubator/profiles/{name}/stages)
// Task 2.2: หยุดพลิกไข่อัตโนมัติเมื่อ turning=false (Lockdown)
// Task 2.4: Log ทุก 1 นาที (/incubator/logs1m) เพิ่มจาก hourly
// Task 3.2: แจ้งเตือน LINE วันส่องไข่
// Phase 1 Task 1.1-1.4: Multi-device — เปลี่ยน path เป็น /incubators/{DEVICE_ID}/

#include <WiFi.h>
#include <Wire.h>
#include <time.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <Adafruit_SHT4x.h>
#include <ESP32Servo.h>
#include <Firebase_ESP_Client.h>
#include <WiFiManager.h>
#include <ArduinoOTA.h>
#include <esp_task_wdt.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"
#include "secrets.h"

// ===== Firebase =====
FirebaseData fbdo, fbdoCtrl, fbdoLog, fbdoProf;
FirebaseAuth auth;
FirebaseConfig config;

// ===== Hardware =====
Adafruit_SHT4x sht45;
SemaphoreHandle_t i2cMutex;
Servo myServo;

#define RELAY1    26
#define RELAY2    27
#define RELAY3    14
#define RELAY4    25
#define RELAY5    33
#define SERVO_PIN 13
#define WDT_TIMEOUT 30

// Task 1.2: helper — คืน path ใต้ /incubators/{DEVICE_ID}/
String fbPath(const String& sub) {
  return String("/incubators/") + DEVICE_ID + "/" + sub;
}

// ===== Servo =====
const int CENTER = 90, LEFT = 45, RIGHT = 135;
volatile int currentPos = 90;
volatile int servoState = 9;  // 9 = STOPPED
volatile unsigned long servoHoldTimer = 0;
volatile unsigned long servoMoveTimer = 0;
const unsigned long SERVO_STEP_MS = 60;
volatile unsigned long servoHoldMs = 3600000UL;
String servoStatus = "STOPPED";
SemaphoreHandle_t servoStatusMutex;
volatile bool servoResetRequest = false;
volatile bool servoStartRequest = false;
bool turningEnabled = true;

// ===== Thresholds =====
float hum_on = 0, hum_off = 0;
float heater_off_temp = 0, fan_temp = 0, fan_hum = 0;
bool thresholdReady = false;

// ===== Incubation =====
long long endTimeMs = 0;
long long startTimeMs = 0;
int startDayOffset = 0;
bool alertedOneDay = false, alertedThirtyMin = false, alertedDone = false;

// ===== Profile =====
String activeProfileName = "";
int lastProfileDay = -1;

// ===== Candling =====
int  candlingDays[5]    = {10, 18, 0, 0, 0};
int  candlingCount      = 2;
bool candlingAlerted[5] = {false};

// ===== Timing =====
unsigned long shtTimer = 0, firebaseTimer = 0;
unsigned long logHrTimer = 0, logMinTimer = 0, controlTimer = 0;
const unsigned long SHT_INTERVAL      = 3000;
const unsigned long FIREBASE_INTERVAL = 5000;
const unsigned long LOG_HR_INTERVAL   = 3600000UL;
const unsigned long LOG_MIN_INTERVAL  = 60000UL;
const unsigned long CONTROL_INTERVAL  = 3000;

// ===== State =====
float latestTemp = 0, latestHum = 0;
bool fogOn = false, fanMainOn = false, fan3On = true, fan4On = false, heaterOn = false;
String systemState = "STOP";
int shtErrorCount = 0;
const int SHT_MAX_ERROR = 5;
bool sensorFailed = false, wasRunning = false;

// ===== LINE =====
unsigned long lineAlertTimer = 0;
const unsigned long LINE_COOLDOWN = 600000UL;
volatile bool lineSending = false;
unsigned long lineSendingStartMs = 0;

// ─── forward declarations ───
void handleServo();
void sendLineAlert(String message);
void sendLineForce(String message);
long long getNTPTime();

// ===== LINE Task =====
void lineTask(void* param) {
  String* msg = (String*)param;
  vTaskDelay(pdMS_TO_TICKS(2000));
  WiFiClientSecure client;
  client.setInsecure();
  client.setTimeout(15);
  HTTPClient http;
  http.begin(client, "https://api.line.me/v2/bot/message/push");
  http.addHeader("Content-Type",  "application/json");
  http.addHeader("Authorization", "Bearer " + String(LINE_TOKEN));
  http.setTimeout(10000);
  String body = "{\"to\":\"" + String(LINE_USER) + "\","
                "\"messages\":[{\"type\":\"text\",\"text\":\"" + *msg + "\"}]}";
  int code = http.POST(body);
  Serial.println(code == 200 ? "[LINE] OK" : "[LINE] err:" + String(code));
  http.end(); client.stop();
  delete msg;
  lineSending = false;
  vTaskDelete(NULL);
}

void sendLineAlert(String message) {
  if (WiFi.status() != WL_CONNECTED) return;
  unsigned long now = millis();
  if (now - lineAlertTimer < LINE_COOLDOWN) return;
  lineAlertTimer = now;
  struct tm ti; String timeStr = "--";
  if (getLocalTime(&ti)) {
    char buf[20]; strftime(buf, sizeof(buf), "%d/%m/%y %H:%M", &ti); timeStr = String(buf);
  }
  String full = "แจ้งเตือนตู้ฟักไข่\n-----------------\n" + message +
                "-----------------\nเวลา: " + timeStr;
  full.replace("\\", "\\\\"); full.replace("\"", "\\\""); full.replace("\n", "\\n");
  String* copy = new String(full);
  lineSending = true;
  lineSendingStartMs = millis();
  xTaskCreatePinnedToCore(lineTask, "lineTask", 16384, copy, 1, NULL, 0);
}

void sendLineForce(String message) {
  lineAlertTimer = 0;
  sendLineAlert(message);
}

// ===== Servo Task =====
void servoTask(void* param) {
  for (;;) { handleServo(); vTaskDelay(pdMS_TO_TICKS(5)); }
}

// ===== Relay =====
void relayWrite(int pin, bool on) { digitalWrite(pin, on ? LOW : HIGH); }
void ssrWrite(int pin, bool on)   { digitalWrite(pin, on ? HIGH : LOW); }

void allRelaysOff() {
  relayWrite(RELAY1, false); relayWrite(RELAY2, false);
  relayWrite(RELAY3, false); relayWrite(RELAY4, false);
  ssrWrite(RELAY5, false);
  fogOn = fanMainOn = fan3On = fan4On = heaterOn = false;
}

// ===== Servo =====
void resetServo() { servoResetRequest = true; }

void moveTo(int target) {
  if (currentPos == target) return;
  unsigned long now = millis();
  if (now - servoMoveTimer < SERVO_STEP_MS) return;
  servoMoveTimer = now;
  currentPos += (currentPos < target) ? 1 : -1;
  myServo.write(currentPos);
}

void handleServo() {
  static int lastState = -1;
  unsigned long now = millis();

  if (servoResetRequest) {
    servoResetRequest = false;
    servoState = 8;
    servoMoveTimer = now;
    lastState = -1;
    return;
  }
  if (servoStartRequest) {
    servoStartRequest = false;
    servoState = turningEnabled ? 6 : 8;
    lastState = -1;
    return;
  }
  if (!turningEnabled && servoState < 8) {
    servoResetRequest = true;
    return;
  }

  if (servoState != lastState) {
    lastState = servoState;
    const char* s = "";
    switch (servoState) {
      case 0: s = "Move to 135";    break;
      case 1: s = "Hold 135";       break;
      case 2: s = "Move to 90";     break;
      case 3: s = "Hold 90 (mid)";  break;
      case 4: s = "Move to 45";     break;
      case 5: s = "Hold 45";        break;
      case 6: s = "Move to 90";     break;
      case 7: s = "Hold 90 (start)";break;
      case 8: s = "Return to 90";  break;
      case 9: s = "STOPPED";       break;
    }
    xSemaphoreTake(servoStatusMutex, pdMS_TO_TICKS(50));
    servoStatus = String(s);
    xSemaphoreGive(servoStatusMutex);
  }

  unsigned long holdMs = servoHoldMs;
  switch (servoState) {
    case 0: moveTo(RIGHT);  if (currentPos == RIGHT)  { servoHoldTimer = now; servoState = 1; } break;
    case 1: if (now - servoHoldTimer >= holdMs) servoState = 2; break;
    case 2: moveTo(CENTER); if (currentPos == CENTER) { servoHoldTimer = now; servoState = 3; } break;
    case 3: if (now - servoHoldTimer >= holdMs) servoState = 4; break;
    case 4: moveTo(LEFT);   if (currentPos == LEFT)   { servoHoldTimer = now; servoState = 5; } break;
    case 5: if (now - servoHoldTimer >= holdMs) servoState = 6; break;
    case 6: moveTo(CENTER); if (currentPos == CENTER) { servoHoldTimer = now; servoState = 7; } break;
    case 7: if (now - servoHoldTimer >= holdMs) servoState = 0; break;
    case 8: moveTo(CENTER); if (currentPos == CENTER) { servoState = 9; } break;
    case 9: break;
  }
}

// ===== Control =====
void controlSystem(float t, float h) {
  float tempMid = (heater_off_temp + fan_temp) / 2.0f;
  heaterOn = (t < tempMid);

  float humMid = (hum_on + hum_off) / 2.0f;
  if      (h >= humMid) { fogOn = false; fanMainOn = false; }
  else if (h < hum_on)  { fogOn = true;  fanMainOn = true;  }

  fan4On = (t >= fan_temp || h > fan_hum);
  fan3On = !(t >= fan_temp && h > fan_hum);

  relayWrite(RELAY1, fogOn);   relayWrite(RELAY2, fanMainOn);
  relayWrite(RELAY3, fan3On);  relayWrite(RELAY4, fan4On);
  ssrWrite(RELAY5, heaterOn);
}

// ===== Task 2.1: โหลด stage จาก profile =====
void loadProfileStage(const String& profileName, int currentDay) {
  String basePath = fbPath("profiles/") + profileName + "/stages/";
  bool found = false;

  for (int idx = 0; idx <= 9 && !found; idx++) {
    String path = basePath + String(idx);
    if (!Firebase.RTDB.getJSON(&fbdoProf, path.c_str())) break;

    FirebaseJson& stage = fbdoProf.jsonObject();
    FirebaseJsonData r;
    int ds = 1, de = 999;
    if (stage.get(r, "dayStart") && r.success) ds = r.intValue;
    if (stage.get(r, "dayEnd")   && r.success) de = r.intValue;
    if (currentDay < ds || currentDay > de) continue;

    bool prevTurning = turningEnabled;
    if (stage.get(r, "tempMin")        && r.success) heater_off_temp = r.floatValue;
    if (stage.get(r, "tempMax")        && r.success) fan_temp        = r.floatValue;
    if (stage.get(r, "humMin")         && r.success) hum_on          = r.floatValue;
    if (stage.get(r, "humMax")         && r.success) { hum_off = r.floatValue; fan_hum = r.floatValue + 5.0f; }
    if (stage.get(r, "servoHoldHours") && r.success) servoHoldMs = (unsigned long)(r.floatValue * 3600000.0f);
    if (stage.get(r, "turning")        && r.success) turningEnabled  = r.boolValue;
    thresholdReady = true;
    found = true;

    if (prevTurning && !turningEnabled) {
      resetServo();
      sendLineAlert("เข้าสู่ระยะ Lockdown — หยุดพลิกไข่แล้ว\n");
    }
    Serial.printf("[Profile] day%d→stage%d T:%.1f-%.1f H:%.0f-%.0f Turn:%d\n",
      currentDay, idx, heater_off_temp, fan_temp, hum_on, hum_off, (int)turningEnabled);
  }

  if (!found) {
    Serial.printf("[Profile] no match day=%d, reading from control\n", currentDay);
  }
}

// ===== Task 3.2: Candling =====
void setCandlingForProfile(const String& name) {
  const int chickenDays[] = {10, 18};
  const int* days = nullptr;
  if (name == "chicken") { days = chickenDays; candlingCount = 2; }
  else                   { candlingCount = 0; }
  for (int i = 0; i < candlingCount; i++) candlingDays[i] = days[i];
  for (int i = 0; i < 5; i++) candlingAlerted[i] = false;
}

void checkCandlingAlert(int currentDay) {
  for (int i = 0; i < candlingCount; i++) {
    if (!candlingAlerted[i] && candlingDays[i] > 0 && currentDay == candlingDays[i]) {
      candlingAlerted[i] = true;
      sendLineAlert("วันส่องไข่! วันที่ " + String(candlingDays[i]) + " ของการฟัก\nอย่าลืมส่องไข่และบันทึกผล\n");
    }
  }
}

// ===== Firebase Read =====
void readControlFromFirebase() {
  if (!Firebase.ready()) return;
  if (!Firebase.RTDB.getJSON(&fbdoCtrl, fbPath("control").c_str())) return;
  FirebaseJson& json = fbdoCtrl.jsonObject();
  FirebaseJsonData r;

  if (json.get(r, "system") && r.success) systemState = r.stringValue;

  if (json.get(r,"endTime")&&r.success)        endTimeMs      = (long long)r.doubleValue;
  if (json.get(r,"startDayOffset")&&r.success) startDayOffset = r.intValue;
  if (json.get(r,"startTime")&&r.success) {
    static long long lastST = 0;
    long long st = (long long)r.doubleValue;
    if (st != lastST) {
      lastST = st; startTimeMs = st;
      alertedOneDay = alertedThirtyMin = alertedDone = false;
      lastProfileDay = -1;
      for (int i = 0; i < 5; i++) candlingAlerted[i] = false;
    }
  }

  if (json.get(r, "turning") && r.success) turningEnabled = r.boolValue;

  String newProfile = "";
  if (json.get(r, "activeProfile") && r.success) newProfile = r.stringValue;
  if (newProfile != activeProfileName) {
    activeProfileName = newProfile;
    lastProfileDay    = -1;
    if (newProfile != "" && newProfile != "custom") setCandlingForProfile(newProfile);
  }

  bool useProfile = (activeProfileName != "" && activeProfileName != "custom");

  if (useProfile && startTimeMs > 0) {
    long long nowMs = getNTPTime();
    if (nowMs > 0) {
      int currentDay = max(1, (int)((nowMs - startTimeMs) / 86400000LL) + 1 + startDayOffset);
      if (currentDay != lastProfileDay || !thresholdReady) {
        lastProfileDay = currentDay;
        loadProfileStage(activeProfileName, currentDay);
        checkCandlingAlert(currentDay);
      }
    }
  }

  if (!thresholdReady) {
    bool ok1=false, ok2=false, ok3=false, ok4=false;
    if (json.get(r,"tempMin")&&r.success&&r.floatValue>=30&&r.floatValue<=42) { heater_off_temp=r.floatValue; ok1=true; }
    if (json.get(r,"tempMax")&&r.success&&r.floatValue>=30&&r.floatValue<=42) { fan_temp=r.floatValue;        ok2=true; }
    if (json.get(r,"humMin") &&r.success&&r.floatValue>=30&&r.floatValue<=90) { hum_on=r.floatValue;          ok3=true; }
    if (json.get(r,"humMax") &&r.success&&r.floatValue>=30&&r.floatValue<=90) { hum_off=r.floatValue; fan_hum=r.floatValue+5.0f; ok4=true; }
    if (ok1&&ok2&&ok3&&ok4) {
      thresholdReady = true;
      Serial.println("[Fallback] threshold from control OK");
    }
    if (json.get(r,"servoHoldHours")&&r.success&&r.floatValue>=0.1f&&r.floatValue<=24.0f)
      servoHoldMs = (unsigned long)(r.floatValue * 3600000.0f);
  }
}

// ===== Send Current (Task 1.4: เพิ่ม deviceId, deviceName) =====
void sendCurrentData() {
  if (!Firebase.ready()) return;
  FirebaseJson json;
  json.set("temp",          latestTemp);
  json.set("humidity",      latestHum);
  json.set("systemState",   systemState);
  json.set("tempMin",       heater_off_temp);
  json.set("tempMax",       fan_temp);
  json.set("humMin",        hum_on);
  json.set("humMax",        hum_off);
  json.set("relay/fog",     fogOn     ? "ON":"OFF");
  json.set("relay/fanMain", fanMainOn ? "ON":"OFF");
  json.set("relay/fan3",    fan3On    ? "ON":"OFF");
  json.set("relay/fan4",    fan4On    ? "ON":"OFF");
  json.set("relay/heater",  heaterOn  ? "ON":"OFF");
  xSemaphoreTake(servoStatusMutex, pdMS_TO_TICKS(50));
  String _ss = servoStatus;
  xSemaphoreGive(servoStatusMutex);
  json.set("servo/status",   _ss);
  json.set("servo/position", currentPos);
  json.set("servo/holdMs",   (int)servoHoldMs);
  json.set("servo/turning",  turningEnabled);
  json.set("online",         true);
  json.set("sensorFailed",   sensorFailed);
  json.set("deviceId",       DEVICE_ID);
  json.set("deviceName",     DEVICE_NAME);
  Firebase.RTDB.setJSON(&fbdo, fbPath("current").c_str(), &json);
}

// ===== Logging =====
long long getNTPTime() {
  struct tm ti; if (!getLocalTime(&ti)) return 0;
  return (long long)mktime(&ti) * 1000LL;
}

void sendHourlyLog() {
  if (!Firebase.ready()) return;
  long long ts = getNTPTime();
  if (ts < 1577836800000LL) { Serial.println("[WARN] NTP fail"); return; }
  FirebaseJson json;
  json.set("temp",          latestTemp);
  json.set("humidity",      latestHum);
  json.set("fog",           fogOn     ? "ON":"OFF");
  json.set("fanMain",       fanMainOn ? "ON":"OFF");
  json.set("fan3",          fan3On    ? "ON":"OFF");
  json.set("fan4",          fan4On    ? "ON":"OFF");
  json.set("heater",        heaterOn  ? "ON":"OFF");
  xSemaphoreTake(servoStatusMutex, pdMS_TO_TICKS(50));
  String _ss = servoStatus;
  xSemaphoreGive(servoStatusMutex);
  json.set("servoStatus",   _ss);
  json.set("servoPosition", currentPos);
  json.set("timestamp",     ts);
  String path = fbPath("logs/") + String(ts);
  Serial.println(Firebase.RTDB.setJSON(&fbdoLog, path.c_str(), &json)
    ? "[LOG-HR] OK" : "[LOG-HR ERR] " + fbdoLog.errorReason());
}

void sendMinuteLog() {
  if (!Firebase.ready()) return;
  long long ts = getNTPTime();
  if (ts < 1577836800000LL) return;
  FirebaseJson json;
  json.set("temp",      latestTemp);
  json.set("humidity",  latestHum);
  json.set("fog",       fogOn    ? "ON":"OFF");
  json.set("heater",    heaterOn ? "ON":"OFF");
  json.set("timestamp", ts);
  Firebase.RTDB.setJSON(&fbdoLog, (fbPath("logs1m/") + String(ts)).c_str(), &json);
}

// ===== SETUP =====
void setup() {
  Serial.begin(115200);
  i2cMutex         = xSemaphoreCreateMutex();
  servoStatusMutex = xSemaphoreCreateMutex();
  Wire.begin();

  if (!sht45.begin()) Serial.println("[ERR] SHT45 not found");
  else { sht45.setPrecision(SHT4X_HIGH_PRECISION); sht45.setHeater(SHT4X_NO_HEATER); Serial.println("SHT45 OK"); }

  pinMode(RELAY1,OUTPUT); pinMode(RELAY2,OUTPUT); pinMode(RELAY3,OUTPUT);
  pinMode(RELAY4,OUTPUT); pinMode(RELAY5,OUTPUT);
  allRelaysOff();

  myServo.setPeriodHertz(50);
  myServo.attach(SERVO_PIN, 500, 2500);
  myServo.write(CENTER);

  WiFiManager wm;
  wm.setConfigPortalTimeout(180);
  wm.setConnectTimeout(30);
  if (!wm.autoConnect(("EggSetup_" + String(DEVICE_ID)).c_str())) {
    Serial.println("WiFi timeout → restart");
    ESP.restart();
  }
  Serial.println("WiFi: " + WiFi.localIP().toString());

  configTime(7*3600, 0, "pool.ntp.org", "time.nist.gov");
  Serial.print("NTP");
  struct tm ti;
  for (int i = 0; i < 20 && !getLocalTime(&ti); i++) { delay(500); Serial.print("."); }
  Serial.println(" OK");

  ArduinoOTA.setHostname("egg-incubator");
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.onStart([]() { allRelaysOff(); Serial.println("[OTA] Start"); });
  ArduinoOTA.onEnd([]()   { Serial.println("[OTA] Done"); });
  ArduinoOTA.onError([](ota_error_t e) { Serial.printf("[OTA] Error %u\n", e); });
  ArduinoOTA.begin();

  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);

  config.api_key      = API_KEY;
  config.database_url = DATABASE_URL;
  auth.user.email     = USER_EMAIL;
  auth.user.password  = USER_PASSWORD;
  config.token_status_callback = tokenStatusCallback;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  Serial.print("Firebase");
  for (int i = 0; i < 20 && !Firebase.ready(); i++) { delay(500); Serial.print("."); }
  Serial.println(Firebase.ready() ? " OK" : " TIMEOUT");

  xTaskCreatePinnedToCore(servoTask, "servoTask", 8192, NULL, 3, NULL, 0);

  logHrTimer  = millis() - LOG_HR_INTERVAL;
  logMinTimer = millis() - LOG_MIN_INTERVAL;
  Serial.printf("=== SYSTEM START === device:%s\n", DEVICE_ID);
}

// ===== LOOP =====
void loop() {
  esp_task_wdt_reset();
  ArduinoOTA.handle();
  unsigned long now = millis();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WiFi] reconnecting...");
    WiFi.reconnect();
    delay(5000);
    return;
  }

  if (now - controlTimer >= CONTROL_INTERVAL) {
    controlTimer = now;
    readControlFromFirebase();
  }

  if (lineSending && lineSendingStartMs > 0 && (now - lineSendingStartMs) > 30000UL) {
    lineSending = false;
    lineSendingStartMs = 0;
    Serial.println("[LINE] timeout reset");
  }

  static uint8_t stopCount = 0;
  if (systemState == "STOP") {
    stopCount++;
    if (stopCount >= 2 && wasRunning) {
      allRelaysOff();
      servoResetRequest = true;
      wasRunning = false;
      Serial.println("[STOP] relays off + servo reset");
      sendLineForce("ระบบตู้ฟักไข่หยุดทำงานแล้ว\n");
    }
    if (now - firebaseTimer >= FIREBASE_INTERVAL) { firebaseTimer = now; sendCurrentData(); }
    delay(50);
    return;
  }
  stopCount = 0;

  if (!wasRunning) {
    wasRunning = true;
    if (servoState == 9) servoStartRequest = true;
    Serial.println("[RUN] system started");
    sendLineForce(turningEnabled
      ? "ระบบตู้ฟักไข่เริ่มทำงานแล้ว\nServo: 90°→hold→135°→hold→90°→hold→45°→hold→90°→hold→วน\n"
      : "ระบบตู้ฟักไข่เริ่มทำงานแล้ว (Lockdown — หยุดพลิกไข่)\n");
  }

  if (now - shtTimer >= SHT_INTERVAL) {
    shtTimer = now;
    sensors_event_t hev, tev;
    bool ok = false;
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
      ok = sht45.getEvent(&hev, &tev);
      xSemaphoreGive(i2cMutex);
    }
    float t = tev.temperature, h = hev.relative_humidity;
    if (ok && !isnan(t) && !isnan(h) && t > 0 && t < 60 && h > 0 && h <= 100) {
      latestTemp = t; latestHum = h;
      shtErrorCount = 0;
      if (sensorFailed) {
        sensorFailed = false;
        sendLineForce("เซ็นเซอร์กลับมาทำงานปกติแล้ว ✅\n");
      }
      if (thresholdReady) {
        controlSystem(t, h);

        String alert = "";
        if (t > 38.0f) alert += "🌡️ อุณหภูมิสูงเกิน: " + String(t,1) + "°C\n";
        if (t < 36.0f) alert += "🌡️ อุณหภูมิต่ำเกิน: " + String(t,1) + "°C\n";
        if (!turningEnabled) {
          if (h > 71.0f) alert += "💧 ความชื้นสูงเกิน (Lockdown): " + String(h,1) + "%\n";
          if (h < 59.0f) alert += "💧 ความชื้นต่ำเกิน (Lockdown): " + String(h,1) + "%\n";
        } else {
          if (h > 61.0f) alert += "💧 ความชื้นสูงเกิน: " + String(h,1) + "%\n";
          if (h < 49.0f) alert += "💧 ความชื้นต่ำเกิน: " + String(h,1) + "%\n";
        }
        if (alert != "") sendLineAlert(alert);
      } else {
        Serial.println("[WAIT] รอค่าจาก Dashboard...");
      }
      xSemaphoreTake(servoStatusMutex, pdMS_TO_TICKS(50));
      String _ss = servoStatus;
      xSemaphoreGive(servoStatusMutex);
      Serial.printf("[OK] T:%.1f H:%.1f Fog:%s Heat:%s Servo:%s Turn:%d\n",
        t, h, fogOn?"ON":"OFF", heaterOn?"ON":"OFF", _ss.c_str(), (int)turningEnabled);
    } else {
      shtErrorCount++;
      if (shtErrorCount >= SHT_MAX_ERROR && !sensorFailed) {
        sensorFailed = true;
        allRelaysOff();
        Serial.println("[CRITICAL] Sensor fail");
        sendLineForce("เซ็นเซอร์ขัดข้อง ⚠️\nRelay ปิดทั้งหมดแล้ว\n");
      }
    }
  }

  if (now - firebaseTimer >= FIREBASE_INTERVAL) {
    firebaseTimer = now;
    sendCurrentData();
  }

  if (now - logHrTimer >= LOG_HR_INTERVAL && latestTemp > 0 && !sensorFailed) {
    logHrTimer = now;
    sendHourlyLog();
  }

  if (now - logMinTimer >= LOG_MIN_INTERVAL && latestTemp > 0 && !sensorFailed) {
    logMinTimer = now;
    sendMinuteLog();
  }

  static unsigned long incubTimer = 0;
  if (endTimeMs > 0 && now - incubTimer >= 30000UL) {
    incubTimer = now;
    long long nowMs = getNTPTime();
    if (nowMs > 0) {
      long long diff = endTimeMs - nowMs;
      if (!alertedOneDay    && diff <= 86400000LL && diff > 0) { alertedOneDay=true;    sendLineAlert("เหลืออีก 1 วัน ตู้จะหยุดทำงาน\n"); }
      if (!alertedThirtyMin && diff <= 1800000LL  && diff > 0) { alertedThirtyMin=true; sendLineAlert("เหลืออีก 30 นาที ตู้จะหยุดทำงาน\n"); }
      if (!alertedDone && diff <= 0) {
        alertedDone = true;
        systemState = "STOP";
        allRelaysOff(); resetServo();
        FirebaseJson doneJson;
        doneJson.set("system", "STOP");
        Firebase.RTDB.updateNode(&fbdo, fbPath("control").c_str(), &doneJson);
        if (startTimeMs > 0) {
          FirebaseJson runJson;
          runJson.set("status",    "completed");
          runJson.set("endTime",   (double)nowMs);
          Firebase.RTDB.updateNode(&fbdoLog,
            (fbPath("runs/") + String((long long)startTimeMs)).c_str(), &runJson);
        }
        sendLineAlert("🥚 ตู้ฟักไข่ครบกำหนดแล้ว หยุดทำงานเรียบร้อย\n");
        Serial.println("[DONE]");
      }
    }
  }

  delay(5);
}
