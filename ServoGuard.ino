// =====================================================================
// ServoGuard.ino — ดูแลเซอร์โวพลิกไข่ (module แยก — กฎเหล็กข้อ 3, ไม่แก้ handleServo/loop)
//
// 1) จำมุมเซอร์โวล่าสุดใน NVS (หน่วยความจำไม่หายตอนไฟดับ)
//    → บูตใหม่เริ่มจากมุมเดิม แล้วค่อยๆ กลับ 90° ทีละ 1° (เดิมดีดกลับ 90° ทันที ถาดสะบัด ไข่กลิ้ง)
// 2) กันเซอร์โวค้างสถานะ: ตู้ RUN แต่เซอร์โวหยุด → สั่งเริ่มใหม่ / ตู้ STOP แต่ถาดเอียงค้าง → ค่อยๆ กลับ 90°
// 3) เฝ้าถาดด้วย MPU6050 (ถ้าติดไว้ — ไม่มีก็ข้ามเอง):
//    - สั่งเอียง แต่ถาดไม่เอียง (ติดไข่ที่ตก/ติดขัด) → แจ้ง LINE + ถอยกลับ 90° แล้วลองอีกฝั่ง
//    - ถาดเอียงเกิน 70° (คว่ำ) → แจ้ง LINE ด่วน + ถอยกลับ 90° และหยุดพลิกจนกว่าถาดกลับปกติ
//    - เตือนซ้ำทุก 30 นาทีพร้อมบอกว่าค้างมานานเท่าไหร่ / แจ้งเมื่อหายแล้ว
//
// ติดตั้ง MPU6050 (GY-521): ยึดแน่นกับถาด (ไม่ใช่กับตัวตู้), ต่อ I2C ร่วมกับ SHT45
//   VCC→3.3V, GND→GND, SDA→GPIO21, SCL→GPIO22, AD0 ปล่อยลอย (address 0x68)
//   ⚠️ ครั้งแรกที่เซอร์โวค้างที่ 90° ระบบจะจำว่า "ถาดตรง" — ตอนนั้นถาดต้องอยู่ตรงจริง ไม่ติดอะไร
// =====================================================================

#define SG_SAVE_STABLE_MS   3000UL     // มุมนิ่ง 3 วิ ค่อยบันทึก (ไม่เขียน flash ทุกองศา)
#define SG_BOOT_WAIT_MS     15000UL    // หลังบูต รอรู้สถานะ RUN/STOP จาก Firebase ก่อนสั่งเซอร์โวเอง
#define SG_SETTLE_MS        8000UL     // เซอร์โวถึงมุมแล้ว รอถาดนิ่ง 8 วิ ก่อนตรวจ
#define SG_CONFIRM_MS       10000UL    // ผิดปกติต่อเนื่อง 10 วิ ถึงถือว่าค้างจริง
#define SG_TILT_MIN_DEG     8.0f       // สั่งเอียง แต่ถาดเอียงน้อยกว่านี้ = ค้าง (ปรับได้ถ้ากลไกทดรอบ)
#define SG_CENTER_MAX_DEG   15.0f      // สั่งกลับกลาง แต่ถาดยังเอียงเกินนี้ = ค้าง
#define SG_FLIP_DEG         70.0f      // ถาดเอียงเกินนี้ = คว่ำ
#define SG_REMIND_MS        1800000UL  // เตือนซ้ำทุก 30 นาที

static Preferences sgPrefs;
static int     sgSavedPos = 90;
static uint8_t sgAddr = 0;             // address MPU6050 (0 = ไม่มี)
static bool    sgHasBase = false;      // จำทิศ "ถาดตรง" แล้วหรือยัง
static float   sgBx = 0, sgBy = 0, sgBz = 1;
static volatile bool sgFlipped = false;   // ถาดคว่ำ → ห้ามสั่งเซอร์โวเริ่มใหม่

// ── เรียกใน setup() แทน myServo.write(CENTER) ──
void servoGuardBegin() {
  sgPrefs.begin("servo", false);
  int p = sgPrefs.getUChar("pos", CENTER);
  if (p < LEFT || p > RIGHT) p = CENTER;   // ค่าเพี้ยน/ยังไม่เคยบันทึก
  sgSavedPos = p;
  currentPos = p;                          // handleServo จะค่อยๆ หมุนจากมุมนี้
  myServo.write(p);
  Serial.printf("[SERVO] บูต: เริ่มที่มุมเดิม %d° (ไม่ดีดกลับ 90°)\n", p);
}

// ── MPU6050 แบบอ่าน register ตรง (ไม่ต้องลงไลบรารีเพิ่ม) — ทุกครั้งต้องถือ i2cMutex (ใช้บัสร่วม SHT45) ──
static bool sgLock()   { return xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(200)) == pdTRUE; }
static void sgUnlock() { xSemaphoreGive(i2cMutex); }

static bool sgWriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(sgAddr); Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool sgReadRaw(float& x, float& y, float& z) {
  Wire.beginTransmission(sgAddr); Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(sgAddr, (uint8_t)6) != 6) return false;
  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  x = ax / 16384.0f; y = ay / 16384.0f; z = az / 16384.0f;   // ±2g
  return true;
}

static bool sgDetect() {
  if (!sgLock()) return false;
  const uint8_t addrs[2] = {0x68, 0x69};
  for (uint8_t a : addrs) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) { sgAddr = a; break; }
  }
  // ปลุก, ช่วงวัด ±2g, low-pass 5Hz (ลดผลสั่นจากพัดลม/เซอร์โว)
  bool ok = sgAddr && sgWriteReg(0x6B, 0x00) && sgWriteReg(0x1C, 0x00) && sgWriteReg(0x1A, 0x06);
  sgUnlock();
  if (!ok) sgAddr = 0;
  return ok;
}

// อ่านเฉลี่ย 4 ครั้ง + เช็คว่าเป็นแรงโน้มถ่วงจริง (~1g) ไม่ใช่ค่าขยะ
static bool sgRead(float& x, float& y, float& z) {
  float sx = 0, sy = 0, sz = 0; int n = 0;
  for (int i = 0; i < 4; i++) {
    float a, b, c;
    if (sgLock()) {
      bool ok = sgReadRaw(a, b, c);
      sgUnlock();
      if (ok) { sx += a; sy += b; sz += c; n++; }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  if (!n) return false;
  x = sx / n; y = sy / n; z = sz / n;
  float g = sqrtf(x * x + y * y + z * z);
  return g > 0.5f && g < 1.5f;
}

// มุมระหว่างแรงโน้มถ่วงตอนนี้ กับตอน "ถาดตรง" — ไม่ต้องรู้ว่าติดเซ็นเซอร์หันทางไหน
static float sgAngleToBase(float x, float y, float z) {
  float d = (x * sgBx + y * sgBy + z * sgBz) /
            (sqrtf(x * x + y * y + z * z) * sqrtf(sgBx * sgBx + sgBy * sgBy + sgBz * sgBz));
  return acosf(constrain(d, -1.0f, 1.0f)) * 57.29578f;
}

static String sgDur(unsigned long ms) {
  unsigned long m = ms / 60000UL;
  return m >= 60 ? String(m / 60) + " ชม. " + String(m % 60) + " นาที" : String(m) + " นาที";
}

void servoGuardTask(void*) {
  vTaskDelay(pdMS_TO_TICKS(2000));
  bool hasMpu = sgDetect();
  if (hasMpu) Serial.printf("[TRAY] พบ MPU6050 ที่ 0x%02X — เปิดระบบเฝ้าถาด\n", sgAddr);
  else        Serial.println("[TRAY] ไม่พบ MPU6050 — ข้ามระบบเฝ้าถาด (จำมุม/กันดีด ยังทำงาน)");

  unsigned long bootMs = millis();
  int lastPos = currentPos;   unsigned long posSince = millis();
  int lastState = -1;         unsigned long stateSince = millis();
  int readFails = 0;          bool mpuLost = false;

  enum { F_NONE, F_TILT, F_CENTER } fault = F_NONE;   // F_TILT = สั่งเอียงแต่ไม่เอียง, F_CENTER = สั่งกลับกลางแต่ไม่กลับ
  unsigned long badSince = 0, faultSince = 0, lastRemind = 0;
  int faultPos = 0;   // มุมที่พลิกไม่ได้ (135 หรือ 45) — หายเมื่อฝั่งนี้พลิกผ่าน ไม่ใช่อีกฝั่ง
  unsigned long flipBad = 0, flipGood = 0, flipSince = 0, flipRemind = 0;

  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(500));
    unsigned long now = millis();
    int pos = currentPos, st = servoState;

    // ── 1) จำมุมล่าสุด ──
    if (pos != lastPos) { lastPos = pos; posSince = now; }
    else if (pos != sgSavedPos && now - posSince >= SG_SAVE_STABLE_MS) {
      sgPrefs.putUChar("pos", (uint8_t)pos);
      sgSavedPos = pos;
    }

    // ── 2) กันเซอร์โวค้างสถานะ ──
    if (now - bootMs >= SG_BOOT_WAIT_MS && !servoStartRequest && !servoResetRequest) {
      // RUN + เปิดพลิกไข่ แต่เซอร์โวหยุดนิ่ง (เช่น กด START ตอนเซอร์โวกำลังถอยกลับ 90° → คำสั่งเริ่มหาย)
      if (wasRunning && turningEnabled && st == 9 && !sgFlipped) servoStartRequest = true;
      // STOP แต่ถาดยังเอียงค้าง (บูตตอน STOP จากมุมเดิม) → ค่อยๆ กลับ 90°
      else if (!wasRunning && st == 9 && pos != CENTER) servoResetRequest = true;
    }

    // ── 3) เฝ้าถาดด้วย MPU6050 ──
    if (!hasMpu) { trayAngle = -1; continue; }
    if (st != lastState) { lastState = st; stateSince = now; badSince = 0; }

    float x, y, z;
    if (!sgRead(x, y, z)) {
      if (++readFails >= 20 && !mpuLost) {   // ~10 วิ อ่านไม่ได้
        mpuLost = true; trayAngle = -1;
        queueLine("📐 เซ็นเซอร์มุมถาด (MPU6050) อ่านค่าไม่ได้\nตรวจสอบสาย/ขั้วต่อที่ถาด — ระบบเฝ้าถาดหยุดชั่วคราว\n", true);
      }
      continue;
    }
    readFails = 0;
    if (mpuLost) { mpuLost = false; queueLine("✅ เซ็นเซอร์มุมถาดกลับมาทำงานแล้ว\n", true); }

    bool holding  = (st == 1 || st == 3 || st == 5 || st == 7 || st == 9);
    bool settled  = holding && now - stateSince >= SG_SETTLE_MS;
    bool atCenter = (pos == CENTER);

    // จำทิศ "ถาดตรง" ครั้งแรกที่เซอร์โวค้างที่ 90° และนิ่ง
    if (!sgHasBase) {
      if (settled && atCenter) {
        sgBx = x; sgBy = y; sgBz = z; sgHasBase = true;
        Serial.println("[TRAY] บันทึกตำแหน่งถาดตรงแล้ว — เริ่มเฝ้าถาด");
      }
      trayAngle = 0;
      continue;
    }

    float ang = sgAngleToBase(x, y, z);
    trayAngle = ang;

    // ปรับทิศ "ถาดตรง" ช้าๆ ตามการทรุดตัวของตู้ — เฉพาะตอนปกติและอยู่กลางจริง
    if (settled && atCenter && fault == F_NONE && !sgFlipped && ang < 5.0f) {
      sgBx = sgBx * 0.98f + x * 0.02f; sgBy = sgBy * 0.98f + y * 0.02f; sgBz = sgBz * 0.98f + z * 0.02f;
    }

    // ── 3a) ถาดคว่ำ — ตรวจตลอดเวลา (แม้ตู้ STOP) ──
    if (ang > SG_FLIP_DEG) {
      flipGood = 0;
      if (!flipBad) flipBad = now;
      if (!sgFlipped && now - flipBad >= 3000) {
        sgFlipped = true; flipSince = flipBad; flipRemind = now;
        if (pos != CENTER || st < 8) servoResetRequest = true;   // ถอยกลับ 90° แล้วหยุด ไม่ดันต่อ
        queueLine("🚨 ถาดไข่อาจคว่ำ!\nมุมถาด " + String(ang, 0) + "° (ปกติไม่เกิน ~45°)\n"
                  "เซอร์โวถอยกลับ 90° และหยุดพลิกแล้ว — กรุณาตรวจสอบตู้ด่วน\n", true);
      } else if (sgFlipped && now - flipRemind >= SG_REMIND_MS) {
        flipRemind = now;
        queueLine("🚨 ถาดไข่ยังคว่ำอยู่: มุมถาด " + String(ang, 0) + "°\nคว่ำมาแล้ว " + sgDur(now - flipSince) +
                  " — ไข่ไม่ได้พลิกตลอดช่วงนี้\n", true);
      }
    } else {
      flipBad = 0;
      if (sgFlipped) {
        if (!flipGood) flipGood = now;
        if (ang < 30.0f && now - flipGood >= 10000) {
          sgFlipped = false;   // ส่วนที่ 2 จะสั่งเซอร์โวเริ่มพลิกใหม่เอง
          queueLine("✅ ถาดไข่กลับตำแหน่งปกติแล้ว (คว่ำนาน " + sgDur(now - flipSince) + ")\nเริ่มพลิกไข่ต่อ\n", true);
        }
      }
    }

    // ── 3b) ถาดไม่หมุนตามเซอร์โว — เฉพาะตอนตู้ RUN ──
    if (!wasRunning) { fault = F_NONE; badSince = 0; trayStuckSec = 0; continue; }
    if (fault == F_TILT && !turningEnabled) fault = F_NONE;   // เข้า Lockdown เลิกพลิก → ไม่ต้องเตือนเรื่องพลิกไม่ได้

    if (settled && !sgFlipped) {
      bool tiltFail   = !atCenter && ang < SG_TILT_MIN_DEG;
      bool centerFail =  atCenter && ang > SG_CENTER_MAX_DEG;
      if (tiltFail || centerFail) {
        if (!badSince) badSince = now;
        if (now - badSince >= SG_CONFIRM_MS) {
          if (fault == F_NONE) {
            fault = tiltFail ? F_TILT : F_CENTER; faultSince = badSince; lastRemind = now; faultPos = pos;
            queueLine(tiltFail
              ? "⚠️ ถาดไข่ไม่หมุนตามเซอร์โว — อาจติดไข่ที่ตก หรือถาดคว่ำ\nสั่งไป " + String(pos) + "° แต่ถาดเอียงแค่ " +
                String(ang, 0) + "°\nเซอร์โวถอยกลับ 90° (กันดันไข่/มอเตอร์ร้อน) แล้วจะลองพลิกอีกฝั่ง\n"
              : "⚠️ ถาดไข่ไม่กลับตำแหน่งกลาง — อาจติดไข่ หรือถาดคว่ำ\nเซอร์โวอยู่ 90° แต่ถาดยังเอียง " +
                String(ang, 0) + "°\n", true);
          }
          // ถอยกลับ 90° แล้วไปทำขั้นถัดไป (ข้ามเวลาค้างที่เหลือ) — ไม่แก้โค้ด handleServo แค่เลื่อนสถานะ
          if (tiltFail && st == 1) servoState = 2;        // ค้างที่ 135° → กลับ 90° → ต่อด้วยฝั่ง 45°
          else if (tiltFail && st == 5) servoState = 6;   // ค้างที่ 45° → กลับ 90° → ต่อด้วยฝั่ง 135°
        }
      } else {
        badSince = 0;
        // หายจริงเมื่อผ่าน "จุดที่เคยล้มเหลว": F_TILT ต้องพลิกฝั่งเดิมที่ค้างได้ (อีกฝั่งผ่านไม่นับ — ไข่อาจขวางฝั่งเดียว)
        //                                  F_CENTER ต้องเห็นถาดกลับกลางได้
        if ((fault == F_TILT && pos == faultPos) || (fault == F_CENTER && atCenter)) {
          queueLine("✅ ถาดไข่กลับมาหมุนตามเซอร์โวปกติแล้ว\nค้างไปนาน " + sgDur(now - faultSince) + "\n", true);
          fault = F_NONE;
        }
      }
    }

    if (fault != F_NONE) {
      trayStuckSec = (now - faultSince) / 1000;
      if (now - lastRemind >= SG_REMIND_MS) {
        lastRemind = now;
        queueLine("⚠️ ถาดไข่ยังค้างอยู่ — ค้างมาแล้ว " + sgDur(now - faultSince) +
                  "\nมุมถาดตอนนี้ " + String(ang, 0) + "° · กรุณาตรวจสอบไข่ที่ตก/ถาด\n", true);
      }
    } else {
      trayStuckSec = 0;
    }
  }
}

void servoGuardStart() {
  xTaskCreatePinnedToCore(servoGuardTask, "servoGuard", 4096, NULL, 1, NULL, 0);
}
