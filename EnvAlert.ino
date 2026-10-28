// =====================================================================
// EnvAlert.ino — แจ้งเตือนอุณหภูมิ/ความชื้น "ตามช่วงของโปรไฟล์" (module แยก — กฎเหล็กข้อ 3)
//
// เดิม: loop() แจ้งด้วยเลขตายตัว 38/36°C, 61/49% (Lockdown 71/59%) ไม่สนโปรไฟล์
// ใหม่: ใช้ช่วงของ stage ปัจจุบัน (tempMin–tempMax, humMin–humMax) ± ระยะเผื่อ
//       ข้อความแบบเดิมจาก loop() ถูกกรองทิ้งใน sendLineAlert() → ไม่ต้องแก้ loop()
//
// ตัวอย่าง ไก่ระยะต้น (36.5–37.5°C, 55–56%):  แจ้งเมื่อ < 36.0 / > 38.0°C,  < 50 / > 61%
//          ไก่ Lockdown (36.8–37.0°C, 69–70%): แจ้งเมื่อ < 36.3 / > 37.5°C,  < 64 / > 75%
// =====================================================================

#define ENV_TEMP_MARGIN   0.5f        // °C เผื่อจากขอบช่วง
#define ENV_HUM_MARGIN    5.0f        // % เผื่อจากขอบช่วง
#define ENV_TEMP_CRIT     1.5f        // °C เกินขอบช่วงเท่านี้ = วิกฤต แจ้งทันทีไม่ต้องรอยืนยัน
#define ENV_CONFIRM_MS    60000UL     // ต้องผิดปกติต่อเนื่อง 1 นาที (กันเปิดประตูตู้แป๊บเดียวแล้วแจ้ง)
#define ENV_RECOVER_MS    60000UL     // ต้องกลับปกติต่อเนื่อง 1 นาที ถึงแจ้งว่าหายแล้ว
#define ENV_REMIND_MS     600000UL    // ยังผิดปกติอยู่ → เตือนซ้ำครั้งแรกหลัง 10 นาที พร้อมบอกว่านานเท่าไหร่
#define ENV_REMIND_MAX_MS 3600000UL   // แล้วห่างขึ้นเรื่อยๆ 10→20→40→60 นาที (ค้างที่ทุก 1 ชม.) กันแจ้งรัว

// ข้อความเลขตายตัวจาก loop() — sendLineAlert() ทิ้งข้อความพวกนี้
bool envAlertIsLegacy(const String& m) {
  return m.startsWith("🌡️ อุณหภูมิสูงเกิน") ||
         m.startsWith("🌡️ อุณหภูมิต่ำเกิน") ||
         m.startsWith("💧 ความชื้นสูงเกิน") ||
         m.startsWith("💧 ความชื้นต่ำเกิน");
}

static String envDur(unsigned long ms) {
  unsigned long m = ms / 60000UL;
  return m >= 60 ? String(m / 60) + " ชม. " + String(m % 60) + " นาที" : String(m) + " นาที";
}

// หมายเหตุ: step() เป็น method ใน struct โดยตั้งใจ — Arduino IDE สร้าง prototype ของฟังก์ชันธรรมดาไว้บนสุดไฟล์
// ก่อน struct ถูกประกาศ → ถ้าเป็นฟังก์ชันที่รับ EnvCheck& จะ compile ไม่ผ่าน ("EnvCheck was not declared")
struct EnvCheck {
  unsigned long badSince = 0, goodSince = 0, lastSent = 0, remindGap = ENV_REMIND_MS;
  bool alerted = false;

  // bad = ค่าเกินช่วงตอนนี้, crit = เกินมากจนต้องแจ้งทันที
  void step(bool bad, bool crit, unsigned long now, const String& title, const String& detail) {
    if (bad) {
      goodSince = 0;
      if (!badSince) badSince = now;
      if (!alerted && (crit || now - badSince >= ENV_CONFIRM_MS)) {
        alerted = true; lastSent = now;
        queueLine(title + ": " + detail + "\n", true);
      } else if (alerted && now - lastSent >= remindGap) {
        lastSent = now;
        remindGap = (remindGap * 2 < ENV_REMIND_MAX_MS) ? remindGap * 2 : ENV_REMIND_MAX_MS;
        queueLine(title + " (ยังไม่หาย): " + detail + "\nผิดปกติมาแล้ว " + envDur(now - badSince) + "\n", true);
      }
    } else if (badSince) {
      if (!goodSince) goodSince = now;
      if (now - goodSince >= ENV_RECOVER_MS) {
        if (alerted)
          queueLine("✅ " + title + " — กลับสู่ช่วงปกติแล้ว: " + detail + "\nผิดปกตินาน " + envDur(goodSince - badSince) + "\n", true);
        *this = EnvCheck();
      }
    }
  }
};
static EnvCheck envT_hi, envT_lo, envH_hi, envH_lo;

void envAlertTask(void*) {
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    unsigned long now = millis();
    // ทำงานเฉพาะตอนตู้ RUN + ได้ค่าโปรไฟล์แล้ว + เซ็นเซอร์ปกติ (เซ็นเซอร์เสียมีแจ้งเตือนของตัวเองอยู่แล้ว)
    float tMin = heater_off_temp, tMax = fan_temp, hMin = hum_on, hMax = hum_off;
    float t = latestTemp, h = latestHum;
    bool active = wasRunning && thresholdReady && !sensorFailed && t > 0 && tMax > tMin && hMax >= hMin;
    if (!active) { envT_hi = envT_lo = envH_hi = envH_lo = EnvCheck(); continue; }

    String tRange = "เป้า " + String(tMin, 1) + "–" + String(tMax, 1) + "°C";
    String hRange = "เป้า " + String(hMin, 0) + "–" + String(hMax, 0) + "%";
    String tNow = String(t, 1) + "°C (" + tRange + ")";
    String hNow = String(h, 1) + "% (" + hRange + ")";

    envT_hi.step(t > tMax + ENV_TEMP_MARGIN, t > tMax + ENV_TEMP_CRIT, now, "🌡️ อุณหภูมิสูงกว่าช่วงโปรไฟล์", tNow);
    envT_lo.step(t < tMin - ENV_TEMP_MARGIN, t < tMin - ENV_TEMP_CRIT, now, "🌡️ อุณหภูมิต่ำกว่าช่วงโปรไฟล์", tNow);
    envH_hi.step(h > hMax + ENV_HUM_MARGIN,  false,                    now, "💧 ความชื้นสูงกว่าช่วงโปรไฟล์", hNow);
    envH_lo.step(h < hMin - ENV_HUM_MARGIN,  false,                    now, "💧 ความชื้นต่ำกว่าช่วงโปรไฟล์", hNow);
  }
}

void envAlertStart() {
  xTaskCreatePinnedToCore(envAlertTask, "envAlert", 4096, NULL, 1, NULL, 0);
}
