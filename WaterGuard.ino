// =====================================================================
// WaterGuard.ino — เปิด/ปิดระบบน้ำทั้งชุดจากหน้าเว็บ (module แยก — กฎเหล็กข้อ 3)
//
// ESP32 ตรวจไม่ได้ว่าต่อปั๊มอยู่ไหม (ปั๊มอยู่หลังหน้าสัมผัสรีเลย์) → ใช้สวิตช์ control/waterEnabled แทน
// ปิด (ค่าเริ่มต้น): ปั๊มไม่ทำงานเลย, ไม่แจ้ง LINE เรื่องถังหมอก/ถังสำรอง/ปั๊ม, ไม่ส่งค่ามั่วขึ้นเว็บ
// เปิด: ทำงานเหมือนเดิมทุกอย่าง (เติมน้ำอัตโนมัติ + แจ้งเตือน)
// =====================================================================

void waterSetEnabled(bool en) {
  if (en == waterEnabled) return;
  waterEnabled = en;
  if (!en) {
    setPump(false);   // ปิดปั๊มทันที (setPump บังคับ off อยู่แล้วเมื่อระบบปิด)
  } else {
    // ช่วงที่ปิด loop() อาจตั้ง flag "แจ้งแล้ว" จากค่ามั่วของขาลอย → ล้าง ให้แจ้งเตือนจริงได้ทันทีที่เปิด
    fogTankAlerted = false; waterAlerted = false; pumpOverrunAlerted = false;
  }
  Serial.printf("[WATER] ระบบน้ำ %s\n", en ? "เปิดใช้งาน" : "ปิดใช้งาน");
}

// ข้อความเรื่องน้ำจาก loop() — ทิ้งเมื่อระบบน้ำปิด
bool waterAlertIsMuted(const String& m) {
  if (waterEnabled) return false;
  return m.startsWith("💧 ถังสำรองน้ำน้อย") ||
          m.startsWith("🪣 ถังพ่นหมอกน้ำน้อย") ||
          m.startsWith("⚠️ ปั๊มน้ำทำงานนานผิดปกติ");
}
