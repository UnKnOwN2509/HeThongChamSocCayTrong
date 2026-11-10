/*
 * ====================================================================================
 * ĐỒ ÁN: HỆ THỐNG IOT GIÁM SÁT VÀ CHĂM SÓC CÂY TRỒNG NĂNG LƯỢNG MẶT TRỜI
 * VI ĐIỀU KHIỂN: ESP32-S3 Dev Module
 * ====================================================================================
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_SHT31.h>
#include <BH1750.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>

// ==========================================
// 1. CẤU HÌNH CHÂN (PINOUT) TRÊN ESP32-S3
// ==========================================
#define PIN_I2C_SDA         8     // Bus I2C Data
#define PIN_I2C_SCL         9     // Bus I2C Clock

#define PIN_SOIL_1          1     // ADC1_CH0: Cảm biến độ ẩm đất điện dung v1.2
#define PIN_BATTERY_ADC     3     // ADC1_CH2: Đo điện áp Pin qua cầu trở 100k-100k
#define PIN_FLOW_SENSOR     4     // Chân ngắt ngoài đếm xung YF-S401

#define PIN_RELAY_PUMP      5     // Relay 1: Máy bơm nước 5V
#define PIN_RELAY_FAN       6     // Relay 2: Quạt mini 5V
#define PIN_RELAY_LIGHT     7     // Relay 3: Đèn LED quang hợp 5V

#define RELAY_ON            LOW   // Module Relay kích mức LOW
#define RELAY_OFF           HIGH

// ==========================================
// 2. THÔNG SỐ HIỆU CHUẨN CẢM BIẾN
// ==========================================
#define SOIL_AIR_VALUE      3200  // Giá trị ADC khi cảm biến khô (0%)
#define SOIL_WATER_VALUE    1500  // Giá trị ADC khi nhúng nước (100%)

// YF-S401: ~5880 xung/lít = 5.88 xung/ml (Nếu dùng YF-S201 đổi thành 0.45)
#define FLOW_CALIBRATION_FACTOR 5.88  

#define BATTERY_DIVIDER_RATIO   2.0   // Tỉ lệ phân áp 100k / 100k
#define BATTERY_MIN_V           3.3   // Pin 0%
#define BATTERY_MAX_V           4.2   // Pin 100%
#define BATTERY_LOW_THRESH      20.0  // Khóa bơm nếu pin < 20% để chống sụt áp

// Ngưỡng cài đặt ban đầu
float soilThresholdMin    = 40.0;     // Độ ẩm đất < 40% thì kích hoạt tưới
float targetWaterVolumeMl = 150.0;    // Thể tích nước mục tiêu (ml)
float tempThresholdMax    = 32.0;     // Nhiệt độ > 32°C bật quạt
float lightThresholdMin   = 300.0;    // Ánh sáng < 300 Lux bật đèn
unsigned long pumpTimeoutMs = 45000;  // 45s: Hạn giờ chống kẹt bơm/cạn nước

// ==========================================
// 3. KHỞI TẠO ĐỐI TƯỢNG NGOẠI VI
// ==========================================
LiquidCrystal_I2C lcd(0x27, 16, 2);   // Màn hình LCD 16x2
Adafruit_SHT31 sht31 = Adafruit_SHT31();
BH1750 lightMeter;
WebServer server(80);
Preferences prefs;

// Biến trạng thái
volatile unsigned long pulseCount = 0;
float currentWaterDispensed = 0.0;
float totalWaterUsedToday = 0.0;

float envTemp = 0.0;
float envHumidity = 0.0;
float envLightLux = 0.0;
float soilMoisture1 = 0.0;
float batteryVoltage = 0.0;
float batteryPercent = 0.0;

bool isPumping = false;
bool isFanOn = false;
bool isLightOn = false;
bool pumpErrorWaterEmpty = false;
unsigned long pumpStartTime = 0;
unsigned long lastSensorReadTime = 0;
unsigned long lastLcdUpdateTime = 0;
int lcdPage = 0;

const char* ap_ssid = "ESP32S3_SmartFarm";
const char* ap_pass = "12345678";

// ==========================================
// 4. HÀM NGẮT (ISR) ĐẾM XUNG LƯU LƯỢNG
// ==========================================
void IRAM_ATTR flowSensorPulseISR() {
  pulseCount++;
}

// ==========================================
// 5. ĐỌC CẢM BIẾN
// ==========================================
void readSensors() {
  // SHT30
  float t = sht31.readTemperature();
  float h = sht31.readHumidity();
  if (!isnan(t)) envTemp = t;
  if (!isnan(h)) envHumidity = h;

  // BH1750
  float lux = lightMeter.readLightLevel();
  if (lux >= 0) envLightLux = lux;

  // Độ ẩm đất (Lấy trung bình 10 mẫu lọc nhiễu)
  long adcSoil = 0;
  for (int i = 0; i < 10; i++) {
    adcSoil += analogRead(PIN_SOIL_1);
    delay(2);
  }
  adcSoil /= 10;
  soilMoisture1 = map(adcSoil, SOIL_AIR_VALUE, SOIL_WATER_VALUE, 0, 100);
  soilMoisture1 = constrain(soilMoisture1, 0.0, 100.0);

  // Pin 18650
  long adcBat = 0;
  for (int i = 0; i < 10; i++) {
    adcBat += analogRead(PIN_BATTERY_ADC);
    delay(2);
  }
  adcBat /= 10;
  float vAdc = (adcBat / 4095.0) * 3.3;
  batteryVoltage = vAdc * BATTERY_DIVIDER_RATIO;
  batteryPercent = ((batteryVoltage - BATTERY_MIN_V) / (BATTERY_MAX_V - BATTERY_MIN_V)) * 100.0;
  batteryPercent = constrain(batteryPercent, 0.0, 100.0);
}

// ==========================================
// 6. THUẬT TOÁN TƯỚI NƯỚC THEO THỂ TÍCH & TIMEOUT
// ==========================================
void startWatering() {
  if (batteryPercent < BATTERY_LOW_THRESH) {
    Serial.println("[CẢNH BÁO] Pin yếu! Không bật bơm để chống sụt áp vi điều khiển.");
    return;
  }
  isPumping = true;
  pumpStartTime = millis();
  pulseCount = 0;
  currentWaterDispensed = 0.0;
  pumpErrorWaterEmpty = false;
  digitalWrite(PIN_RELAY_PUMP, RELAY_ON);
  Serial.println("[BƠM] Bắt đầu kích hoạt máy bơm...");
}

void stopWatering() {
  isPumping = false;
  digitalWrite(PIN_RELAY_PUMP, RELAY_OFF);
  totalWaterUsedToday += currentWaterDispensed;
  prefs.putFloat("water_total", totalWaterUsedToday);
  Serial.printf("[BƠM] Đã dừng. Đợt này: %.1f ml | Tổng: %.1f ml\n", currentWaterDispensed, totalWaterUsedToday);
}

void handleWateringProcess() {
  if (!isPumping) return;

  currentWaterDispensed = pulseCount / FLOW_CALIBRATION_FACTOR;

  // Đủ lượng nước mục tiêu
  if (currentWaterDispensed >= targetWaterVolumeMl) {
    Serial.println("[BƠM] Đã đạt đủ lượng nước yêu cầu!");
    stopWatering();
    return;
  }

  // Safety Timeout: Quá thời gian mà xung không nhảy -> Cạn nước hoặc kẹt bơm
  unsigned long runningTime = millis() - pumpStartTime;
  if (runningTime > pumpTimeoutMs) {
    if (currentWaterDispensed < 10.0) {
      pumpErrorWaterEmpty = true;
      Serial.println("[LỖI NGUY HIỂM] Hết nước bình chứa hoặc kẹt cánh quạt bơm!");
    }
    stopWatering();
  }
}

// ==========================================
// 7. ĐIỀU KHIỂN TỰ ĐỘNG QUẠT & ĐÈN
// ==========================================
void handleActuatorsAuto() {
  // Quạt tản nhiệt
  if (envTemp >= tempThresholdMax) {
    if (!isFanOn) {
      isFanOn = true;
      digitalWrite(PIN_RELAY_FAN, RELAY_ON);
    }
  } else if (envTemp < (tempThresholdMax - 1.5)) {
    if (isFanOn) {
      isFanOn = false;
      digitalWrite(PIN_RELAY_FAN, RELAY_OFF);
    }
  }

  // Đèn quang hợp
  if (envLightLux < lightThresholdMin) {
    if (!isLightOn) {
      isLightOn = true;
      digitalWrite(PIN_RELAY_LIGHT, RELAY_ON);
    }
  } else if (envLightLux > (lightThresholdMin + 100)) {
    if (isLightOn) {
      isLightOn = false;
      digitalWrite(PIN_RELAY_LIGHT, RELAY_OFF);
    }
  }

  // Tưới tự động khi đất khô
  if (!isPumping && !pumpErrorWaterEmpty && soilMoisture1 < soilThresholdMin) {
    startWatering();
  }
}

// ==========================================
// 8. HIỂN THỊ LCD 16x2 (4 TRANG LUÂN PHIÊN)
// ==========================================
void updateLcdDisplay() {
  lcd.clear();
  switch (lcdPage) {
    case 0:
      lcd.setCursor(0, 0);
      lcd.print("T:"); lcd.print(envTemp, 1); lcd.print((char)223); lcd.print("C H:"); lcd.print(envHumidity, 0); lcd.print("%");
      lcd.setCursor(0, 1);
      lcd.print("Lux:"); lcd.print((int)envLightLux); lcd.print(" lx");
      break;
    case 1:
      lcd.setCursor(0, 0);
      lcd.print("Dat:"); lcd.print(soilMoisture1, 0); lcd.print("% Set:"); lcd.print(soilThresholdMin, 0); lcd.print("%");
      lcd.setCursor(0, 1);
      lcd.print("Pin:"); lcd.print(batteryVoltage, 2); lcd.print("V ("); lcd.print((int)batteryPercent); lcd.print("%)");
      break;
    case 2:
      lcd.setCursor(0, 0);
      lcd.print(isPumping ? "BOM DANG TUOI..." : (pumpErrorWaterEmpty ? "CAN NUOC/KET BOM" : "BOM: DANG NGHI"));
      lcd.setCursor(0, 1);
      lcd.print("Nuoc:"); lcd.print((int)currentWaterDispensed); lcd.print("/"); lcd.print((int)targetWaterVolumeMl); lcd.print("ml");
      break;
    case 3:
      lcd.setCursor(0, 0);
      lcd.print("Quat:"); lcd.print(isFanOn ? "ON " : "OFF ");
      lcd.print("Den:"); lcd.print(isLightOn ? "ON" : "OFF");
      lcd.setCursor(0, 1);
      lcd.print("Tong:"); lcd.print((int)totalWaterUsedToday); lcd.print(" ml");
      break;
  }
  lcdPage = (lcdPage + 1) % 4;
}

// ==========================================
// 9. GIAO DIỆN WEB SERVER TRỰC QUAN
// ==========================================
void handleRoot() {
  String html = "<!DOCTYPE html><html lang='vi'><head><meta charset='UTF-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<title>ESP32-S3 Smart Farm</title>";
  html += "<style>body{font-family:sans-serif;background:#0f172a;color:#f8fafc;padding:16px;}";
  html += ".box{max-width:500px;margin:auto;background:#1e293b;border-radius:12px;padding:20px;}";
  html += "h2{color:#38bdf8;text-align:center;}";
  html += ".grid{display:grid;grid-template-columns:1fr 1fr;gap:10px;}";
  html += ".item{background:#334155;border-radius:8px;padding:10px;text-align:center;}";
  html += ".item h4{margin:0;color:#94a3b8;font-size:12px;} .item p{margin:5px 0 0;font-size:18px;font-weight:bold;}";
  html += ".btn{display:block;width:100%;padding:12px;margin-top:12px;background:#0284c7;color:#fff;border:none;border-radius:8px;font-size:16px;font-weight:bold;cursor:pointer;}";
  html += ".btn-danger{background:#e11d48;}";
  html += ".warn{background:#dc2626;color:#fff;padding:10px;border-radius:6px;text-align:center;font-weight:bold;margin-bottom:10px;}";
  html += "</style><meta http-equiv='refresh' content='4'></head><body>";
  html += "<div class='box'><h2>ESP32-S3 SMART FARM</h2>";
  if (pumpErrorWaterEmpty) html += "<div class='warn'>CẢNH BÁO: CẠN NƯỚC / KẸT BƠM!</div>";
  html += "<div class='grid'>";
  html += "<div class='item><h4>NHIỆT ĐỘ</h4><p>" + String(envTemp, 1) + " °C</p></div>";
  html += "<div class='item'><h4>ĐỘ ẨM KHÍ</h4><p>" + String(envHumidity, 1) + " %</p></div>";
  html += "<div class='item'><h4>ÁNH SÁNG</h4><p>" + String((int)envLightLux) + " Lux</p></div>";
  html += "<div class='item'><h4>ĐỘ ẨM ĐẤT</h4><p>" + String(soilMoisture1, 1) + " %</p></div>";
  html += "<div class='item'><h4>PIN 18650</h4><p>" + String(batteryVoltage, 2) + "V (" + String((int)batteryPercent) + "%)</p></div>";
  html += "<div class='item'><h4>TỔNG NƯỚC</h4><p>" + String((int)totalWaterUsedToday) + " ml</p></div>";
  html += "</div>";
  html += "<p style='margin-top:14px;'>Bơm: <b>" + String(isPumping ? "<span style='color:#4ade80;'>ĐANG TƯỚI</span>" : "Nghỉ") + "</b> | Quạt: <b>" + (isFanOn ? "BẬT" : "TẮT") + "</b> | Đèn: <b>" + (isLightOn ? "BẬT" : "TẮT") + "</b></p>";
  if (!isPumping) html += "<a href='/water'><button class='btn'>TƯỚI NƯỚC THỦ CÔNG</button></a>";
  else html += "<a href='/stop'><button class='btn btn-danger'>DỪNG BƠM KHẨN CẤP</button></a>";
  html += "</div></body></html>";
  server.send(200, "text/html", html);
}

// ==========================================
// 10. SETUP & LOOP
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(PIN_RELAY_PUMP, OUTPUT);
  pinMode(PIN_RELAY_FAN, OUTPUT);
  pinMode(PIN_RELAY_LIGHT, OUTPUT);
  digitalWrite(PIN_RELAY_PUMP, RELAY_OFF);
  digitalWrite(PIN_RELAY_FAN, RELAY_OFF);
  digitalWrite(PIN_RELAY_LIGHT, RELAY_OFF);

  pinMode(PIN_FLOW_SENSOR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_FLOW_SENSOR), flowSensorPulseISR, RISING);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0); lcd.print("ESP32-S3 IoT");
  lcd.setCursor(0, 1); lcd.print("Smart Farm v1.0");

  sht31.begin(0x44);
  lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);

  prefs.begin("plant_iot", false);
  totalWaterUsedToday = prefs.getFloat("water_total", 0.0);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ap_ssid, ap_pass);
  Serial.print("[WIFI AP] SSID: "); Serial.println(ap_ssid);
  Serial.print("[WIFI IP] http://"); Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/water", []() { startWatering(); server.sendHeader("Location", "/"); server.send(303); });
  server.on("/stop", []() { stopWatering(); server.sendHeader("Location", "/"); server.send(303); });
  server.begin();
  delay(1500);
}

void loop() {
  server.handleClient();
  handleWateringProcess();

  unsigned long now = millis();
  if (now - lastSensorReadTime >= 1500) {
    lastSensorReadTime = now;
    readSensors();
    handleActuatorsAuto();
    Serial.printf("[LOG] T:%.1f°C | H:%.1f%% | Lux:%.0f | Đất:%.1f%% | Pin:%.2fV | Nước:%.0fml\n",
                  envTemp, envHumidity, envLightLux, soilMoisture1, batteryVoltage, currentWaterDispensed);
  }

  if (now - lastLcdUpdateTime >= 2500) {
    lastLcdUpdateTime = now;
    updateLcdDisplay();
  }
}