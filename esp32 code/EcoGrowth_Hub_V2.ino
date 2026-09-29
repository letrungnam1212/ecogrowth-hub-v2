#include <WiFi.h>
#include <Wire.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <BH1750.h>
#include <FirebaseESP32.h>

#include "config.h"
#include "pid_controller.h"

// =================== ĐỐI TƯỢNG VÀ BIẾN TOÀN CỤC ===================
Adafruit_BME280 bme;
BH1750 lightMeter;
AdaptivePID pid(2.0, 0.5, 0.1);
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig fbConfig;

SystemState currentState = STATE_IDLE;

// Biến cảm biến & quá trình
float currentSoilMoisture = 0.0;
float currentTemp = 0.0;
float currentHum = 0.0;
float currentLux = 0.0;
float currentPressure = 1013.2;
float pressureBuffer[10];
int pressureIndex = 0;

// Biến cấu hình
float targetSetpoint = 60.0;
String currentMode = "AUTO";
bool manualTriggerFlag = false;
float rainThresholdHpa = 1.5;
String userChatId = TG_DEFAULT_CHAT_ID;
bool isRainingExpected = false;

// Biến điều khiển chu kỳ tưới
volatile unsigned long pulseCount = 0;
float mlAccumulated = 0.0;
float moistureBeforeCycle = 0.0;
unsigned long manualStartTime = 0;

// Timer non-blocking millis()
unsigned long lastSensorTick = 0;
unsigned long lastWeatherTick = 0;
unsigned long lastSmartAdvisorTick = 0;

// Ngắt Flow Sensor đếm xung
void IRAM_ATTR flowPulseCounter() {
  pulseCount++;
}

// =================== CÁC HÀM HỖ TRỢ (HELPERS) ===================

void sendTelegramAlert(String message) {
  if (WiFi.status() == WL_CONNECTED && userChatId.length() > 0) {
    HTTPClient http;
    String url = "https://api.telegram.org/bot" + String(TG_BOT_TOKEN) + "/sendMessage?chat_id=" + userChatId + "&text=" + message;
    http.begin(url);
    http.GET();
    http.end();
  }
}

void checkWeatherAPI() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    String url = "https://api.openweathermap.org/data/2.5/forecast?lat=" + String(OWM_LAT) + "&lon=" + String(OWM_LON) + "&appid=" + String(OWM_API_KEY) + "&units=metric";
    http.begin(url);
    int httpCode = http.GET();
    if (httpCode == 200) {
      String payload = http.getString();
      DynamicJsonDocument doc(2048);
      deserializeJson(doc, payload);
      float pop = doc["list"][0]["pop"].as<float>() * 100.0;
      isRainingExpected = (pop > 60.0);
    }
    http.end();
  }
}

void readAllSensors() {
  // 1. Đọc độ ẩm đất
  int rawADC = analogRead(PIN_SOIL_ADC);
  currentSoilMoisture = map(rawADC, SOIL_AIR_VALUE, SOIL_WATER_VALUE, 0, 100);
  currentSoilMoisture = constrain(currentSoilMoisture, 0.0, 100.0);

  // 2. Đọc BME280
  currentTemp = bme.readTemperature();
  currentHum = bme.readHumidity();
  currentPressure = bme.readPressure() / 100.0F; // Pa sang hPa

  // Đệm áp suất tính độ dốc cục bộ
  pressureBuffer[pressureIndex] = currentPressure;
  pressureIndex = (pressureIndex + 1) % 10;
  float pressureDrop = pressureBuffer[0] - currentPressure;
  if (pressureDrop >= rainThresholdHpa) {
    isRainingExpected = true;
  }

  // 3. Đọc BH1750
  currentLux = lightMeter.readLightLevel();
}

void checkSmartAdvisor() {
  if (currentSoilMoisture < 25.0) {
    sendTelegramAlert("⚠ [EcoGrowth Hub]: Đất khô hạn nghiêm trọng! Ẩm đất: " + String(currentSoilMoisture, 1) + "%");
  } else if (currentTemp > 37.0 && currentHum < 40.0) {
    sendTelegramAlert("🔥 [EcoGrowth Hub]: Cảnh báo sốc nhiệt! Nhiệt độ: " + String(currentTemp, 1) + "°C");
  } else if (currentLux < 300.0) {
    sendTelegramAlert("☁ [EcoGrowth Hub]: Cảnh báo thiếu ánh sáng quang hợp!");
  }
}

void syncFirebaseCurrent(bool isPumpOn) {
  FirebaseJson json;
  json.set("soil_moisture", currentSoilMoisture);
  json.set("temperature", currentTemp);
  json.set("humidity", currentHum);
  json.set("lux", currentLux);
  json.set("pressure", currentPressure);
  json.set("pump_status", isPumpOn ? "ON" : "OFF");
  json.set("rain_flag", isRainingExpected);
  json.set("last_updated", (int)(millis() / 1000));
  Firebase.setJSON(fbdo, "/current", json);
}

void syncFirebaseConfigRead() {
  if (Firebase.getJSON(fbdo, "/config")) {
    FirebaseJson &json = fbdo.jsonObject();
    FirebaseJsonData jsonData;
    
    if (json.get(jsonData, "soil_setpoint")) targetSetpoint = jsonData.floatValue;
    if (json.get(jsonData, "mode")) currentMode = jsonData.stringValue;
    if (json.get(jsonData, "manual_trigger")) manualTriggerFlag = jsonData.boolValue;
    if (json.get(jsonData, "rain_threshold_hpa")) rainThresholdHpa = jsonData.floatValue;
    if (json.get(jsonData, "telegram_chat_id")) userChatId = jsonData.stringValue;

    float p = 2.0, i = 0.5, d = 0.1;
    if (json.get(jsonData, "kp")) p = jsonData.floatValue;
    if (json.get(jsonData, "ki")) i = jsonData.floatValue;
    if (json.get(jsonData, "kd")) d = jsonData.floatValue;
    pid.setGains(p, i, d);
  }
}

// =================== SETUP ===================
void setup() {
  Serial.begin(115200);
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

  // Khởi tạo chân ngoại vi
  pinMode(PIN_RELAY_PUMP, OUTPUT);
  digitalWrite(PIN_RELAY_PUMP, LOW); // Tắt bơm ban đầu
  pinMode(PIN_FLOW_SENSOR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_FLOW_SENSOR), flowPulseCounter, RISING);

  // Khởi tạo cảm biến I2C
  bme.begin(0x76);
  lightMeter.begin();

  // Kết nối WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println("\n[WiFi] Đã kết nối thành công!");

  // Kết nối Firebase
  fbConfig.database_url = FIREBASE_HOST;
  fbConfig.signer.tokens.legacy_token = FIREBASE_AUTH;
  Firebase.begin(&fbConfig, &auth);
  Firebase.reconnectWiFi(true);

  // Đọc dữ liệu ban đầu
  readAllSensors();
  for (int i = 0; i < 10; i++) pressureBuffer[i] = currentPressure;
  checkWeatherAPI();
  syncFirebaseConfigRead();

  sendTelegramAlert("🌱 [EcoGrowth Hub V2]: ESP32 đã khởi động và sẵn sàng điều khiển!");
}

// =================== LOOP: FINITE STATE MACHINE (FSM) ===================
void loop() {
  unsigned long currentMillis = millis();

  // 1. Chu kỳ đọc cảm biến & đọc cấu hình Cloud mỗi 3 giây
  if (currentMillis - lastSensorTick >= 3000) {
    lastSensorTick = currentMillis;
    readAllSensors();
    syncFirebaseConfigRead();
    if (currentState == STATE_IDLE) {
      syncFirebaseCurrent(false);
    }
  }

  // 2. Chu kỳ kiểm tra thời tiết API mỗi 30 phút
  if (currentMillis - lastWeatherTick >= 1800000) {
    lastWeatherTick = currentMillis;
    checkWeatherAPI();
  }

  // 3. Chu kỳ chẩn đoán Smart Advisor mỗi 15 phút
  if (currentMillis - lastSmartAdvisorTick >= 900000) {
    lastSmartAdvisorTick = currentMillis;
    checkSmartAdvisor();
  }

  // 4. MÁY TRẠNG THÁI VẬN HÀNH (FSM)
  switch (currentState) {
    case STATE_IDLE:
      if (currentMode == "MANUAL" && manualTriggerFlag) {
        currentState = STATE_MANUAL;
        moistureBeforeCycle = currentSoilMoisture;
        pulseCount = 0;
        mlAccumulated = 0;
        manualStartTime = millis();
        digitalWrite(PIN_RELAY_PUMP, HIGH); // Bật bơm
        syncFirebaseCurrent(true);
      } else if (currentMode == "AUTO") {
        if (isRainingExpected) {
          currentState = STATE_RAIN_HOLD;
        } else if (currentSoilMoisture < (targetSetpoint - 1.0)) {
          currentState = STATE_WATERING;
          moistureBeforeCycle = currentSoilMoisture;
          pulseCount = 0;
          mlAccumulated = 0;
          pid.reset();
          digitalWrite(PIN_RELAY_PUMP, HIGH); // Bật bơm
          syncFirebaseCurrent(true);
        }
      }
      break;

    case STATE_RAIN_HOLD:
      digitalWrite(PIN_RELAY_PUMP, LOW);
      syncFirebaseCurrent(false);
      // Ghi nhận bản ghi hoãn tưới vào Firebase /history
      {
        FirebaseJson hist;
        hist.set("timestamp", (int)(millis() / 1000));
        hist.set("ml_used", 0);
        hist.set("moisture_before", currentSoilMoisture);
        hist.set("moisture_after", currentSoilMoisture);
        hist.set("kp_updated", pid.getKp());
        hist.set("trigger", "RAIN_HOLD");
        Firebase.pushJSON(fbdo, "/history", hist);
      }
      currentState = STATE_IDLE;
      break;

    case STATE_WATERING:
      mlAccumulated = pulseCount * 2.25;
      
      // Kiểm tra điều kiện ngắt: đạt Setpoint hoặc vượt quá mức an toàn 350ml
      if (currentSoilMoisture >= targetSetpoint || mlAccumulated >= 350.0) {
        digitalWrite(PIN_RELAY_PUMP, LOW); // Ngắt bơm
        currentState = STATE_LEARNING;
      }
      break;

    case STATE_MANUAL:
      mlAccumulated = pulseCount * 2.25;
      // Ngắt khi bơm đủ 150ml hoặc timeout 60s
      if (mlAccumulated >= 150.0 || (millis() - manualStartTime >= 60000)) {
        digitalWrite(PIN_RELAY_PUMP, LOW); // Ngắt bơm
        
        // TỰ ĐỘNG TRẢ VỀ AUTO TRÊN FIREBASE (Edge-Driven)
        FirebaseJson patch;
        patch.set("manual_trigger", false);
        patch.set("mode", "AUTO");
        Firebase.updateNode(fbdo, "/config", patch);
        
        currentMode = "AUTO";
        manualTriggerFlag = false;
        currentState = STATE_LEARNING;
      }
      break;

    case STATE_LEARNING:
      readAllSensors(); // Cập nhật lại độ ẩm sau khi ngấm nước
      
      // Cập nhật Adaptive Gain Kp
      pid.autoTuneKp(mlAccumulated, moistureBeforeCycle, currentSoilMoisture);

      // Ghi nhật ký tưới vào /history
      {
        FirebaseJson hist;
        hist.set("timestamp", (int)(millis() / 1000));
        hist.set("ml_used", mlAccumulated);
        hist.set("moisture_before", moistureBeforeCycle);
        hist.set("moisture_after", currentSoilMoisture);
        hist.set("kp_updated", pid.getKp());
        hist.set("trigger", (currentMode == "MANUAL") ? "MANUAL" : "AUTO");
        Firebase.pushJSON(fbdo, "/history", hist);

        // Đồng bộ Kp mới lên /config
        Firebase.setFloat(fbdo, "/config/kp", pid.getKp());
      }

      sendTelegramAlert("✅ [Tưới Xong]: Đã bơm " + String(mlAccumulated, 0) + "ml. Độ ẩm: " + String(moistureBeforeCycle, 1) + "% -> " + String(currentSoilMoisture, 1) + "%. Kp mới: " + String(pid.getKp(), 2));

      syncFirebaseCurrent(false);
      currentState = STATE_IDLE;
      break;
  }
}