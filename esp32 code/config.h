#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// =================== 1. CẤU HÌNH KẾT NỐI WIFI & CLOUD ===================
#define WIFI_SSID           "YOUR_WIFI_SSID"          // Điền tên WiFi nhà bạn
#define WIFI_PASSWORD       "YOUR_WIFI_PASSWORD"      // Điền mật khẩu WiFi

#define FIREBASE_HOST       "https://ecogrowth-v2-default-rtdb.asia-southeast1.firebasedatabase.app/"
#define FIREBASE_AUTH       ""                        // Để trống nếu dùng Rule public test

#define OWM_API_KEY         "45c6f318d9053113d5b865f0117fedc5" // API Key OpenWeatherMap
#define OWM_LAT             "10.762622"               // Tọa độ TP.HCM
#define OWM_LON             "106.660172"

#define TG_BOT_TOKEN        "8952177455:AAEvtIUy5aSurX1Pz7OJ22fmm7u8gD8wJnU" // Token Telegram Bot
#define TG_DEFAULT_CHAT_ID  "8984302218"              // Chat ID Telegram của bạn

// =================== 2. SƠ ĐỒ CHÂN NGOẠI VI (PINOUT) ===================
#define PIN_SOIL_ADC        34      // ADC1 - Cảm biến độ ẩm đất Capacitive v1.2
#define PIN_FLOW_SENSOR     18      // Digital In (Interrupt) - Cảm biến Flow Sensor YF-S201
#define PIN_RELAY_PUMP      23      // Digital Out - Điều khiển Relay bơm
#define PIN_I2C_SDA         21      // Bus I2C - BME280 & BH1750
#define PIN_I2C_SCL         22      // Bus I2C - BME280 & BH1750

// Hiệu chuẩn ADC cảm biến đất (Calib)
#define SOIL_AIR_VALUE      3200.0  // Giá trị ADC khi để ngoài không khí khô (0%)
#define SOIL_WATER_VALUE    1400.0  // Giá trị ADC khi nhúng ngập nước (100%)

// =================== 3. STRUCT & ENUM KIẾN TRÚC FSM ===================
enum SystemState {
  STATE_IDLE,       // Chờ chu kỳ đo tiếp theo (3-10s)
  STATE_CHECKING,   // Đọc cảm biến, tính Error, check cờ mưa
  STATE_RAIN_HOLD,  // Dự báo mưa -> Hoãn tưới bảo vệ rễ
  STATE_WATERING,   // Bật bơm đóng vòng PID, đếm xung ml
  STATE_LEARNING,   // Tính Adaptive Kp, lưu lịch sử, Sync Firebase
  STATE_MANUAL      // Nhận lệnh tưới cưỡng bức (đủ 150ml tự trả về AUTO)
};

struct PlantProfile {
  const char* name;
  float setpoint;    // %
  float default_kp;
  float default_ki;
  float default_kd;
};

const PlantProfile PLANT_MODES[3] = {
  {"Cây ưa ẩm",    70.0, 2.5, 0.6, 0.1},
  {"Cây chịu hạn",  35.0, 1.5, 0.3, 0.05},
  {"Cây ra hoa",   55.0, 2.0, 0.5, 0.1}
};

#endif