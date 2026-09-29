# 🌱 EcoGrowth Hub V2 — Smart Adaptive PID Irrigation & Dual Rain Forecasting IoT System


> **EcoGrowth Hub V2** là hệ thống tưới cây tự động khép kín ứng dụng lý thuyết **Điều khiển Tự động (Control Theory)** kết hợp kiến trúc **IoT 3 lớp**. Khác biệt với các thiết bị bật/tắt (On/Off hysteresis) thông thường, EcoGrowth Hub V2 sử dụng bộ điều khiển **PID đóng vòng kín** kết hợp thuật toán **Adaptive Gain Tuning** tự học hệ số $K_p$ on-device và cơ chế **dự báo mưa kép** bảo vệ cây ngay cả khi mất mạng.

---

## 📌 Bảng So Sánh Tính Năng Cốt Lõi

| Tiêu chí | Đồ án tưới On/Off thông thường | EcoGrowth Hub V2 |
| :--- | :--- | :--- |
| **Giải thuật tưới** | Hysteresis thô sơ (gây sốc ẩm, dao động răng cưa) | **Vòng kín PID ($P-I-D$)** mượt mà + **Adaptive Gain** tự học $K_p$ theo từng loại đất |
| **Dự báo thời tiết** | Không có hoặc chỉ phụ thuộc API 1 chiều | **Dự báo mưa kép**: API OpenWeatherMap + Đo độ sụt áp suất $\Delta P \ge 1.5\text{ hPa}$ qua BME280 khi mất mạng |
| **Xử lý tại biên (Edge)** | Phụ thuộc hoàn toàn vào Cloud / Blynk | **FSM 6 trạng thái tự trị**: Mất mạng vẫn tưới chính xác; tự ngắt khi đủ $150\text{ ml}$ và tự trả về `AUTO` |
| **Giám sát & Cố vấn** | Chỉ hiển thị chỉ số cảm biến rời rạc | **Smart Advisor**: Tự phát hiện Khô hạn, Ngập úng, Sốc nhiệt, Thiếu sáng & cảnh báo Telegram |
| **Nhật ký & Báo cáo** | Dữ liệu tức thời, không lưu vết | **Digital Garden Log**: Lưu trữ thể tích nước ($ml$), độ ẩm trước/sau, lịch sử tự học của đất |

---

## 🏛️ Kiến Trúc Hệ Thống (3-Tier Architecture)

```
        [ Cảm biến tại chậu ] ──(ADC / I2C / Hardware Interrupt)──┐
                                                                 │
  ┌──────────────────────────────────────────────────────────────▼──────────────────────────┐
  │ LỚP 2: XỬ LÝ BIÊN & ĐIỀU KHIỂN (ESP32 Edge Layer)                                       │
  │  - FSM 6 trạng thái: IDLE ➔ CHECKING ➔ WATERING / RAIN_HOLD / MANUAL ➔ LEARNING         │
  │  - Closed-loop PID & Tự hiệu chỉnh Kp: Kp_new = Kp_old * (StandardRatio / ActualRatio)  │
  │  - Non-blocking loop với millis() | Ngắt Interrupt đếm xung Flow Sensor YF-S201         │
  └──────────────────────────────┬──────────────────────────────▲───────────────────────────┘
                                 │ (Telemetry: /current)        │ (Commands: /config)
                                 ▼                              │
  ┌─────────────────────────────────────────────────────────────────────────────────────────┐
  │ LỚP 3: ĐÁM MÂY & ỨNG DỤNG (Cloud & Application Layer)                                   │
  │  - Firebase Realtime Database: Lưu trữ JSON phân luồng, đồng bộ 2 chiều tức thì         │
  │  - MQTT Broker (HiveMQ Cloud): Giao tiếp Pub/Sub độ trễ thấp phục vụ Debug / Node-RED   │
  │  - Web Dashboard: HTML5, Tailwind CSS, Chart.js giám sát và điều khiển trực quan        │
  │  - OpenWeatherMap API: Phân tích xác suất mưa theo tọa độ GPS                           │
  │  - Telegram Bot API: Đẩy thông báo khẩn cấp (Push Notification) đến điện thoại          │
  └─────────────────────────────────────────────────────────────────────────────────────────┘
```

---

## ⚙️ Sơ Đồ Chân Phần Cứng (Hardware Pinout)

| Linh kiện | Chân ESP32 | Giao thức / Chức năng | Ghi chú |
| :--- | :--- | :--- | :--- |
| **Capacitive Soil Moisture v1.2** | `GPIO 34` | ADC1 (Analog In) | Chống ăn mòn điện hóa, calib Air/Water |
| **BME280** (Nhiệt/Ẩm/Khí áp) | `GPIO 21` (SDA), `GPIO 22` (SCL) | I2C ($0\text{x}76$) | Cảnh báo sốc nhiệt + Đo sụt áp suất dự báo mưa |
| **BH1750** (Cường độ ánh sáng) | `GPIO 21` (SDA), `GPIO 22` (SCL) | I2C ($0\text{x}23$) | Cảnh báo thiếu sáng quang hợp (Lux) |
| **Flow Sensor YF-S201** | `GPIO 18` | Digital In (`RISING` Interrupt) | Đếm xung đo thể tích nước ($1\text{ xung} \approx 2.25\text{ ml}$) |
| **Relay Module (Bơm 5V/12V)** | `GPIO 23` | Digital Out (`ACTIVE HIGH`) | Điều khiển ngắt/mở nguồn máy bơm |

---

## 🧮 Giải Thuật Điều Khiển & Tự Học

### 1. Thuật toán PID đóng vòng kín
$$\text{Output}(t) = K_p \cdot e(t) + K_i \int_0^t e(\tau)d\tau + K_d \cdot \frac{de(t)}{dt}$$

* Sai số hệ thống: $e(t) = \text{Setpoint} - \text{SoilMoisture}(t)$.
* Tích hợp cơ chế **Anti-windup** giới hạn tích phân $(\int e \in [-50, 50])$ chống hiện tượng quá nhiệt/bão hòa tích lũy.

### 2. Tự học hệ số thích ứng ($K_p$ Adaptive Gain)
Sau mỗi chu kỳ tưới xong, ESP32 tự động tính toán lại mức hấp thụ thực tế của đất:

$$\Delta\text{Moisture} = \text{Moisture}_{\text{after}} - \text{Moisture}_{\text{before}}$$

$$\text{ActualRatio} = \frac{\text{ml}_{\text{used}}}{\Delta\text{Moisture}}$$

$$\text{Factor} = \text{constrain}\left(\frac{\text{StandardRatio}}{\text{ActualRatio}},\, 0.85,\, 1.15\right) \quad (\text{với } \text{StandardRatio} = 11.5\text{ ml}/1\%)$$

$$K_{p(\text{new})} = \text{constrain}(K_{p(\text{old})} \times \text{Factor},\, 0.5,\, 6.0)$$

---

## 📁 Cấu Trúc Thư Mục Repository

```text
ecogrowth-hub-v2/
├── firmware/                      # Mã nguồn ESP32 (Arduino IDE / PlatformIO)
│   ├── EcoGrowth_Hub_V2.ino       # Vòng lặp chính FSM, ngắt xung và đồng bộ Cloud
│   ├── config.h                   # Cấu hình PIN, Wi-Fi, API Keys, struct PlantProfile
│   └── pid_controller.h           # Class tính toán PID & Thuật toán Adaptive Gain
├── web-dashboard/                 # Mã nguồn Web Frontend
│   ├── index.html                 # UI Clean Dashboard phong cách Green Tech
│   ├── style.css                  # Tùy biến animations, LED status, scrollbar
│   └── app.js                     # Điều khiển kết nối Firebase SDK v9 Compat & Chart.js
├── simulator/                     # Công cụ kiểm thử offline
│   ├── esp32_simulator.html       # Giả lập vi điều khiển ESP32 ảo trên trình duyệt
│   └── nodered_flows.json         # Luồng debug MQTT qua Node-RED
└── README.md                      # Tài liệu kỹ thuật dự án
```

---

## 🚀 Hướng Dẫn Cài Đặt & Triển Khai

### 1. Triển khai Web Dashboard
1. Sao chép kho lưu trữ:
   ```bash
   git clone https://github.com/letrungnam1212/ecogrowth-hub-v2.git
   cd ecogrowth-hub-v2/web-dashboard
   ```
2. Cập nhật cấu hình URL Firebase tại `app.js` nếu sử dụng cơ sở dữ liệu riêng:
   ```javascript
   const firebaseConfig = {
     databaseURL: "https://your-project-id-default-rtdb.firebaseio.com/"
   };
   ```
3. Mở file `index.html` trực tiếp trên trình duyệt hoặc chạy qua tiện ích **Live Server**.

### 2. Triển khai Web lên Firebase Hosting (Tùy chọn)
```bash
npm install -g firebase-tools
firebase login
firebase init hosting
firebase deploy --only hosting
```

### 3. Nạp Firmware ESP32
1. Mở Arduino IDE, cài đặt các thư viện thông qua **Library Manager**:
   * `Firebase ESP32 Client` (bởi Mobizt)
   * `ArduinoJson` (v6.x trở lên)
   * `Adafruit BME280 Library`
   * `BH1750` (bởi Christopher Laws)
2. Mở thư mục `firmware/` trong Arduino IDE, điền thông tin mạng và API tại `config.h`:
   ```cpp
   #define WIFI_SSID           "Tên_WiFi_Của_Bạn"
   #define WIFI_PASSWORD       "Mat_Khau_WiFi"
   #define FIREBASE_HOST       "https://your-project-default-rtdb.firebasedatabase.app/"
   #define OWM_API_KEY         "OpenWeatherMap_API_Key"
   #define TG_BOT_TOKEN        "Telegram_Bot_Token"
   #define TG_DEFAULT_CHAT_ID  "Telegram_Chat_ID"
   ```
3. Chọn board **ESP32 Dev Module**, cắm cáp và bấm **Upload**.

---

## 📊 Cấu Trúc Dữ Liệu Firebase Realtime Database

* `/current`: Cập nhật liên tục mỗi 3-5 giây (telemetry cảm biến).
  ```json
  {
    "soil_moisture": 45.2,
    "temperature": 29.1,
    "humidity": 62.0,
    "lux": 1150,
    "pressure": 1012.3,
    "pump_status": "OFF",
    "rain_flag": false,
    "last_updated": 1727581230
  }
  ```
* `/config`: Đồng bộ tham số 2 chiều giữa Web và ESP32.
  ```json
  {
    "mode": "AUTO",
    "soil_setpoint": 60,
    "kp": 2.05,
    "ki": 0.5,
    "kd": 0.1,
    "manual_trigger": false,
    "rain_threshold_hpa": 1.5,
    "telegram_chat_id": "8984362218"
  }
  ```
* `/history`: Nhật ký chu kỳ tưới dạng append-only phục vụ thống kê và vẽ biểu đồ.
  ```json
  {
    "timestamp": 1727581300,
    "ml_used": 148.5,
    "moisture_before": 42.0,
    "moisture_after": 60.5,
    "kp_updated": 2.12,
    "trigger": "AUTO"
  }
  ```

---

## 👨‍💻 Tác Giả & Bản Quyền

* **Tác giả:** Lê Trung Nam ([@letrungnam1212](https://github.com/letrungnam1212))
* **Email liên hệ:** trnam.automation@gmail.com
* **Đề tài:** Đồ án IoT / Samsung Innovation Campus (SIC) Capstone Project
