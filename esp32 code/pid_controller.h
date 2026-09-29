#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

class AdaptivePID {
private:
  float kp, ki, kd;
  float integral;
  float prevError;
  unsigned long lastTime;

public:
  AdaptivePID(float p = 2.0, float i = 0.5, float d = 0.1) {
    kp = p; ki = i; kd = d;
    integral = 0;
    prevError = 0;
    lastTime = millis();
  }

  void setGains(float p, float i, float d) {
    kp = p; ki = i; kd = d;
  }

  float getKp() { return kp; }

  void reset() {
    integral = 0;
    prevError = 0;
    lastTime = millis();
  }

  // Tính toán Output PID vòng kín: Output = Kp*E + Ki*∫E + Kd*(dE/dt)
  float compute(float setpoint, float input) {
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    if (dt <= 0.0) dt = 0.1;

    float error = setpoint - input;
    integral += error * dt;
    integral = constrain(integral, -50.0, 50.0); // Chống tích lũy bão hòa Anti-windup

    float derivative = (error - prevError) / dt;
    float output = (kp * error) + (ki * integral) + (kd * derivative);

    prevError = error;
    lastTime = now;
    return constrain(output, 0.0, 100.0);
  }

  // Thuật toán Tự học (Adaptive Gain Tuning) theo đặc tính thực tế của đất
  void autoTuneKp(float mlUsed, float moistureBefore, float moistureAfter) {
    float gain = moistureAfter - moistureBefore;
    if (gain <= 0.2) gain = 0.2; // Tránh chia cho 0

    // Tỉ lệ tiêu chuẩn hệ thống: 11.5 ml/1% ẩm
    float actualRatio = mlUsed / gain;
    float factor = constrain(11.5 / actualRatio, 0.85, 1.15);

    kp = constrain(kp * factor, 0.5, 6.0);
  }
};

#endif