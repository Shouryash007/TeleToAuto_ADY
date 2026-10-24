
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

Adafruit_MPU6050 mpu;

// Typical ESP32 DevKit I2C pins
constexpr int SDA_PIN = 21;
constexpr int SCL_PIN = 22;

constexpr uint32_t OUTPUT_PERIOD_MS = 20;  // 50 Hz
constexpr float ALPHA = 0.98f;
constexpr float RAD_TO_DEG_F = 57.2957795f;

float rollDeg = 0.0f;
float pitchDeg = 0.0f;
float yawDeg = 0.0f;

float rollZero = 0.0f;
float pitchZero = 0.0f;
float yawZero = 0.0f;

uint32_t lastSampleUs = 0;
uint32_t lastOutputMs = 0;
bool initialized = false;

void zeroOrientation() {
  rollZero = rollDeg;
  pitchZero = pitchDeg;
  yawZero = yawDeg;

  Serial.println("{\"type\":\"zeroed\"}");
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  if (!mpu.begin(0x68, &Wire)) {
    Serial.println("{\"type\":\"error\",\"message\":\"MPU6050 not found at 0x68\"}");
    while (true) {
      delay(1000);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Keep the sensor completely still during startup.
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  rollDeg = atan2f(a.acceleration.y, a.acceleration.z)
            * RAD_TO_DEG_F;

  pitchDeg = atan2f(
      -a.acceleration.x,
      sqrtf(a.acceleration.y * a.acceleration.y +
            a.acceleration.z * a.acceleration.z)
  ) * RAD_TO_DEG_F;

  yawDeg = 0.0f;
  lastSampleUs = micros();
  lastOutputMs = millis();
  initialized = true;

  Serial.println("{\"type\":\"ready\",\"sensor\":\"MPU6050\"}");
}

void loop() {
  // Accept a zeroing command from the serial connection.
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "ZERO") {
      zeroOrientation();
    }
  }

  uint32_t nowUs = micros();
  float dt = (nowUs - lastSampleUs) / 1000000.0f;
  lastSampleUs = nowUs;

  // Ignore abnormal timing intervals after startup or pauses.
  if (!initialized || dt <= 0.0f || dt > 0.1f) {
    return;
  }

  sensors_event_t a, g, temp;
  if (!mpu.getEvent(&a, &g, &temp)) {
    return;
  }

  // Accelerometer-derived tilt angles.
  float accelRoll = atan2f(
      a.acceleration.y, a.acceleration.z
  ) * RAD_TO_DEG_F;

  float accelPitch = atan2f(
      -a.acceleration.x,
      sqrtf(a.acceleration.y * a.acceleration.y +
            a.acceleration.z * a.acceleration.z)
  ) * RAD_TO_DEG_F;

  // Gyroscope readings from Adafruit are in radians/second.
  float gyroX = g.gyro.x * RAD_TO_DEG_F;
  float gyroY = g.gyro.y * RAD_TO_DEG_F;
  float gyroZ = g.gyro.z * RAD_TO_DEG_F;

  // Complementary filter: gyro response + accelerometer correction.
  rollDeg = ALPHA * (rollDeg + gyroX * dt)
            + (1.0f - ALPHA) * accelRoll;

  pitchDeg = ALPHA * (pitchDeg + gyroY * dt)
             + (1.0f - ALPHA) * accelPitch;

  // Relative yaw only; it will drift without an absolute reference.
  yawDeg += gyroZ * dt;

  // Emit one JSON line every 20 ms.
  uint32_t nowMs = millis();
  if (nowMs - lastOutputMs >= OUTPUT_PERIOD_MS) {
    lastOutputMs = nowMs;

    Serial.printf(
      "{\"type\":\"imu\",\"yaw\":%.2f,\"pitch\":%.2f,\"roll\":%.2f,\"timestamp_ms\":%lu}\n",
      yawDeg - yawZero,
      pitchDeg - pitchZero,
      rollDeg - rollZero,
      (unsigned long)nowMs
    );
  }
}
