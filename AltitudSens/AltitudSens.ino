
#include <Wire.h>
#include <math.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>

//#define USE_BMP180 1
#define USE_BMP280 1

#if defined(USE_BMP280) && defined(USE_BMP180)
#error "Define solo uno: USE_BMP280 o USE_BMP180"
#endif
#if !defined(USE_BMP280) && !defined(USE_BMP180)
#define USE_BMP280 1
#endif

#if USE_BMP280
  #include <Adafruit_BMP280.h>
  Adafruit_BMP280 bmp280;
#elif USE_BMP180
  #include <Adafruit_BMP085.h>   // BMP180
  Adafruit_BMP085 bmp180;
#endif

Adafruit_MPU6050 mpu;

// Pines ESP32
static const int I2C_SDA = 21;
static const int I2C_SCL = 22;

// Config
static const uint32_t SAMPLE_MS = 40;       // 25 Hz
static const int CALIB_SAMPLES = 300;       // 12 s aprox a 40 ms

// Barómetro robusto
static const float MAX_PRESS_STEP_PA = 25.0f; // limita saltos muestra-a-muestra
static const float P_EMA_ALPHA = 0.18f;       // EMA presión
static const int P_MED_WIN = 5;               // mediana de 5

// Actitud
static const float ATT_ALPHA = 0.98f;         // gyro dominante, accel corrige
static const float G0 = 9.80665f;

// Kalman 1D (altura, velocidad)
static const float SIGMA_A = 2.5f;            // ruido accel (m/s^2), tunear
static const float R_BARO = 0.8f * 0.8f;      // varianza medición baro (m^2), tunear
static const float INNOV_GATE_M = 8.0f;       // rechazo de outliers baro en update

// Delta window
static const int DELTA_WINDOW = 10;

// Estado baro
float p0_pa = 101325.0f;
float p_raw_prev_pa = 101325.0f;
float p_ema_pa = 101325.0f;
float p_med_buf[P_MED_WIN];
int p_med_idx = 0;
int p_med_count = 0;

// Estado IMU
float gyro_bias_x = 0.0f, gyro_bias_y = 0.0f, gyro_bias_z = 0.0f;
float az_bias = 0.0f;
float roll = 0.0f, pitch = 0.0f;

// Estado Kalman
float x_h = 0.0f;    // altura fusionada (m)
float x_v = 0.0f;    // velocidad vertical fusionada (m/s)
float P00 = 10.0f, P01 = 0.0f, P10 = 0.0f, P11 = 10.0f;

// Salidas
float alt_raw_m = 0.0f;   // altura cruda baro
float alt_filt_m = 0.0f;  // altura fusionada

float p_hist[DELTA_WINDOW];
float h_hist[DELTA_WINDOW];
int hist_idx = 0;
bool first_sample = true;

uint32_t t0_ms = 0;
uint32_t last_ms = 0;
uint32_t last_us = 0;

static inline float pressureToAltitudeM(float p_pa, float p0_pa_local) {
  return 44330.0f * (1.0f - powf(p_pa / p0_pa_local, 0.1903f));
}

float medianN(const float *arr, int n) {
  float tmp[P_MED_WIN];
  for (int i = 0; i < n; i++) tmp[i] = arr[i];
  for (int i = 0; i < n - 1; i++) {
    for (int j = i + 1; j < n; j++) {
      if (tmp[j] < tmp[i]) {
        float t = tmp[i];
        tmp[i] = tmp[j];
        tmp[j] = t;
      }
    }
  }
  return tmp[n / 2];
}

float updatePressureRobust(float p_raw_pa) {
  // 1) clamp de salto
  float dp = p_raw_pa - p_raw_prev_pa;
  if (dp > MAX_PRESS_STEP_PA) dp = MAX_PRESS_STEP_PA;
  if (dp < -MAX_PRESS_STEP_PA) dp = -MAX_PRESS_STEP_PA;
  float p_clamped = p_raw_prev_pa + dp;
  p_raw_prev_pa = p_clamped;

  // 2) mediana
  p_med_buf[p_med_idx] = p_clamped;
  p_med_idx = (p_med_idx + 1) % P_MED_WIN;
  if (p_med_count < P_MED_WIN) p_med_count++;
  float p_med = medianN(p_med_buf, p_med_count);

  // 3) EMA
  p_ema_pa = P_EMA_ALPHA * p_med + (1.0f - P_EMA_ALPHA) * p_ema_pa;

  return p_ema_pa;
}

bool readBaro(float &temp_c, float &press_pa) {
#if USE_BMP280
  temp_c = bmp280.readTemperature();
  press_pa = bmp280.readPressure(); // Pa
#elif USE_BMP180
  temp_c = bmp180.readTemperature();
  press_pa = (float)bmp180.readPressure(); // Pa
#endif
  return !(isnan(temp_c) || isnan(press_pa) || press_pa <= 0.0f);
}

// Proyección a Z global usando roll/pitch
float bodyToWorldZ(float ax, float ay, float az, float roll_r, float pitch_r) {
  float sr = sinf(roll_r), cr = cosf(roll_r);
  float sp = sinf(pitch_r), cp = cosf(pitch_r);
  return (-ax * sp) + (ay * sr * cp) + (az * cr * cp);
}

void calibrateAll() {
  Serial.printf("# CAL_START samples=%d\n", CALIB_SAMPLES);

  double acc_p = 0.0;
  int valid_p = 0;

  double acc_gx = 0.0, acc_gy = 0.0, acc_gz = 0.0;
  double acc_roll = 0.0, acc_pitch = 0.0;
  int valid_imu = 0;

  // P0 + bias gyro + actitud inicial
  for (int i = 0; i < CALIB_SAMPLES; i++) {
    float t, p;
    if (readBaro(t, p)) {
      acc_p += p;
      valid_p++;
    }

    sensors_event_t a, g, tmp;
    if (mpu.getEvent(&a, &g, &tmp)) {
      float ax = a.acceleration.x;
      float ay = a.acceleration.y;
      float az = a.acceleration.z;

      float roll_acc = atan2f(ay, az);
      float pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az));

      acc_roll += roll_acc;
      acc_pitch += pitch_acc;

      acc_gx += g.gyro.x;
      acc_gy += g.gyro.y;
      acc_gz += g.gyro.z;
      valid_imu++;
    }

    delay(SAMPLE_MS);
  }

  if (valid_p > 0) p0_pa = (float)(acc_p / (double)valid_p);
  if (valid_imu > 0) {
    roll = (float)(acc_roll / (double)valid_imu);
    pitch = (float)(acc_pitch / (double)valid_imu);
    gyro_bias_x = (float)(acc_gx / (double)valid_imu);
    gyro_bias_y = (float)(acc_gy / (double)valid_imu);
    gyro_bias_z = (float)(acc_gz / (double)valid_imu);
  }

  // bias de aceleración vertical (lineal)
  double acc_az_lin = 0.0;
  int valid_az = 0;
  for (int i = 0; i < CALIB_SAMPLES; i++) {
    sensors_event_t a, g, tmp;
    if (mpu.getEvent(&a, &g, &tmp)) {
      float az_world = bodyToWorldZ(a.acceleration.x, a.acceleration.y, a.acceleration.z, roll, pitch);
      float az_lin = az_world - G0;
      acc_az_lin += az_lin;
      valid_az++;
    }
    delay(SAMPLE_MS);
  }
  az_bias = (valid_az > 0) ? (float)(acc_az_lin / (double)valid_az) : 0.0f;

  p_raw_prev_pa = p0_pa;
  p_ema_pa = p0_pa;
  for (int i = 0; i < P_MED_WIN; i++) p_med_buf[i] = p0_pa;

  x_h = 0.0f;
  x_v = 0.0f;
  P00 = 10.0f; P01 = 0.0f; P10 = 0.0f; P11 = 10.0f;

  Serial.printf("# CAL_DONE p0_pa=%.2f gyroBias(rad/s)=(%.4f,%.4f,%.4f) az_bias=%.4f\n",
                p0_pa, gyro_bias_x, gyro_bias_y, gyro_bias_z, az_bias);
}

void setup() {
  Serial.begin(115200);
  delay(300);

  Wire.begin(I2C_SDA, I2C_SCL);

#if USE_BMP280
  if (!bmp280.begin(0x76) && !bmp280.begin(0x77)) {
    Serial.println("# ERROR BMP280 not found");
    while (true) delay(1000);
  }
  bmp280.setSampling(
    Adafruit_BMP280::MODE_NORMAL,
    Adafruit_BMP280::SAMPLING_X2,
    Adafruit_BMP280::SAMPLING_X16,
    Adafruit_BMP280::FILTER_X16,
    Adafruit_BMP280::STANDBY_MS_1
  );
  Serial.println("# SENSOR BMP280");
#elif USE_BMP180
  if (!bmp180.begin()) {
    Serial.println("# ERROR BMP180 not found");
    while (true) delay(1000);
  }
  Serial.println("# SENSOR BMP180");
#endif

  if (!mpu.begin(0x68, &Wire)) {
    Serial.println("# ERROR MPU6050 not found at 0x68");
    while (true) delay(1000);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("# SENSOR MPU6050");

  calibrateAll();

  t0_ms = millis();
  last_ms = millis();
  last_us = micros();

  Serial.println("ms,alt_raw_m,alt_filt_m,dP_pa,dH_m");
}

void loop() {
  uint32_t now_ms = millis();
  if (now_ms - last_ms < SAMPLE_MS) return;
  last_ms = now_ms;

  uint32_t now_us = micros();
  float dt = (now_us - last_us) * 1e-6f;
  last_us = now_us;
  if (dt <= 0.0f || dt > 0.2f) dt = SAMPLE_MS * 1e-3f;

  // Barómetro 
  float temp_c, p_raw_pa;
  if (!readBaro(temp_c, p_raw_pa)) return;

  alt_raw_m = pressureToAltitudeM(p_raw_pa, p0_pa);

  float p_filt_pa = updatePressureRobust(p_raw_pa);
  float alt_baro_filt_m = pressureToAltitudeM(p_filt_pa, p0_pa);

  // IMU 
  sensors_event_t a, g, t;
  if (!mpu.getEvent(&a, &g, &t)) return;

  float gx = g.gyro.x - gyro_bias_x;
  float gy = g.gyro.y - gyro_bias_y;

  // Integración gyro
  roll += gx * dt;
  pitch += gy * dt;

  // Corrección con accel
  float roll_acc = atan2f(a.acceleration.y, a.acceleration.z);
  float pitch_acc = atan2f(-a.acceleration.x,
                           sqrtf(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z));
  roll = ATT_ALPHA * roll + (1.0f - ATT_ALPHA) * roll_acc;
  pitch = ATT_ALPHA * pitch + (1.0f - ATT_ALPHA) * pitch_acc;

  float az_world = bodyToWorldZ(a.acceleration.x, a.acceleration.y, a.acceleration.z, roll, pitch);
  float az_lin = (az_world - G0) - az_bias;  // m/s^2

  // Kalman 1D (predicción)
  x_h = x_h + x_v * dt + 0.5f * az_lin * dt * dt;
  x_v = x_v + az_lin * dt;

  float sa2 = SIGMA_A * SIGMA_A;
  float dt2 = dt * dt;
  float dt3 = dt2 * dt;
  float dt4 = dt2 * dt2;

  float Q00 = 0.25f * dt4 * sa2;
  float Q01 = 0.5f  * dt3 * sa2;
  float Q10 = Q01;
  float Q11 = dt2 * sa2;

  float nP00 = P00 + dt * (P10 + P01) + dt2 * P11 + Q00;
  float nP01 = P01 + dt * P11 + Q01;
  float nP10 = P10 + dt * P11 + Q10;
  float nP11 = P11 + Q11;

  P00 = nP00; P01 = nP01; P10 = nP10; P11 = nP11;

  // Kalman 1D, update con barómetro filtrado
  float innov = alt_baro_filt_m - x_h;
  if (fabsf(innov) < INNOV_GATE_M) {
    float S = P00 + R_BARO;
    float K0 = P00 / S;
    float K1 = P10 / S;

    x_h = x_h + K0 * innov;
    x_v = x_v + K1 * innov;

    float uP00 = (1.0f - K0) * P00;
    float uP01 = (1.0f - K0) * P01;
    float uP10 = P10 - K1 * P00;
    float uP11 = P11 - K1 * P01;

    P00 = uP00; P01 = uP01; P10 = uP10; P11 = uP11;
  }

  alt_filt_m = x_h; // salida filtrada = fusionada

  // dP y dH en ventana
  if (first_sample) {
    for (int i = 0; i < DELTA_WINDOW; i++) {
      p_hist[i] = p_filt_pa;
      h_hist[i] = alt_filt_m;
    }
    first_sample = false;
  }

  float dP_pa = p_filt_pa - p_hist[hist_idx];
  float dH_m  = alt_filt_m - h_hist[hist_idx];

  p_hist[hist_idx] = p_filt_pa;
  h_hist[hist_idx] = alt_filt_m;
  hist_idx = (hist_idx + 1) % DELTA_WINDOW;

  uint32_t t_ms = now_ms - t0_ms;

  // Formato (5 columnas)
  Serial.printf("%lu,%.4f,%.4f,%.4f,%.4f\n",
                (unsigned long)t_ms, alt_raw_m, alt_filt_m, dP_pa, dH_m);
}