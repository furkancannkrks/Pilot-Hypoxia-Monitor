#include <Wire.h>
#include <Adafruit_BMP280.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

// ————— STATE MACHINE —————
enum State { NORMAL, PREVENTIVE_WARNING, CRITICAL_EMERGENCY, SENSOR_FAULT };

// ————— PINS —————
#define LED_PIN  2
#define INT_MPU  18

// ————— BMP280 —————
Adafruit_BMP280 bmp;

// ————— MAX30102 —————
MAX30105 particleSensor;
#define BUFFER_LENGTH 100
static uint32_t irBuffer[BUFFER_LENGTH];
static uint32_t redBuffer[BUFFER_LENGTH];
int32_t spo2Val;
int8_t  spo2Valid;
int32_t heartRate;
int8_t  hrValid;

// ————— MPU6050 —————
#define MPU_ADDR 0x68
volatile bool mpuDataReady = false;
float tiltCalibration = 0;

// ————— THRESHOLDS —————
#define SPO2_CRITICAL      60.0
#define PRESSURE_DROP      0.001
#define HEAD_TILT_CRITICAL 30.0

// ————— MOVING AVERAGE (Pressure filter) —————
#define FILTER_SIZE 10
float pressureBuffer[FILTER_SIZE];
int   pressureIndex = 0;
bool  pressureFull  = false;

float filterPressure(float newValue) {
  pressureBuffer[pressureIndex] = newValue;
  pressureIndex = (pressureIndex + 1) % FILTER_SIZE;
  if (pressureIndex == 0) pressureFull = true;
  int count = pressureFull ? FILTER_SIZE : pressureIndex;
  float sum = 0;
  for (int i = 0; i < count; i++) sum += pressureBuffer[i];
  return sum / count;
}

// ————— NON-BLOCKING LED —————
unsigned long ledPreviousTime = 0;
bool ledState = false;
int ledBlinkRate = 0;

void updateLed() {
  if (ledBlinkRate == -1) { digitalWrite(LED_PIN, HIGH); return; }
  if (ledBlinkRate ==  0) { digitalWrite(LED_PIN, LOW);  return; }
  unsigned long currentTime = millis();
  if (currentTime - ledPreviousTime >= (unsigned long)ledBlinkRate) {
    ledPreviousTime = currentTime;
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState);
  }
}

// ————— MPU6050 INTERRUPT —————
void IRAM_ATTR mpuInterrupt() {
  mpuDataReady = true;
}

// ————— MPU6050 FUNCTIONS —————
void initMpu6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(100);

  // Sample rate divider: 19 → 1000/(1+19) = 50Hz
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x19); // SMPLRT_DIV
  Wire.write(0x13);
  Wire.endTransmission();

  // Low-pass filter active
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A); // CONFIG
  Wire.write(0x03); // 44Hz LPF
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x37);
  Wire.write(0x10);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x38);
  Wire.write(0x01);
  Wire.endTransmission();
}

void readMpu6050(float &ax, float &ay, float &az) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);
  int16_t raw_ax = Wire.read() << 8 | Wire.read();
  int16_t raw_ay = Wire.read() << 8 | Wire.read();
  int16_t raw_az = Wire.read() << 8 | Wire.read();
  ax = raw_ax / 16384.0;
  ay = raw_ay / 16384.0;
  az = raw_az / 16384.0;
}

float calculateTilt(float ax, float ay, float az) {
  return atan2(sqrt(ax*ax + ay*ay), az) * 180.0 / PI;
}

// ————— CROSS-VALIDATION (Safe Aviation Logic) —————
State crossValidate(float spo2, bool isSpo2Valid, float pressureDrop, float tilt) {
  if (!isSpo2Valid) {
    Serial.println("[STATUS] SENSOR ERROR: SpO2 measurement invalid, check the sensor!");
    return SENSOR_FAULT;
  }

  bool isSpo2Critical     = spo2 < SPO2_CRITICAL;
  bool isPressureCritical = pressureDrop > PRESSURE_DROP;
  bool isTiltCritical     = tilt > HEAD_TILT_CRITICAL;

  // STAGE 2 (CABIN ALARM): Tilt MUST BE critical AND there must be a risk alongside it.
  // Meaning: (Head tilt AND Oxygen dropped) OR (Head tilt AND Pressure dropped)
  if (isTiltCritical && (isSpo2Critical || isPressureCritical)) {
    return CRITICAL_EMERGENCY;
  }

  // STAGE 1 (PILOT WARNING): Oxygen or Pressure dropped but HEAD UPRIGHT (Pilot is conscious)
  if (isSpo2Critical || isPressureCritical) {
    return PREVENTIVE_WARNING;
  }

  // If only head is tilted (looking down) or everything is normal
  return NORMAL;
}

// ————— GLOBAL TIMING & VARIABLES —————
float previousFilteredPressure = 0;
unsigned long previousTime  = 0;

// ————— SETUP —————
void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(INT_MPU, INPUT_PULLUP);
  digitalWrite(LED_PIN, LOW);
  delay(1000);

  Wire.setBufferSize(512);
  Wire.begin(21, 22);
  delay(100);

  // BMP280
  if (!bmp.begin(0x76)) {
    Serial.println("ERROR: BMP280 not found!");
    while (1) delay(10);
  }

  // MPU6050
  initMpu6050();
  attachInterrupt(digitalPinToInterrupt(INT_MPU), mpuInterrupt, FALLING);

  // MAX30102
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("ERROR: MAX30102 not found!");
    while (1) delay(10);
  }
  particleSensor.setup(60, 4, 2, 100, 411, 4096);

  // Initial pressure
  float initialPressure = bmp.readPressure() / 100.0;
  for (int i = 0; i < FILTER_SIZE; i++) pressureBuffer[i] = initialPressure;
  pressureFull = true;
  previousFilteredPressure = initialPressure;
  previousTime = millis();

  // MAX30102 calibration
  Serial.println("Calibration starting, please attach the oxygen sensor...");
  for (int i = 0; i < BUFFER_LENGTH; i++) {
    while (!particleSensor.available()) particleSensor.check();
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i]  = particleSensor.getIR();
    particleSensor.nextSample();
  }
  maxim_heart_rate_and_oxygen_saturation(irBuffer, BUFFER_LENGTH, redBuffer,
                                          &spo2Val, &spo2Valid, &heartRate, &hrValid);

  // MPU6050 tilt calibration
  float cax, cay, caz;
  readMpu6050(cax, cay, caz);
  tiltCalibration = calculateTilt(cax, cay, caz);
  Serial.print("Initial tilt angle: "); Serial.print(tiltCalibration); Serial.println(" degrees");

  Serial.println("Calibration completed! System ready.");
  Serial.println("========================================");
}

// ————— LOOP —————
void loop() {
  // ——— Update MAX30102 buffer ———
  for (int i = 25; i < BUFFER_LENGTH; i++) {
    redBuffer[i - 25] = redBuffer[i];
    irBuffer[i - 25]  = irBuffer[i];
  }
  for (int i = 75; i < BUFFER_LENGTH; i++) {
    while (!particleSensor.available()) particleSensor.check();
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i]  = particleSensor.getIR();
    particleSensor.nextSample();
  }
  maxim_heart_rate_and_oxygen_saturation(irBuffer, BUFFER_LENGTH, redBuffer,
                                          &spo2Val, &spo2Valid, &heartRate, &hrValid);
                                          
  // Using Tissue Contact logic as we discussed
  bool hasTissueContact = (irBuffer[BUFFER_LENGTH-1] > 30000);

  // ——— BMP280 ———
  float rawPressure      = bmp.readPressure() / 100.0;
  float filteredPressure = filterPressure(rawPressure);
  float temperature      = bmp.readTemperature();

  unsigned long currentTime = millis();
  float elapsedTime         = (currentTime - previousTime) / 1000.0;
  
  if (elapsedTime <= 0.001) elapsedTime = 0.001; // PREVENTS DIVIDE BY ZERO ERROR
  
  float currentPressureDrop = (previousFilteredPressure - filteredPressure) / elapsedTime;
  if (currentPressureDrop < 0) currentPressureDrop = 0;
  
  previousFilteredPressure = filteredPressure;
  previousTime             = currentTime;

  // ——— MPU6050: Read only if interrupt occurred ———
  static float ax = 0, ay = 0, az = 0;
  if (mpuDataReady) {
    mpuDataReady = false;
    readMpu6050(ax, ay, az);
  }
  float tilt = abs(calculateTilt(ax, ay, az) - tiltCalibration);

  // ——— Pulse validity check ———
  bool isPulseValid = (hrValid == 1 && hasTissueContact && heartRate > 40 && heartRate < 180);

  // ——— CROSS-VALIDATION ———
  State currentState = crossValidate((float)spo2Val, spo2Valid == 1 && hasTissueContact, currentPressureDrop, tilt);

  // ——— LED ———
  switch(currentState) {
    case NORMAL:             ledBlinkRate = 0;   break;
    case PREVENTIVE_WARNING: ledBlinkRate = 25;  break;
    case CRITICAL_EMERGENCY: ledBlinkRate = 2;   break;
    case SENSOR_FAULT:       ledBlinkRate = -1;  break;
  }
  updateLed();

  // ——— SERIAL OUTPUT ———
  Serial.println("=== SENSOR DATA ===");
  Serial.print("[BMP280]   Temperature: "); Serial.print(temperature);
  Serial.print(" C  |  Pressure: "); Serial.print(filteredPressure); Serial.println(" hPa");
  Serial.print("[MPU6050]  Head Tilt: "); Serial.print(tilt); Serial.println(" degrees");
  Serial.print("[MAX30102] SpO2: ");
  if (spo2Valid == 1 && hasTissueContact) { Serial.print(spo2Val); Serial.println(" %"); }
  else Serial.println("Invalid - Sensor disconnection.");
  Serial.print("[MAX30102] Pulse: ");
  if (isPulseValid) { Serial.print(heartRate); Serial.println(" bpm"); }
  else Serial.println("Invalid");

  Serial.print("[STATUS] ");
  switch(currentState) {
    case NORMAL:         
      Serial.println("NORMAL - Flight is safe, values are stable."); 
      break;
    case PREVENTIVE_WARNING: 
      Serial.println("*** STAGE 1 (PILOT WARNING) - Values are dropping! Put on your oxygen mask!"); 
      break;
    case CRITICAL_EMERGENCY:    
      Serial.println("!!! STAGE 2 (CABIN CREW ALERT) !!! - Pilot has lost consciousness, EMERGENCY INTERVENTION!"); 
      break;
    case SENSOR_FAULT:   
      Serial.println("SENSOR FAULT - Hardware check required."); 
      break;
  }
  Serial.println("----------------------------------------");
}
