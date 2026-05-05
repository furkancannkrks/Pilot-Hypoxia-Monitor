#include <Wire.h>
#include <Adafruit_BMP280.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

// ————— DURUM MAKİNESİ —————
enum Durum { NORMAL, ONLEYICI_UYARI, KRITIK_ACIL, SENSOR_HATASI };

// ————— PİNLER —————
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
volatile bool mpuVeriHazir = false;
float egimKalibrasyonu = 0;

// ————— EŞİK DEĞERLERİ —————
#define SPO2_KRITIK      60.0
#define BASINC_DUSUS     0001.0
#define BAS_EGIMI_KRITIK 30.0

// ————— HAREKETLİ ORTALAMA (Basınç filtresi) —————
#define FILTER_SIZE 10
float basincBuffer[FILTER_SIZE];
int   basincIndex = 0;
bool  basincDolu  = false;

float basincFiltrele(float yeniDeger) {
  basincBuffer[basincIndex] = yeniDeger;
  basincIndex = (basincIndex + 1) % FILTER_SIZE;
  if (basincIndex == 0) basincDolu = true;
  int sayi = basincDolu ? FILTER_SIZE : basincIndex;
  float toplam = 0;
  for (int i = 0; i < sayi; i++) toplam += basincBuffer[i];
  return toplam / sayi;
}

// ————— NON-BLOCKING LED —————
unsigned long ledOncekiZaman = 0;
bool ledDurum = false;
int ledYanipSonmeHizi = 0;

void ledGuncelle() {
  if (ledYanipSonmeHizi == -1) { digitalWrite(LED_PIN, HIGH); return; }
  if (ledYanipSonmeHizi ==  0) { digitalWrite(LED_PIN, LOW);  return; }
  unsigned long simdi = millis();
  if (simdi - ledOncekiZaman >= (unsigned long)ledYanipSonmeHizi) {
    ledOncekiZaman = simdi;
    ledDurum = !ledDurum;
    digitalWrite(LED_PIN, ledDurum);
  }
}

// ————— MPU6050 INTERRUPT —————
void IRAM_ATTR mpuInterrupt() {
  mpuVeriHazir = true;
}

// ————— MPU6050 FONKSİYONLARI —————
void initMpu6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(100);

  // Örnekleme hızı bölen: 19 → 1000/(1+19) = 50Hz
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x19); // SMPLRT_DIV
  Wire.write(0x13);
  Wire.endTransmission();

  // Düşük geçiren filtre aktif
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

float hesaplaEgim(float ax, float ay, float az) {
  return atan2(sqrt(ax*ax + ay*ay), az) * 180.0 / PI;
}

// ————— ÇAPRAZ DOĞRULAMA (Güvenli Havacılık Mantığı) —————
Durum caprazDogrula(float spo2, bool spo2Gecerli, float basincDusus, float egim) {
  if (!spo2Gecerli) {
    Serial.println("[STATUS] SENSOR ERROR: SpO2 measurement invalid, check the sensor!");
    return SENSOR_HATASI;
  }

  bool spo2Kritik   = spo2 < SPO2_KRITIK;
  bool basincKritik = basincDusus > BASINC_DUSUS;
  bool egimKritik   = egim > BAS_EGIMI_KRITIK;

  // SEVİYE 2 (KABİN ALARMI): Eğim KESİNLİKLE bozulmuş olmalı VE yanında bir risk olmalı.
  // Yani: (Baş düştü VE Oksijen azaldı) VEYA (Baş düştü VE Basınç azaldı)
  if (egimKritik && (spo2Kritik || basincKritik)) {
    return KRITIK_ACIL;
  }

  // SEVİYE 1 (PİLOTA UYARI): Oksijen veya Basınç düşmüş ama BAŞ DİMDİK (Pilot uyanık)
  if (spo2Kritik || basincKritik) {
    return ONLEYICI_UYARI;
  }

  // Sadece baş eğimi varsa (Aşağı bakıyorsa) veya her şey normalse
  return NORMAL;
}

// ————— GLOBAL ZAMANLAR —————
float oncekiBasincFiltreli = 0;
unsigned long oncekiZaman  = 0;

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

  // İlk basınç
  float ilkBasinc = bmp.readPressure() / 100.0;
  for (int i = 0; i < FILTER_SIZE; i++) basincBuffer[i] = ilkBasinc;
  basincDolu = true;
  oncekiBasincFiltreli = ilkBasinc;
  oncekiZaman = millis();

  // MAX30102 kalibrasyon
  Serial.println("Calibration starting, please attach the oxygen sensor...");
  for (int i = 0; i < BUFFER_LENGTH; i++) {
    while (!particleSensor.available()) particleSensor.check();
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i]  = particleSensor.getIR();
    particleSensor.nextSample();
  }
  maxim_heart_rate_and_oxygen_saturation(irBuffer, BUFFER_LENGTH, redBuffer,
                                          &spo2Val, &spo2Valid, &heartRate, &hrValid);

  // MPU6050 eğim kalibrasyonu
  float kax, kay, kaz;
  readMpu6050(kax, kay, kaz);
  egimKalibrasyonu = hesaplaEgim(kax, kay, kaz);
  Serial.print("Initial tilt angle: "); Serial.print(egimKalibrasyonu); Serial.println(" degrees");

  Serial.println("Calibration completed! System ready.");
  Serial.println("========================================");
}

// ————— LOOP —————
void loop() {
  // ——— MAX30102 buffer güncelle ———
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
  bool parmakVar = (irBuffer[BUFFER_LENGTH-1] > 30000);

  // ——— BMP280 ———
  float basincHam      = bmp.readPressure() / 100.0;
  float basincFiltreli = basincFiltrele(basincHam);
  float sicaklik       = bmp.readTemperature();

  unsigned long simdi  = millis();
  float gecenSure      = (simdi - oncekiZaman) / 1000.0;
  if (gecenSure <= 0.001) gecenSure = 0.001; // SIFIRA BÖLME HATASINI ENGELLER
  float basincDusus    = (oncekiBasincFiltreli - basincFiltreli) / gecenSure;
  if (basincDusus < 0) basincDusus = 0;
  oncekiBasincFiltreli = basincFiltreli;
  oncekiZaman          = simdi;

  // ——— MPU6050: Sadece interrupt geldiyse oku ———
  static float ax = 0, ay = 0, az = 0;
  if (mpuVeriHazir) {
    mpuVeriHazir = false;
    readMpu6050(ax, ay, az);
  }
  float egim = abs(hesaplaEgim(ax, ay, az) - egimKalibrasyonu);

  // ——— Nabız geçerlilik kontrolü ———
  bool nabizGecerli = (hrValid == 1 && parmakVar && heartRate > 40 && heartRate < 180);

  // ——— ÇAPRAZ DOĞRULAMA ———
  Durum durum = caprazDogrula((float)spo2Val, spo2Valid == 1 && parmakVar, basincDusus, egim);

  // ——— LED ———
  switch(durum) {
    case NORMAL:         ledYanipSonmeHizi = 0;   break;
    case ONLEYICI_UYARI: ledYanipSonmeHizi = 25; break;
    case KRITIK_ACIL:    ledYanipSonmeHizi = 2; break;
    case SENSOR_HATASI:  ledYanipSonmeHizi = -1;  break;
  }
  ledGuncelle();

  // ——— SERIAL ÇIKTI ———
  Serial.println("=== SENSOR DATA ===");
  Serial.print("[BMP280]   Temperature: "); Serial.print(sicaklik);
  Serial.print(" C  |  Pressure: "); Serial.print(basincFiltreli); Serial.println(" hPa");
  Serial.print("[MPU6050]  Head Tilt: "); Serial.print(egim); Serial.println(" degrees");
  Serial.print("[MAX30102] SpO2: ");
  if (spo2Valid == 1 && parmakVar) { Serial.print(spo2Val); Serial.println(" %"); }
  else Serial.println("Invalid - Sensor disconnection.");
  Serial.print("[MAX30102] Pulse: ");
  if (nabizGecerli) { Serial.print(heartRate); Serial.println(" bpm"); }
  else Serial.println("Invalid");

  Serial.print("[STATUS] ");
  switch(durum) {
    case NORMAL:         
      Serial.println("NORMAL - Flight is safe, values are stable."); 
      break;
    case ONLEYICI_UYARI: 
      Serial.println("*** STAGE 1 (PILOT WARNING) - Values are dropping! Put on your oxygen mask!"); 
      break;
    case KRITIK_ACIL:    
      Serial.println("!!! STAGE 2 (CABIN CREW ALERT) !!! - Pilot has lost consciousness, EMERGENCY INTERVENTION!"); 
      break;
    case SENSOR_HATASI:  
      Serial.println("SENSOR FAULT - Hardware check required."); 
      break;
  }
  Serial.println("----------------------------------------");
}
