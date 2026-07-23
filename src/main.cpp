#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <STM32FreeRTOS.h>
#include <string.h>

// ════════════════════════════════════════════════════════════
//  UART Port & Pin Bağlantıları
// ════════════════════════════════════════════════════════════
HardwareSerial Serial1(PA10, PA9);   // Debug / PC
HardwareSerial Serial2(PA3, PA2);    // GPS
HardwareSerial Serial3(PB11, PB10);  // RFD900x
HardwareSerial Serial6(PC7, PC6);    // RS232 – test cihazı (RX=PC7, TX=PC6)

const uint8_t MY_ID = 17;
const uint8_t GROUND_STATION_ID = 1;

#define BUZZER_PIN PE9

// ════════════════════════════════════════════════════════════
//  Sensor Data Strukturu (ivmə & bucaq əlavə edildi)
// ════════════════════════════════════════════════════════════
struct SensorData {
  float qw=1, qx=0, qy=0, qz=0;
  float bme_temp=0, bme_hum=0, bme_pres=0;
  float aht_temp=0, aht_hum=0;
  float altitude=0;
  float gps_lat=0.0f, gps_lon=0.0f, gps_alt=0.0f;
  bool  gps_fix=false;
  // Roket gövde eksenləri (EK-7 Bölüm 1.2)
  float accel_x=0, accel_y=0, accel_z=0;  // m/s²
  float angle_x=0, angle_y=0, angle_z=0;  // dərəcə (Euler)
};

// ════════════════════════════════════════════════════════════
//  Filter & Slerp
// ════════════════════════════════════════════════════════════
struct Kalman1D {
  float Q, R, x, P; bool ready = false;
  void init(float q, float r) { Q=q; R=r; P=1.0f; }
  float update(float z) {
    if (!ready) { x = z; ready = true; return x; }
    P += Q; const float K = P / (P + R);
    x += K * (z - x); P = (1.0f - K) * P; return x;
  }
};

struct QuatSlerp {
  float w=1, x=0, y=0, z=0, alpha; bool ready = false;
  void init(float a) { alpha = a; }
  void update(float nw, float nx, float ny, float nz) {
    if (!ready) { w=nw; x=nx; y=ny; z=nz; ready=true; return; }
    float dot = w*nw + x*nx + y*ny + z*nz;
    if (dot < 0.0f) { nw=-nw; nx=-nx; ny=-ny; nz=-nz; dot=-dot; }
    float t0, t1;
    if (dot < 0.9999f) {
      const float theta = acosf(dot), s = sinf(theta);
      t0 = sinf((1.0f - alpha) * theta) / s;
      t1 = sinf(alpha * theta) / s;
    } else { t0 = 1.0f - alpha; t1 = alpha; }
    w = t0*w + t1*nw; x = t0*x + t1*nx; y = t0*y + t1*ny; z = t0*z + t1*nz;
    const float n = sqrtf(w*w + x*x + y*y + z*z);
    if (n > 0.0f) { w/=n; x/=n; y/=n; z/=n; }
  }
};

SensorData shared;
SemaphoreHandle_t i2c_mutex, data_mutex;

Kalman1D kf_bme_temp, kf_bme_hum, kf_bme_pres, kf_aht_temp, kf_aht_hum, kf_altitude;
QuatSlerp qslerp;

// ════════════════════════════════════════════════════════════
//  I2C Sensorlar (BME280, AHT20, BNO055)
// ════════════════════════════════════════════════════════════
bool i2c_write(uint8_t addr, uint8_t reg, uint8_t data) { Wire.beginTransmission(addr); Wire.write(reg); Wire.write(data); return Wire.endTransmission() == 0; }
bool i2c_read_buf(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(addr); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, len) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

struct I2CHealth { uint8_t fails = 0; uint32_t next_try = 0; };
static inline bool i2c_should_try(const I2CHealth &h, uint32_t now) { return h.fails < 3 || (int32_t)(now - h.next_try) >= 0; }
static inline void i2c_mark_result(I2CHealth &h, bool ok, uint32_t now) {
  if (ok) { h.fails = 0; return; }
  if (h.fails < 255) h.fails++;
  if (h.fails >= 3) h.next_try = now + 1000;
}
static I2CHealth health_bno, health_bme, health_aht;

#define BME280_ADDR 0x76
struct { uint16_t T1; int16_t T2, T3; uint16_t P1; int16_t P2,P3,P4,P5,P6,P7,P8,P9; uint8_t H1,H3; int16_t H2,H4,H5; int8_t H6; } bme_cal;
int32_t bme_t_fine;
void bme280_init() { uint8_t buf[26]; i2c_read_buf(BME280_ADDR, 0x88, buf, 26);
  bme_cal.T1=buf[1]<<8|buf[0]; bme_cal.T2=buf[3]<<8|buf[2]; bme_cal.T3=buf[5]<<8|buf[4]; bme_cal.P1=buf[7]<<8|buf[6]; bme_cal.P2=buf[9]<<8|buf[8]; bme_cal.P3=buf[11]<<8|buf[10]; bme_cal.P4=buf[13]<<8|buf[12]; bme_cal.P5=buf[15]<<8|buf[14]; bme_cal.P6=buf[17]<<8|buf[16]; bme_cal.P7=buf[19]<<8|buf[18]; bme_cal.P8=buf[21]<<8|buf[20]; bme_cal.P9=buf[23]<<8|buf[22]; bme_cal.H1=buf[25]; uint8_t hb[7]; i2c_read_buf(BME280_ADDR,0xE1,hb,7);
  bme_cal.H2=hb[1]<<8|hb[0]; bme_cal.H3=hb[2]; bme_cal.H4=(int16_t)(hb[3]<<4)|(hb[4]&0x0F); bme_cal.H5=(int16_t)(hb[5]<<4)|(hb[4]>>4); bme_cal.H6=(int8_t)hb[6];
  i2c_write(BME280_ADDR,0xF2,0x01); i2c_write(BME280_ADDR,0xF4,0x27); i2c_write(BME280_ADDR,0xF5,0xA0);
}
bool bme280_read_temp(float &out) { uint8_t b[3]; if (!i2c_read_buf(BME280_ADDR,0xFA,b,3)) return false; int32_t raw=(b[0]<<12)|(b[1]<<4)|(b[2]>>4); int32_t v1=((((raw>>3)-((int32_t)bme_cal.T1<<1)))*bme_cal.T2)>>11; int32_t v2=(((((raw>>4)-(int32_t)bme_cal.T1)*((raw>>4)-(int32_t)bme_cal.T1))>>12)*bme_cal.T3)>>14; bme_t_fine=v1+v2; out=(bme_t_fine*5+128)/25600.0f; return true; }
bool bme280_read_pressure(float &out) { uint8_t b[3]; if (!i2c_read_buf(BME280_ADDR,0xF7,b,3)) return false; int32_t raw=(b[0]<<12)|(b[1]<<4)|(b[2]>>4); int64_t v1=(int64_t)bme_t_fine-128000; int64_t v2=v1*v1*bme_cal.P6; v2+=((v1*bme_cal.P5)<<17); v2+=((int64_t)bme_cal.P4<<35); v1=((v1*v1*bme_cal.P3)>>8)+((v1*bme_cal.P2)<<12); v1=(((int64_t)1<<47)+v1)*bme_cal.P1>>33; if(v1==0) { out=0; return true; } int64_t p=((int64_t)(1048576-raw)<<31)-v2; p=(p*3125)/v1; v1=((int64_t)bme_cal.P9*(p>>13)*(p>>13))>>25; v2=((int64_t)bme_cal.P8*p)>>19; out=((p+v1+v2)>>8)/25600.0f; return true; }
bool bme280_read_humidity(float &out) { uint8_t b[2]; if (!i2c_read_buf(BME280_ADDR,0xFD,b,2)) return false; int32_t raw=(b[0]<<8)|b[1]; int32_t v=bme_t_fine-76800; v=(((raw<<14)-((int32_t)bme_cal.H4<<20)-((int32_t)bme_cal.H5*v))+16384)>>15; v=v*(((((((v*bme_cal.H6)>>10)*(((v*bme_cal.H3)>>11)+32768))>>10)+2097152)*bme_cal.H2+8192)>>14); v=v-(((((v>>15)*(v>>15))>>7)*bme_cal.H1)>>4); v=constrain(v,0,419430400); out=(v>>12)/1024.0f; return true; }

#define AHT20_ADDR 0x38
void aht20_init() { delay(40); Wire.beginTransmission(AHT20_ADDR); Wire.write(0xBE); Wire.write(0x08); Wire.write(0x00); Wire.endTransmission(); delay(10); }
void aht20_trigger() { Wire.beginTransmission(AHT20_ADDR); Wire.write(0xAC); Wire.write(0x33); Wire.write(0x00); Wire.endTransmission(); }
bool aht20_read(float &temp, float &hum) { uint8_t buf[6]; if (Wire.requestFrom((uint8_t)AHT20_ADDR, (uint8_t)6) != 6) return false; for (uint8_t i=0; i<6; i++) buf[i]=Wire.read(); uint32_t hr=((uint32_t)buf[1]<<12)|((uint32_t)buf[2]<<4)|(buf[3]>>4); uint32_t tr=((uint32_t)(buf[3]&0x0F)<<16)|((uint32_t)buf[4]<<8)|buf[5]; hum = hr / 1048576.0f * 100.0f; temp = tr / 1048576.0f * 200.0f - 50.0f; return true; }

#define BNO055_ADDR 0x28
void bno055_init() { i2c_write(BNO055_ADDR,0x3D,0x00); delay(25); i2c_write(BNO055_ADDR,0x3E,0x00); i2c_write(BNO055_ADDR,0x3B,0x00); delay(10); i2c_write(BNO055_ADDR,0x3D,0x0C); delay(20); }
bool bno055_read_quat(float &qw, float &qx, float &qy, float &qz) { uint8_t buf[8]; if (!i2c_read_buf(BNO055_ADDR,0x20,buf,8)) return false; qw=(int16_t)(buf[1]<<8|buf[0])/16384.0f; qx=(int16_t)(buf[3]<<8|buf[2])/16384.0f; qy=(int16_t)(buf[5]<<8|buf[4])/16384.0f; qz=(int16_t)(buf[7]<<8|buf[6])/16384.0f; return true; }
bool bno055_read_accel(float &ax, float &ay, float &az) { uint8_t buf[6]; if (!i2c_read_buf(BNO055_ADDR,0x08,buf,6)) return false; ax=(int16_t)(buf[1]<<8|buf[0])/100.0f; ay=(int16_t)(buf[3]<<8|buf[2])/100.0f; az=(int16_t)(buf[5]<<8|buf[4])/100.0f; return true; }
bool bno055_read_euler(float &ex, float &ey, float &ez) { uint8_t buf[6]; if (!i2c_read_buf(BNO055_ADDR,0x1A,buf,6)) return false; ex=(int16_t)(buf[1]<<8|buf[0])/16.0f; ey=(int16_t)(buf[3]<<8|buf[2])/16.0f; ez=(int16_t)(buf[5]<<8|buf[4])/16.0f; return true; }

static float p_ref = 1013.25f;
float calc_altitude(float pres_hPa) { return 44330.0f * (1.0f - powf(pres_hPa / p_ref, 0.190284f)); }
void init_altitude_ref() {
  float sum = 0; int good = 0; const int N = 30;
  for (int i = 0; i < N; i++) { float t, p; if (bme280_read_temp(t) && bme280_read_pressure(p)) { sum += p; good++; } delay(50); }
  p_ref = good > 0 ? sum / good : p_ref;
}

// ════════════════════════════════════════════════════════════
//  GPS Parser
// ════════════════════════════════════════════════════════════
void parse_gps_line(char *line) {
  if (strncmp(line, "$GNGGA", 6) == 0 || strncmp(line, "$GPGGA", 6) == 0) {
    int field_idx = 0; char *p = line; char *token;
    float raw_lat = 0, raw_lon = 0, gps_alt = 0; char lat_dir = 'N', lon_dir = 'E'; int fix_quality = 0;
    while ((token = strsep(&p, ",")) != NULL) {
      if (field_idx == 6) fix_quality = atoi(token); else if (field_idx == 2) raw_lat = atof(token); else if (field_idx == 3) lat_dir = token[0]; else if (field_idx == 4) raw_lon = atof(token); else if (field_idx == 5) lon_dir = token[0]; else if (field_idx == 9) gps_alt = atof(token); field_idx++;
    }
    if (fix_quality > 0) {
      int deg_lat = (int)(raw_lat / 100); float min_lat = raw_lat - (deg_lat * 100); float lat = deg_lat + (min_lat / 60.0f); if (lat_dir == 'S') lat = -lat;
      int deg_lon = (int)(raw_lon / 100); float min_lon = raw_lon - (deg_lon * 100); float lon = deg_lon + (min_lon / 60.0f); if (lon_dir == 'W') lon = -lon;
      xSemaphoreTake(data_mutex, portMAX_DELAY); shared.gps_lat = lat; shared.gps_lon = lon; shared.gps_alt = gps_alt; shared.gps_fix = true; xSemaphoreGive(data_mutex);
    } else { xSemaphoreTake(data_mutex, portMAX_DELAY); shared.gps_fix = false; xSemaphoreGive(data_mutex); }
  }
}

// ════════════════════════════════════════════════════════════
//  CRC16 (RFD üçün)
// ════════════════════════════════════════════════════════════
static uint16_t crc16_table[256];
static void crc16_table_init() { for (uint16_t i = 0; i < 256; i++) { uint16_t crc = (uint16_t)(i << 8); for (uint8_t j = 0; j < 8; j++) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1); crc16_table[i] = crc; } }
static inline uint16_t calculate_crc16(const uint8_t* data, uint16_t length) { uint16_t crc = 0xFFFF; while (length--) crc = (uint16_t)((crc << 8) ^ crc16_table[((crc >> 8) ^ *data++) & 0xFF]); return crc; }

// ════════════════════════════════════════════════════════════
//  RFD900x Konfiqurasiyası
// ════════════════════════════════════════════════════════════
String rfd_send_at(const String &cmd, uint32_t timeout_ms = 500) {
  if (cmd.length() > 0) { Serial3.print(cmd); Serial3.print("\r\n"); }
  String resp; uint32_t start = millis();
  while (millis() - start < timeout_ms) { while (Serial3.available()) resp += (char)Serial3.read(); if (resp.indexOf("OK") != -1 || resp.indexOf("ERROR") != -1) break; delay(10); }
  return resp;
}
bool rfd_enter_at_mode() { delay(1100); Serial3.print("+++"); delay(1100); return rfd_send_at("", 500).indexOf("OK") != -1; }
bool rfd_set(const char* reg, const char* val) { String cmd = String(reg) + "=" + val; String resp = rfd_send_at(cmd, 400); return resp.indexOf("OK") != -1; }
bool rfd_configure_slave(uint8_t nodeID) {
  if (!rfd_enter_at_mode()) return false;
  bool ok = true;
  ok &= rfd_set("ATS1", "9"); ok &= rfd_set("ATS2", "64"); ok &= rfd_set("ATS3", "128"); ok &= rfd_set("ATS4", "1"); ok &= rfd_set("ATS9", "25");
  { char buf[8]; snprintf(buf, sizeof(buf), "%u", nodeID); ok &= rfd_set("ATS10", buf); }
  ok &= rfd_set("ATS11", "1"); ok &= rfd_set("ATS12", "24"); ok &= rfd_set("ATS14", "914500"); ok &= rfd_set("ATS15", "928000");
  ok &= rfd_set("ATS16", "23"); ok &= rfd_set("ATS17", "100"); ok &= rfd_set("ATS18", "0"); ok &= rfd_set("ATS19", "0"); ok &= rfd_set("ATS23", "1");
  if (ok) { rfd_send_at("AT&W", 500); rfd_send_at("ATZ", 2500); }
  Serial3.print("ATO\r\n"); delay(200);
  return ok;
}

// ════════════════════════════════════════════════════════════
//  TEST REJİMLƏRİ VƏ DƏYİŞƏNLƏR (4 km üçün)
// ════════════════════════════════════════════════════════════
enum TestMode : uint8_t { MODE_NORMAL = 0, MODE_SIT, MODE_SUT };
volatile TestMode currentMode = MODE_NORMAL;
volatile bool testRunning = false;
volatile bool synDataValid = false;
volatile TestMode pendingMode = MODE_NORMAL;
volatile uint32_t pendingStartTime = 0;

float synAlt = 0.0f, synAccX = 0.0f, synAccY = 0.0f, synAccZ = 0.0f;
float synAngX = 0.0f, synAngY = 0.0f, synAngZ = 0.0f;
volatile uint16_t statusBits = 0;

// ════════════════════════════════════════════════════════════
//  CHECKSUM (8-bit) – EK-7 (header daxil)
// ════════════════════════════════════════════════════════════
uint8_t calc_checksum(const uint8_t* data, uint8_t len) {
  uint16_t sum = 0;
  for (uint8_t i = 0; i < len; i++) sum += data[i];
  return (uint8_t)(sum & 0xFF);
}

// ════════════════════════════════════════════════════════════
//  SİT – Telemetriya Paketi (Cədvəl 3) – Header DAXİL checksum
// ════════════════════════════════════════════════════════════
void send_sit_telemetry() {
  SensorData snap;
  xSemaphoreTake(data_mutex, portMAX_DELAY);
  memcpy(&snap, &shared, sizeof(SensorData));
  xSemaphoreGive(data_mutex);

  uint8_t pkt[36];
  uint8_t idx = 0;
  pkt[idx++] = 0xAB;  // Header

  // EK-7: Big-Endian byte sıralaması (MSB first)
  auto append_float = [&](float val) {
    uint8_t* b = (uint8_t*)&val;
    for (int i = 3; i >= 0; i--) pkt[idx++] = b[i];
  };

  // EK-7: irtifa 0.0-10000.0 araliginda olmalidir, menfi olursa 0'a clamp et
  if (snap.altitude < 0.0f) snap.altitude = 0.0f;
  append_float(snap.altitude);
  append_float(snap.bme_pres);
  append_float(snap.accel_x);
  append_float(snap.accel_y);
  append_float(snap.accel_z);
  append_float(snap.angle_x);
  append_float(snap.angle_y);
  append_float(snap.angle_z);

  // idx = 33 (Header + 8*4)
  pkt[idx] = calc_checksum(pkt, idx);  // Header daxil 33 bayt
  idx++;
  pkt[idx++] = 0x0D;
  pkt[idx++] = 0x0A;

  // ---------- DEBUG: qısa SIT özeti ----------
  static uint32_t last_sit_debug = 0;
  if (millis() - last_sit_debug >= 1000) {  // saniyede 1 defe
    Serial1.print("[SIT] A:"); Serial1.print(snap.altitude, 1);
    Serial1.print(" P:"); Serial1.print(snap.bme_pres, 1);
    Serial1.print(" X:"); Serial1.print(snap.accel_x, 1);
    Serial1.print(" Y:"); Serial1.print(snap.accel_y, 1);
    Serial1.print(" Z:"); Serial1.print(snap.accel_z, 1);
    Serial1.print(" aX:"); Serial1.print(snap.angle_x, 1);
    Serial1.print(" aY:"); Serial1.print(snap.angle_y, 1);
    Serial1.print(" aZ:"); Serial1.println(snap.angle_z, 1);
    last_sit_debug = millis();
  }
  // ------------------------------------------

  if (Serial6.availableForWrite() >= idx) {
    Serial6.write(pkt, idx);
  }
}

// ════════════════════════════════════════════════════════════
//  SUT – Sintetik Data Parse (Cədvəl 4) – Header DAXİL checksum
// ════════════════════════════════════════════════════════════
void parse_synthetic_data(const uint8_t* buf, uint8_t len) {
  if (len < 36 || buf[0] != 0xAB) return;
  // Header (buf[0]) daxil 33 bayt (0..32) üzərindən checksum
  uint8_t calc = calc_checksum(buf, 33);
  if (calc != buf[33]) return; // 33-cü indeks Checksum

  // EK-7: Big Endian byte sıralaması ilə gəlir -> Little Endian float
  auto read_float = [&](uint8_t field_idx) {
    uint8_t tmp[4];
    const uint8_t* src = buf + 1 + field_idx * 4;
    tmp[0] = src[3]; tmp[1] = src[2]; tmp[2] = src[1]; tmp[3] = src[0]; // reverse
    float v; memcpy(&v, tmp, sizeof(float)); return v;
  };
  xSemaphoreTake(data_mutex, portMAX_DELAY);
  synAlt   = read_float(0);
  // field 1 – Pressure (istifadə etmirik)
  synAccX  = read_float(2);
  synAccY  = read_float(3);
  synAccZ  = read_float(4);
  synAngX  = read_float(5);
  synAngY  = read_float(6);
  synAngZ  = read_float(7);
  xSemaphoreGive(data_mutex);
  synDataValid = true;
}

// ════════════════════════════════════════════════════════════
//  SUT – Status Paketi (Cədvəl 6) – Header DAXİL checksum
// ════════════════════════════════════════════════════════════
void send_sut_status() {
  uint8_t pkt[6];
  pkt[0] = 0xAA;
  uint16_t bits;
  xSemaphoreTake(data_mutex, portMAX_DELAY);
  bits = statusBits;
  xSemaphoreGive(data_mutex);
  pkt[1] = (uint8_t)(bits & 0xFF);
  pkt[2] = (uint8_t)((bits >> 8) & 0xFF);
  
  // Header (pkt[0]) daxil 3 bayt
  pkt[3] = calc_checksum(pkt, 3);
  pkt[4] = 0x0D;
  pkt[5] = 0x0A;

  if (Serial6.availableForWrite() >= 6) {
    Serial6.write(pkt, 6);
  }
}

// ════════════════════════════════════════════════════════════
//  STATUS BİTLƏRİ – 4 km Apogey üçün OPTİMAL EŞİKLƏR
// ════════════════════════════════════════════════════════════
// Bit0,1,2,4,5,7 bir-dəfəlik hadisələrdir (kalkış, burnout, min-irtifa,
// apogey, drogue, main) — bir dəfə aktiv olandan sonra HƏMİŞƏ aktiv qalır
// (yalnız start_sut()/stop_test() sıfırlayana qədər). Əks halda ayrı-ayrı
// 10Hz tapşırıqlar (rs232_rx_task, test_tx_task) sinxron olmadığı üçün
// eyni "synAlt" nümunəsi iki dəfə oxuna bilər, "prevAlt > altitude" şərti
// bir anlıq yalan olar və bit "geri qapanar" (real testdə müşahidə edildi).
// Bit3 (bucaq xətası) və Bit6 (irtifa<500) isə canlı/davamlı göstəricilərdir.
void update_status_bits(float altitude, float accelZ, float angleX) {
  static float prevAlt = 0.0f;

  uint16_t bits;
  xSemaphoreTake(data_mutex, portMAX_DELAY);
  bits = statusBits;
  xSemaphoreGive(data_mutex);

  // Bir-dəfəlik hadisələr — yalnız əlavə olunur, heç vaxt silinmir
  if (altitude > 15.0f) bits |= (1 << 0);   // Kalkış
  if (altitude > 100.0f) bits |= (1 << 1);  // Burnout
  if (altitude > 250.0f) bits |= (1 << 2);  // Min irtifa eşiyi
  if (prevAlt > altitude && altitude > 50.0f) bits |= (1 << 4); // Apogey/enmə
  if ((bits & (1 << 4)) && altitude < 3900.0f) bits |= (1 << 5); // Drogue
  if ((bits & (1 << 5)) && altitude < 400.0f) bits |= (1 << 7);  // Main

  // Canlı göstəricilər — hər çağırışda yenidən qiymətləndirilir
  bits &= ~((1 << 3) | (1 << 6));
  if (angleX > 25.0f || fabs(accelZ) > 10.0f) bits |= (1 << 3);
  if (altitude < 500.0f) bits |= (1 << 6);

  prevAlt = altitude;

  xSemaphoreTake(data_mutex, portMAX_DELAY);
  statusBits = bits;
  xSemaphoreGive(data_mutex);
}

// ════════════════════════════════════════════════════════════
//  BUZZER (PE9) – Non-blocking tone()
// ════════════════════════════════════════════════════════════
void play_buzzer_tone(int frequency, int duration_ms) {
  tone(BUZZER_PIN, frequency, duration_ms);
}

// ════════════════════════════════════════════════════════════
//  TEST İDARƏ FUNKSİYALARI
// ════════════════════════════════════════════════════════════
void start_sit() {
  if (currentMode != MODE_NORMAL) return;
  // EK-7: non-blocking 1 sn gecikme – test_tx_task aktivlesdirir
  pendingMode = MODE_SIT; pendingStartTime = millis();
  Serial1.println("[SIT] Komut alindi, 1 sn sonra baslayacak...");
  play_buzzer_tone(400, 100);
}
void start_sut() {
  if (currentMode != MODE_NORMAL) return;
  xSemaphoreTake(data_mutex, portMAX_DELAY); statusBits = 0; xSemaphoreGive(data_mutex);
  pendingMode = MODE_SUT; pendingStartTime = millis();
  Serial1.println("[SUT] Komut alindi, 1 sn sonra baslayacak...");
  play_buzzer_tone(600, 150);
}
void stop_test() {
  if (!testRunning && pendingMode == MODE_NORMAL) return;
  currentMode = MODE_NORMAL; testRunning = false; synDataValid = false;
  pendingMode = MODE_NORMAL;
  xSemaphoreTake(data_mutex, portMAX_DELAY); statusBits = 0; xSemaphoreGive(data_mutex);
  Serial1.println("[TEST] Durduruldu.");
  play_buzzer_tone(200, 200);
}

// ════════════════════════════════════════════════════════════
//  RS232 RX TASK – 0xAA (əmrlər) və 0xAB (SUT dataları)
//  ✅ Hər iki halda checksum HEADER DAXİL
// ════════════════════════════════════════════════════════════
void rs232_rx_task(void*) {
  static uint8_t buf[64];
  static uint8_t idx = 0;
  static int raw_count = 0;
  for (;;) {
    while (Serial6.available()) {
      uint8_t c = Serial6.read();

      // ---------- DEBUG: qısa hex (10 baytdan bir sətir) ----------
      if (raw_count == 0) Serial1.print("[R] ");
      Serial1.print(c >> 4, HEX); Serial1.print(c & 0x0F, HEX); Serial1.print(' ');
      raw_count++;
      if (raw_count >= 10) { Serial1.println(); raw_count = 0; }
      // ------------------------------------------------------------
      static const char hex_digits[] = "0123456789ABCDEF"; // PKT-AA/AB çapı üçün

      if (idx == 0 && c != 0xAA && c != 0xAB) continue;
      buf[idx++] = c;

      // Əmr paketi (0xAA, 5 bayt) – Header DAXİL checksum (header+cmd)
      if (buf[0] == 0xAA && idx >= 5) {
        // Paketin tam şəklini də Serial1-ə yaz
        Serial1.print("\n[PKT-AA] ");
        for (uint8_t i = 0; i < 5; i++) { Serial1.print("0x"); Serial1.print(hex_digits[buf[i] >> 4]); Serial1.print(hex_digits[buf[i] & 0x0F]); Serial1.print(' '); }
        Serial1.println();
        raw_count = 0;

        if (buf[3] == 0x0D && buf[4] == 0x0A) {
          uint8_t cmd = buf[1];
          uint8_t calc_csum = calc_checksum(buf, 2); // Header (0xAA) + Cmd

          if (buf[2] == calc_csum) {
            Serial1.print("[CMD] Checksum OK – ");
            if (cmd == 0x20) { start_sit(); Serial1.println("SIT baslatildi"); }
            else if (cmd == 0x22) { start_sut(); Serial1.println("SUT baslatildi"); }
            else if (cmd == 0x24) { stop_test(); Serial1.println("STOP"); }
            else Serial1.println("bilinmeyen emr");
          } else {
            Serial1.print("[CMD] Checksum HATALI! alindi=0x"); Serial1.print(hex_digits[buf[2] >> 4]); Serial1.print(hex_digits[buf[2] & 0x0F]); Serial1.print(" beklenen=0x"); Serial1.print(hex_digits[calc_csum >> 4]); Serial1.println(hex_digits[calc_csum & 0x0F]);
          }
        } else {
          Serial1.println("[CMD] 0x0D 0x0A sonlanma YOK");
        }
        idx = 0; continue;
      }

      // ✅ SUT sintetik veri paketi (0xAB, 36 bayt) – Header daxil
      if (buf[0] == 0xAB && idx >= 36) {
        Serial1.print("\n[PKT-AB] ");
        for (uint8_t i = 0; i < 36; i++) { Serial1.print("0x"); Serial1.print(hex_digits[buf[i] >> 4]); Serial1.print(hex_digits[buf[i] & 0x0F]); Serial1.print(' '); }
        Serial1.println();
        raw_count = 0;

        if (buf[34] == 0x0D && buf[35] == 0x0A) {
          uint8_t calc = calc_checksum(buf, 33); // Header daxil
          if (calc == buf[33]) {
            parse_synthetic_data(buf, 36);
            Serial1.println("[DATA] SUT veri alindi, checksum OK");
          } else {
            Serial1.print("[DATA] Checksum HATALI! calc=0x"); Serial1.print(hex_digits[calc >> 4]); Serial1.print(hex_digits[calc & 0x0F]); Serial1.print(" pkt=0x"); Serial1.print(hex_digits[buf[33] >> 4]); Serial1.println(hex_digits[buf[33] & 0x0F]);
          }
        } else {
          Serial1.println("[DATA] 0x0D 0x0A sonlanma YOK");
        }
        idx = 0; continue;
      }

      if (idx >= sizeof(buf)) idx = 0;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ════════════════════════════════════════════════════════════
//  TEST TX TASK – 10 Hz, SİT/SUT göndərişi
// ════════════════════════════════════════════════════════════
void test_tx_task(void*) {
  TickType_t lastWake = xTaskGetTickCount();
  uint16_t prevStatus = 0;
  for (;;) {
    // EK-7: pending mode – 1 saniye sonra testi aktivlestir
    if (pendingMode != MODE_NORMAL && (millis() - pendingStartTime) >= 1000) {
      currentMode = pendingMode; testRunning = true; synDataValid = false;
      pendingMode = MODE_NORMAL;
      Serial1.print("[TEST] ");
      Serial1.println(currentMode == MODE_SIT ? "SIT baslatildi!" : "SUT baslatildi!");
    }

    if (testRunning) {
      if (currentMode == MODE_SIT) {
        send_sit_telemetry();
      } else if (currentMode == MODE_SUT) {
        if (synDataValid) {
          float snapAlt, snapAccZ, snapAngX;
          xSemaphoreTake(data_mutex, portMAX_DELAY);
          snapAlt = synAlt; snapAccZ = synAccZ; snapAngX = synAngX;
          xSemaphoreGive(data_mutex);
          update_status_bits(snapAlt, snapAccZ, snapAngX);
          uint16_t curStatus;
          xSemaphoreTake(data_mutex, portMAX_DELAY);
          curStatus = statusBits;
          xSemaphoreGive(data_mutex);

          // Buzzer – YALNIZ status dəyişdikdə
          if (curStatus != prevStatus) {
            if ((curStatus & (1<<0)) && !(prevStatus & (1<<0))) play_buzzer_tone(400, 200);
            if ((curStatus & (1<<4)) && !(prevStatus & (1<<4))) play_buzzer_tone(600, 300);
            if ((curStatus & (1<<7)) && !(prevStatus & (1<<7))) play_buzzer_tone(800, 500);
            prevStatus = curStatus;
          }
        }
        send_sut_status();
      }
    }
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(100)); // 10 Hz
  }
}

// ════════════════════════════════════════════════════════════
//  IMU TASK (BNO055) – 50 Hz
// ════════════════════════════════════════════════════════════
void imu_task(void*) {
  TickType_t t = xTaskGetTickCount();
  for (;;) {
    uint32_t now = millis();
    if (i2c_should_try(health_bno, now)) {
      float qw, qx, qy, qz, ax, ay, az, ex, ey, ez;
      xSemaphoreTake(i2c_mutex, portMAX_DELAY);
      bool okQ = bno055_read_quat(qw, qx, qy, qz);
      bool okA = bno055_read_accel(ax, ay, az);
      bool okE = bno055_read_euler(ex, ey, ez);
      xSemaphoreGive(i2c_mutex);
      i2c_mark_result(health_bno, okQ && okA && okE, now);
      if (okQ && okA && okE) {
        qslerp.update(qw, qx, qy, qz);
        xSemaphoreTake(data_mutex, portMAX_DELAY);
        shared.qw = qslerp.w; shared.qx = qslerp.x; shared.qy = qslerp.y; shared.qz = qslerp.z;
        shared.accel_x = ax; shared.accel_y = ay; shared.accel_z = az;
        shared.angle_x = ex; shared.angle_y = ey; shared.angle_z = ez;
        xSemaphoreGive(data_mutex);
      }
    }
    vTaskDelayUntil(&t, pdMS_TO_TICKS(20));
  }
}

// ════════════════════════════════════════════════════════════
//  ENV TASK – 5 Hz, SUT-da sintetik irtifa istifadə edir
// ════════════════════════════════════════════════════════════
void env_task(void*) {
  TickType_t t = xTaskGetTickCount();
  for (;;) {
    uint32_t now = millis();
    bool bme_ok = false; float bt=0, bp=0, bh=0;
    if (i2c_should_try(health_bme, now)) {
      xSemaphoreTake(i2c_mutex, portMAX_DELAY);
      bme_ok = bme280_read_temp(bt) && bme280_read_pressure(bp) && bme280_read_humidity(bh);
      xSemaphoreGive(i2c_mutex);
      i2c_mark_result(health_bme, bme_ok, now);
    }
    bool aht_try = i2c_should_try(health_aht, now);
    if (aht_try) { xSemaphoreTake(i2c_mutex, portMAX_DELAY); aht20_trigger(); xSemaphoreGive(i2c_mutex); }
    vTaskDelay(pdMS_TO_TICKS(80));
    bool aht_ok = false; float at=0, ah=0;
    if (aht_try) {
      xSemaphoreTake(i2c_mutex, portMAX_DELAY);
      aht_ok = aht20_read(at, ah);
      xSemaphoreGive(i2c_mutex);
      i2c_mark_result(health_aht, aht_ok, now);
    }

    if (currentMode == MODE_SUT && synDataValid) {
      xSemaphoreTake(data_mutex, portMAX_DELAY);
      shared.altitude = synAlt;
      xSemaphoreGive(data_mutex);
    } else {
      if (bme_ok) {
        float f_bt = kf_bme_temp.update(bt), f_bh = kf_bme_hum.update(bh), f_bp = kf_bme_pres.update(bp);
        float f_alt = kf_altitude.update(calc_altitude(f_bp));
        xSemaphoreTake(data_mutex, portMAX_DELAY);
        shared.bme_temp = f_bt; shared.bme_hum = f_bh; shared.bme_pres = f_bp; shared.altitude = f_alt;
        xSemaphoreGive(data_mutex);
      }
      if (aht_ok) {
        float f_at = kf_aht_temp.update(at), f_ah = kf_aht_hum.update(ah);
        xSemaphoreTake(data_mutex, portMAX_DELAY);
        shared.aht_temp = f_at; shared.aht_hum = f_ah;
        xSemaphoreGive(data_mutex);
      }
    }
    vTaskDelayUntil(&t, pdMS_TO_TICKS(200));
  }
}

// ════════════════════════════════════════════════════════════
//  GPS TASK
// ════════════════════════════════════════════════════════════
void gps_task(void*) {
  static char gps_buf[100]; static int buf_idx = 0;
  for (;;) {
    while (Serial2.available()) {
      char c = Serial2.read();
      if (c == '\n' || c == '\r') { if (buf_idx > 0) { gps_buf[buf_idx] = '\0'; parse_gps_line(gps_buf); buf_idx = 0; } } else if (buf_idx < 99) { gps_buf[buf_idx++] = c; }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ════════════════════════════════════════════════════════════
//  RFD TX TASK (20 Hz binar paket)
// ════════════════════════════════════════════════════════════
#pragma pack(push, 1)
struct BinaryTelemetry { uint8_t header; int16_t qw,qx,qy,qz; int16_t bme_temp; uint16_t bme_hum; float bme_pres; int16_t aht_temp; uint16_t aht_hum; int16_t altitude; int32_t gps_lat,gps_lon; int16_t gps_alt; uint8_t gps_fix; uint16_t crc; };
#pragma pack(pop)

void tx_task(void*) {
  TickType_t t = xTaskGetTickCount();
  BinaryTelemetry pkt;
  for (;;) {
    SensorData snap;
    xSemaphoreTake(data_mutex, portMAX_DELAY);
    memcpy(&snap, &shared, sizeof(SensorData));
    xSemaphoreGive(data_mutex);

    pkt.header = 17;
    pkt.qw = (int16_t)(snap.qw * 30000.0f); pkt.qx = (int16_t)(snap.qx * 30000.0f);
    pkt.qy = (int16_t)(snap.qy * 30000.0f); pkt.qz = (int16_t)(snap.qz * 30000.0f);
    pkt.bme_temp = (int16_t)(snap.bme_temp * 100.0f); pkt.bme_hum = (uint16_t)(snap.bme_hum * 100.0f);
    pkt.bme_pres = snap.bme_pres; pkt.aht_temp = (int16_t)(snap.aht_temp * 100.0f);
    pkt.aht_hum = (uint16_t)(snap.aht_hum * 100.0f); pkt.altitude = (int16_t)(snap.altitude * 10.0f);
    pkt.gps_lat = (int32_t)(snap.gps_lat * 10000000.0f); pkt.gps_lon = (int32_t)(snap.gps_lon * 10000000.0f);
    pkt.gps_alt = (int16_t)(snap.gps_alt * 10.0f); pkt.gps_fix = snap.gps_fix ? 1 : 0;
    pkt.crc = calculate_crc16((uint8_t*)&pkt, sizeof(BinaryTelemetry)-2);
    if (Serial3.availableForWrite() >= (int)sizeof(BinaryTelemetry)) Serial3.write((uint8_t*)&pkt, sizeof(BinaryTelemetry));
    vTaskDelayUntil(&t, pdMS_TO_TICKS(50));
  }
}

// ════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════
void setup() {
  Serial1.begin(115200);
  Serial2.begin(9600);
  Serial3.begin(9600);
  Serial6.begin(115200);  // RS232 test cihazı

  pinMode(BUZZER_PIN, OUTPUT);
  delay(1500);

  rfd_configure_slave(MY_ID);

  Wire.begin(); Wire.setClock(400000);
  crc16_table_init();
  bme280_init(); aht20_init(); bno055_init();

  init_altitude_ref();

  kf_bme_temp.init(0.005f, 0.25f); kf_bme_hum.init(0.05f, 9.0f); kf_bme_pres.init(0.05f, 1.0f);
  kf_altitude.init(0.1f, 4.0f); kf_aht_temp.init(0.005f, 0.09f); kf_aht_hum.init(0.05f, 4.0f);
  qslerp.init(0.15f);

  i2c_mutex = xSemaphoreCreateMutex();
  data_mutex = xSemaphoreCreateMutex();

  xTaskCreate(tx_task, "TX", 512, NULL, 4, NULL);
  xTaskCreate(imu_task, "IMU", 512, NULL, 3, NULL);
  xTaskCreate(gps_task, "GPS", 384, NULL, 2, NULL);
  xTaskCreate(env_task, "ENV", 512, NULL, 1, NULL);
  xTaskCreate(rs232_rx_task, "RS232_RX", 512, NULL, 3, NULL);
  xTaskCreate(test_tx_task, "TEST_TX", 512, NULL, 3, NULL);

  vTaskStartScheduler();
}

void loop() {}