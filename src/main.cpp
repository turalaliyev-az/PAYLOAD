#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <STM32FreeRTOS.h>

// ════════════════════════════════════════════════════════════
//  UART Port  & Pin Bağlantıları
// ════════════════════════════════════════════════════════════
HardwareSerial Serial1(PA10, PA9); // Debug / PC bağlantısı
HardwareSerial Serial2(PA3, PA2);  // GPS Modulu üçün
HardwareSerial Serial3(PB11, PB10);// RFD900x Modulu üçün

// Şəbəkə ID-ləri
const uint8_t MY_ID = 23;
const uint8_t GROUND_STATION_ID = 13;

// ════════════════════════════════════════════════════════════
//  Strukturlar (Kalman, Slerp və Data)
// ══════════════════════════════════════════════════════
struct Kalman1D {
  float Q, R, x, P;
  bool  ready = false;
  void init(float q, float r) { Q=q; R=r; P=1.0f; }
  float update(float z) {
    if (!ready) { x = z; ready = true; return x; }
    P += Q;
    const float K = P / (P + R);
    x += K * (z - x);
    P  = (1.0f - K) * P;
    return x;
  }
};

struct QuatSlerp {
  float w=1, x=0, y=0, z=0;
  float alpha;
  bool  ready = false;
  void init(float a) { alpha = a; }
  void update(float nw, float nx, float ny, float nz) {
    if (!ready) { w=nw; x=nx; y=ny; z=nz; ready=true; return; }
    float dot = w*nw + x*nx + y*ny + z*nz;
    if (dot < 0.0f) { nw=-nw; nx=-nx; ny=-ny; nz=-nz; dot=-dot; }
    float t0, t1;
    if (dot < 0.9999f) {
      const float theta = acosf(dot);
      const float s     = sinf(theta);
      t0 = sinf((1.0f - alpha) * theta) / s;
      t1 = sinf(alpha * theta) / s;
    } else { t0 = 1.0f - alpha; t1 = alpha; }
    w = t0*w + t1*nw; x = t0*x + t1*nx; y = t0*y + t1*ny; z = t0*z + t1*nz;
    const float n = sqrtf(w*w + x*x + y*y + z*z);
    if (n > 0.0f) { w/=n; x/=n; y/=n; z/=n; }
  }
};

struct SensorData {
  float qw=1, qx=0, qy=0, qz=0;
  float bme_temp=0, bme_hum=0, bme_pres=0;
  float aht_temp=0, aht_hum=0;
  float altitude=0;
  float gps_lat=0.0f;
  float gps_lon=0.0f;
  float gps_alt=0.0f;
  bool  gps_fix=false;
};

// ════════════════════════════════════════════════════════════
//  36 Baytlıq Binar Paket Formatı
// ════════════════════════════════════════════════════════════
#pragma pack(push, 1) 
struct BinaryTelemetry {
  uint8_t  header;       // 1 bayt: Paket başlanğıcı (Həmişə 17)
  
  int16_t  qw;           // 2 bayt: * 30000
  int16_t  qx;           // 2 bayt: * 30000
  int16_t  qy;           // 2 bayt: * 30000
  int16_t  qz;           // 2 bayt: * 30000
  
  int16_t  bme_temp;     // 2 bayt: * 100 (məs. 25.34C -> 2534)
  uint16_t bme_hum;      // 2 bayt: * 100 (məs. 45.12% -> 4512)
  float    bme_pres;     // 4 bayt: Xam float formatında (hPa)
  
  int16_t  aht_temp;     // 2 bayt: * 100
  uint16_t aht_hum;      // 2 bayt: * 100
  
  int16_t  altitude;     // 2 bayt: * 10  (məs. 120.5m -> 1205)
  
  int32_t  gps_lat;      // 4 bayt: * 10^7 (Dərəcə)
  int32_t  gps_lon;      // 4 bayt: * 10^7 (Dərəcə)
  int16_t  gps_alt;      // 2 bayt: * 10  (metr)
  uint8_t  gps_fix;      // 1 bayt: 0 (Yoxdur) / 1 (Peyk tutdu)
  
  uint16_t crc;          // 2 bayt: CRC16 Yoxlanışı (Sondakı imza)
};
#pragma pack(pop)

SensorData shared;
SemaphoreHandle_t i2c_mutex;
SemaphoreHandle_t data_mutex;

Kalman1D kf_bme_temp, kf_bme_hum, kf_bme_pres, kf_aht_temp, kf_aht_hum, kf_altitude;
QuatSlerp qslerp;

// ════════════════════════════════════════════════════════════
//  I2C Sensor Funksiyaları
// ════════════════════════════════════════════════════════════
void i2c_write(uint8_t addr, uint8_t reg, uint8_t data) { Wire.beginTransmission(addr); Wire.write(reg); Wire.write(data); Wire.endTransmission(); }
void i2c_read_buf(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len) { Wire.beginTransmission(addr); Wire.write(reg); Wire.endTransmission(false); Wire.requestFrom(addr, len); for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read(); }

#define BME280_ADDR 0x76
struct { uint16_t T1; int16_t T2, T3; uint16_t P1; int16_t P2,P3,P4,P5,P6,P7,P8,P9; uint8_t H1,H3; int16_t H2,H4,H5; int8_t H6; } bme_cal;
int32_t bme_t_fine;

void bme280_init() {
  uint8_t buf[26]; i2c_read_buf(BME280_ADDR, 0x88, buf, 26);
  bme_cal.T1=buf[1]<<8|buf[0]; bme_cal.T2=buf[3]<<8|buf[2]; bme_cal.T3=buf[5]<<8|buf[4]; bme_cal.P1=buf[7]<<8|buf[6]; bme_cal.P2=buf[9]<<8|buf[8]; bme_cal.P3=buf[11]<<8|buf[10]; bme_cal.P4=buf[13]<<8|buf[12]; bme_cal.P5=buf[15]<<8|buf[14]; bme_cal.P6=buf[17]<<8|buf[16]; bme_cal.P7=buf[19]<<8|buf[18]; bme_cal.P8=buf[21]<<8|buf[20]; bme_cal.P9=buf[23]<<8|buf[22]; bme_cal.H1=buf[25]; uint8_t hb[7]; i2c_read_buf(BME280_ADDR,0xE1,hb,7);
  bme_cal.H2=hb[1]<<8|hb[0]; bme_cal.H3=hb[2]; bme_cal.H4=(int16_t)(hb[3]<<4)|(hb[4]&0x0F); bme_cal.H5=(int16_t)(hb[5]<<4)|(hb[4]>>4); bme_cal.H6=(int8_t)hb[6];
  i2c_write(BME280_ADDR,0xF2,0x01); i2c_write(BME280_ADDR,0xF4,0x27); i2c_write(BME280_ADDR,0xF5,0xA0);
}
float bme280_read_temp() {
  uint8_t b[3]; i2c_read_buf(BME280_ADDR,0xFA,b,3); int32_t raw=(b[0]<<12)|(b[1]<<4)|(b[2]>>4); int32_t v1=((((raw>>3)-((int32_t)bme_cal.T1<<1)))*bme_cal.T2)>>11; int32_t v2=(((((raw>>4)-(int32_t)bme_cal.T1)*((raw>>4)-(int32_t)bme_cal.T1))>>12)*bme_cal.T3)>>14; bme_t_fine=v1+v2; return (bme_t_fine*5+128)/25600.0f;
}
float bme280_read_pressure() {
  uint8_t b[3]; i2c_read_buf(BME280_ADDR,0xF7,b,3); int32_t raw=(b[0]<<12)|(b[1]<<4)|(b[2]>>4); int64_t v1=(int64_t)bme_t_fine-128000; int64_t v2=v1*v1*bme_cal.P6; v2+=((v1*bme_cal.P5)<<17); v2+=((int64_t)bme_cal.P4<<35); v1=((v1*v1*bme_cal.P3)>>8)+((v1*bme_cal.P2)<<12); v1=(((int64_t)1<<47)+v1)*bme_cal.P1>>33; if(v1==0) return 0;
  int64_t p=((int64_t)(1048576-raw)<<31)-v2; p=(p*3125)/v1; v1=((int64_t)bme_cal.P9*(p>>13)*(p>>13))>>25; v2=((int64_t)bme_cal.P8*p)>>19; return ((p+v1+v2)>>8)/25600.0f;
}
float bme280_read_humidity() {
  uint8_t b[2]; i2c_read_buf(BME280_ADDR,0xFD,b,2); int32_t raw=(b[0]<<8)|b[1]; int32_t v=bme_t_fine-76800; v=(((raw<<14)-((int32_t)bme_cal.H4<<20)-((int32_t)bme_cal.H5*v))+16384)>>15; v=v*(((((((v*bme_cal.H6)>>10)*(((v*bme_cal.H3)>>11)+32768))>>10)+2097152)*bme_cal.H2+8192)>>14); v=v-(((((v>>15)*(v>>15))>>7)*bme_cal.H1)>>4); v=constrain(v,0,419430400); return (v>>12)/1024.0f;
}

#define AHT20_ADDR 0x38
void aht20_init() { delay(40); Wire.beginTransmission(AHT20_ADDR); Wire.write(0xBE); Wire.write(0x08); Wire.write(0x00); Wire.endTransmission(); delay(10); }
void aht20_trigger() { Wire.beginTransmission(AHT20_ADDR); Wire.write(0xAC); Wire.write(0x33); Wire.write(0x00); Wire.endTransmission(); }
void aht20_read(float &temp, float &hum) { uint8_t buf[6]; Wire.requestFrom((uint8_t)AHT20_ADDR, (uint8_t)6); for (uint8_t i=0; i<6; i++) buf[i]=Wire.read(); uint32_t hr=((uint32_t)buf[1]<<12)|((uint32_t)buf[2]<<4)|(buf[3]>>4); uint32_t tr=((uint32_t)(buf[3]&0x0F)<<16)|((uint32_t)buf[4]<<8)|buf[5]; hum = hr / 1048576.0f * 100.0f; temp = tr / 1048576.0f * 200.0f - 50.0f; }

#define BNO055_ADDR 0x28
void bno055_init() { i2c_write(BNO055_ADDR,0x3D,0x00); delay(25); i2c_write(BNO055_ADDR,0x3E,0x00); i2c_write(BNO055_ADDR,0x3B,0x00); delay(10); i2c_write(BNO055_ADDR,0x3D,0x0C); delay(20); }
void bno055_read_quat(float &qw, float &qx, float &qy, float &qz) { uint8_t buf[8]; i2c_read_buf(BNO055_ADDR,0x20,buf,8); qw=(int16_t)(buf[1]<<8|buf[0])/16384.0f; qx=(int16_t)(buf[3]<<8|buf[2])/16384.0f; qy=(int16_t)(buf[5]<<8|buf[4])/16384.0f; qz=(int16_t)(buf[7]<<8|buf[6])/16384.0f; }

static float p_ref = 1013.25f;
float calc_altitude(float pres_hPa) { return 44330.0f * (1.0f - powf(pres_hPa / p_ref, 0.190284f)); }
void init_altitude_ref() { float sum = 0; const int N = 30; for (int i = 0; i < N; i++) { bme280_read_temp(); sum += bme280_read_pressure(); delay(50); } p_ref = sum / N; Serial1.print("P_ref: "); Serial1.print(p_ref, 2); Serial1.println(" hPa"); }

// ════════════════════════════════════════════════════════════
//   NMEA GPS Parser Funksiyası
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
    } else {
      xSemaphoreTake(data_mutex, portMAX_DELAY); shared.gps_fix = false; xSemaphoreGive(data_mutex);
    }
  }
}

// ════════════════════════════════════════════════════════════
//  Dözümlü Rabitə üçün CRC16-CCITT Funksiyası
// ════════════════════════════════════════════════════════════
static uint16_t calculate_crc16(const uint8_t* data, uint16_t length) {
  uint16_t crc = 0xFFFF;
  while (length--) {
    crc ^= (uint16_t)(*data++) << 8;
    for (int i = 0; i < 8; i++) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else crc = (crc << 1);
    }
  }
  return crc;
}

// ════════════════════════════════════════════════════════════
//  TASKS
// ════════════════════════════════════════════════════════════
void imu_task(void*) {
  TickType_t t = xTaskGetTickCount();
  for (;;) {
    float qw, qx, qy, qz; xSemaphoreTake(i2c_mutex, portMAX_DELAY); bno055_read_quat(qw, qx, qy, qz); xSemaphoreGive(i2c_mutex);
    qslerp.update(qw, qx, qy, qz);
    xSemaphoreTake(data_mutex, portMAX_DELAY); shared.qw = qslerp.w; shared.qx = qslerp.x; shared.qy = qslerp.y; shared.qz = qslerp.z; xSemaphoreGive(data_mutex);
    vTaskDelayUntil(&t, pdMS_TO_TICKS(20));
  }
}

void env_task(void*) {
  TickType_t t = xTaskGetTickCount();
  for (;;) {
    xSemaphoreTake(i2c_mutex, portMAX_DELAY); float bt = bme280_read_temp(); float bp = bme280_read_pressure(); float bh = bme280_read_humidity(); xSemaphoreGive(i2c_mutex);
    xSemaphoreTake(i2c_mutex, portMAX_DELAY); aht20_trigger(); xSemaphoreGive(i2c_mutex);
    vTaskDelay(pdMS_TO_TICKS(80));
    xSemaphoreTake(i2c_mutex, portMAX_DELAY); float at, ah; aht20_read(at, ah); xSemaphoreGive(i2c_mutex);
    float f_bt = kf_bme_temp.update(bt); float f_bh = kf_bme_hum.update(bh); float f_bp = kf_bme_pres.update(bp); float f_at = kf_aht_temp.update(at); float f_ah = kf_aht_hum.update(ah);
    float raw_alt = calc_altitude(f_bp); float f_alt = kf_altitude.update(raw_alt);
    xSemaphoreTake(data_mutex, portMAX_DELAY); shared.bme_temp = f_bt; shared.bme_hum = f_bh; shared.bme_pres = f_bp; shared.aht_temp = f_at; shared.aht_hum = f_ah; shared.altitude = f_alt; xSemaphoreGive(data_mutex);
    vTaskDelayUntil(&t, pdMS_TO_TICKS(200));
  }
}

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
//  TASK 4 — TX (Binar Təhlükəsiz Ötürmə - 50Hz)
// ════════════════════════════════════════════════════════════
void tx_task(void*) {
  TickType_t t = xTaskGetTickCount();
  BinaryTelemetry pkt;

  for (;;) {
    SensorData snap;
    xSemaphoreTake(data_mutex, portMAX_DELAY);
    memcpy(&snap, (const void*)&shared, sizeof(SensorData));
    xSemaphoreGive(data_mutex);

    Serial1.print(snap.qw, 4); Serial1.print(','); Serial1.print(snap.bme_temp, 2); Serial1.print(','); Serial1.print(snap.altitude, 1); Serial1.print(','); Serial1.println((int)snap.gps_fix);

    pkt.header   = 23; 
    
    pkt.qw       = (int16_t)(snap.qw * 30000.0f);
    pkt.qx       = (int16_t)(snap.qx * 30000.0f);
    pkt.qy       = (int16_t)(snap.qy * 30000.0f);
    pkt.qz       = (int16_t)(snap.qz * 30000.0f);
    
    pkt.bme_temp = (int16_t)(snap.bme_temp * 100.0f);
    pkt.bme_hum  = (uint16_t)(snap.bme_hum * 100.0f);
    pkt.bme_pres = snap.bme_pres; 
    
    pkt.aht_temp = (int16_t)(snap.aht_temp * 100.0f);
    pkt.aht_hum  = (uint16_t)(snap.aht_hum * 100.0f);
    
    pkt.altitude = (int16_t)(snap.altitude * 10.0f);
    
    pkt.gps_lat  = (int32_t)(snap.gps_lat * 10000000.0f);
    pkt.gps_lon  = (int32_t)(snap.gps_lon * 10000000.0f);
    pkt.gps_alt  = (int16_t)(snap.gps_alt * 10.0f);
    pkt.gps_fix  = snap.gps_fix ? 1 : 0;

    pkt.crc = calculate_crc16((uint8_t*)&pkt, sizeof(BinaryTelemetry) - 2);

    Serial3.write((uint8_t*)&pkt, sizeof(BinaryTelemetry));

    vTaskDelayUntil(&t, pdMS_TO_TICKS(20));
  }
}

// ════════════════════════════════════════════════════════════
//  Setup & Loop
// ════════════════════════════════════════════════════════════
void setup() {
  Serial1.begin(115200); 
  Serial2.begin(9600);   
  Serial3.begin(115200); 
  
  delay(1500);
  Wire.begin();
  Wire.setClock(400000);

  bme280_init(); aht20_init(); bno055_init();

  Serial1.println("P_ref olcunur..."); init_altitude_ref(); Serial1.println("Sistem Hazirdir.");

  kf_bme_temp.init(0.005f, 0.25f); kf_bme_hum.init(0.05f, 9.0f); kf_bme_pres.init(0.05f, 1.0f);
  kf_altitude.init(0.1f, 4.0f); kf_aht_temp.init(0.005f, 0.09f); kf_aht_hum.init(0.05f, 4.0f); qslerp.init(0.15f);

  i2c_mutex  = xSemaphoreCreateMutex(); data_mutex = xSemaphoreCreateMutex();

  xTaskCreate(imu_task, "IMU", 512, NULL, 4, NULL);
  xTaskCreate(tx_task,  "TX",  512, NULL, 3, NULL);
  xTaskCreate(gps_task, "GPS", 384, NULL, 2, NULL);
  xTaskCreate(env_task, "ENV", 512, NULL, 1, NULL); 

  vTaskStartScheduler();
}

void loop() {}