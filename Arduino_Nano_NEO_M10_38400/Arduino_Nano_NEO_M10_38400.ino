/*
 * Hộp đo khí di động — Arduino Nano
 *
 * Thay các thư viện PMS, TinyGPS++, MQUnifiedsensor, DFRobot_ENS160,
 * DFRobot_AHT20 và uRTCLib. Những thư viện đó chiếm gần hết 2 KB RAM
 * của Nano nên mạch báo low memory và dễ treo. Sketch này chỉ dùng
 * SPI, SD, Wire và SoftwareSerial, đều có sẵn trong Arduino IDE.
 *
 * Chân đúng dây đang cắm:
 *   PMS nhận D2, PMS phát D3
 *   GPS nhận D6, GPS phát D5
 *   MQ2 A0, MQ135 A1
 *   Thẻ SD: CS D10, MOSI D11, MISO D12, SCK D13
 *   I2C A4 SDA, A5 SCL: DS3231, ENS160, AHT20
 *   D0 D1 để dành cho USB
 *
 * Cách chạy: nghe PMS 5 giây, nghe GPS 5 giây, cứ thế đổi.
 * MQ, ENS160 và AHT20 đọc mỗi 0,5 giây ở cả hai pha.
 * Cứ 30 giây ghi một dòng trung bình vào log/YYYYMMDD.csv.
 *
 * MQ2 tính ppm H2, MQ135 tính ppm CO, cùng hệ số với bản đang dùng.
 * Cột mq2_adc và mq135_adc vẫn là số ADC thô. Hiệu chuẩn R0 cần
 * không khí sạch lúc bật máy. Đo được R0 một lần thì điền vào
 * R0_MQ2_SAN và R0_MQ135_SAN để lần sau khỏi hiệu chuẩn lại.
 */

#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#define _SS_MAX_RX_BUFF 40
#include <SoftwareSerial.h>
#include <math.h>
#include <string.h>

const uint8_t CHAN_PMS_RX = 2;
const uint8_t CHAN_PMS_TX = 3;
const uint8_t CHAN_GPS_RX = 6;
const uint8_t CHAN_GPS_TX = 5;
const uint8_t CHAN_SD_CS = 10;
const uint8_t CHAN_MQ2 = A0;
const uint8_t CHAN_MQ135 = A1;

const unsigned long MS_PHA = 5000UL;
const unsigned long MS_TB = 30000UL;
const unsigned long MS_CAM = 500UL;
const long BAUD_SERIAL = 9600;
const long BAUD_PMS = 9600;
const long BAUD_GPS = 38400;

const uint8_t DIA_DS3231 = 0x68;
const uint8_t DIA_AHT = 0x38;
const float RL_KOHM = 10.0f;
const float DIEN_AP = 5.0f;

// Hệ số đường cong trong bản cũ. MQ2 là H2, MQ135 là CO.
const float MQ2_A = 987.99f;
const float MQ2_B = -2.675f;
const float MQ2_SACH = 9.83f;
const float MQ135_A = 605.18f;
const float MQ135_B = -3.937f;
const float MQ135_SACH = 3.6f;

// Điền R0 đã đo nếu không muốn hiệu chuẩn mỗi lần bật. Để 0 là tự đo.
const float R0_MQ2_SAN = 0.0f;
const float R0_MQ135_SAN = 0.0f;

const uint8_t LENH_PMS_THUC[] = {0x42, 0x4D, 0xE4, 0x00, 0x01, 0x01, 0x74};
const uint8_t LENH_PMS_CHAY[] = {0x42, 0x4D, 0xE1, 0x00, 0x01, 0x01, 0x71};

SoftwareSerial congPms(CHAN_PMS_RX, CHAN_PMS_TX);
SoftwareSerial congGps(CHAN_GPS_RX, CHAN_GPS_TX);

struct Tong {
  float lat;
  float lon;
  float toc;
  float alt;
  unsigned long ve;
  unsigned int nGps;
  unsigned int nVe;
  unsigned int nAlt;
  unsigned long pm1;
  unsigned long pm25;
  unsigned long pm10;
  unsigned int nPm;
  unsigned long mq2Adc;
  unsigned long mq135Adc;
  float mq2Ppm;
  float mq135Ppm;
  unsigned int nMq;
  unsigned int nPpm2;
  unsigned int nPpm135;
  unsigned long aqi;
  unsigned long tvoc;
  unsigned long eco2;
  unsigned int nEns;
  float nhiet;
  float am;
  unsigned int nKhi;
  uint8_t ensTrangThai;
  bool ensCo;
};

Tong tong;
char dongNmea[84];
uint8_t nmeaN = 0;
uint8_t pmsBuf[32];
uint8_t pmsN = 0;
bool dangPms = true;
bool pmsDaMo = false;
bool gpsDaMo = false;
bool sdOk = false;
bool daThuMuc = false;
bool daCanhGio = false;
bool coAht = false;
uint8_t ensDia = 0;
float r0Mq2 = 0;
float r0Mq135 = 0;

int gioNam, gioThang, gioNgay, gioGio, gioPhut, gioGiay;
bool coGioGps = false;

unsigned long mocPha = 0;
unsigned long mocTb = 0;
unsigned long mocCam = 0;

uint8_t tuBcd(uint8_t b) {
  return (uint8_t)((b >> 4) * 10 + (b & 0x0F));
}

uint8_t sangBcd(uint8_t v) {
  return (uint8_t)(((v / 10) << 4) | (v % 10));
}

bool i2cViet(uint8_t dia, uint8_t reg, const uint8_t* buf, uint8_t n) {
  Wire.beginTransmission(dia);
  Wire.write(reg);
  for (uint8_t i = 0; i < n; i++) Wire.write(buf[i]);
  return Wire.endTransmission() == 0;
}

bool i2cViet1(uint8_t dia, uint8_t reg, uint8_t giaTri) {
  return i2cViet(dia, reg, &giaTri, 1);
}

bool i2cDoc(uint8_t dia, uint8_t reg, uint8_t* buf, uint8_t n) {
  Wire.beginTransmission(dia);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  uint8_t m = Wire.requestFrom(dia, n);
  if (m != n) return false;
  for (uint8_t i = 0; i < n; i++) {
    if (!Wire.available()) return false;
    buf[i] = Wire.read();
  }
  return true;
}

void congBayGio(int& nam, int& thang, int& ngay, int& gio) {
  gio += 7;
  if (gio < 24) return;
  gio -= 24;
  static const uint8_t ngayThang[] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  uint8_t dim = ngayThang[thang];
  bool nhuan = (nam % 4 == 0 && nam % 100 != 0) || (nam % 400 == 0);
  if (thang == 2 && nhuan) dim = 29;
  ngay++;
  if (ngay > dim) {
    ngay = 1;
    thang++;
    if (thang > 12) {
      thang = 1;
      nam++;
    }
  }
}

bool docRtc(int& nam, int& thang, int& ngay, int& gio, int& phut, int& giay, uint8_t& trangThai) {
  uint8_t b[7];
  if (!i2cDoc(DIA_DS3231, 0x00, b, 7)) return false;
  giay = tuBcd(b[0] & 0x7F);
  phut = tuBcd(b[1] & 0x7F);
  gio = tuBcd(b[2] & 0x3F);
  ngay = tuBcd(b[4] & 0x3F);
  thang = tuBcd(b[5] & 0x1F);
  nam = 2000 + tuBcd(b[6]);
  uint8_t st;
  if (!i2cDoc(DIA_DS3231, 0x0F, &st, 1)) return false;
  trangThai = st;
  return true;
}

void ghiRtc(int nam, int thang, int ngay, int gio, int phut, int giay) {
  Wire.beginTransmission(DIA_DS3231);
  Wire.write((uint8_t)0x00);
  Wire.write(sangBcd(giay));
  Wire.write(sangBcd(phut));
  Wire.write(sangBcd(gio));
  Wire.write((uint8_t)0x01);
  Wire.write(sangBcd(ngay));
  Wire.write(sangBcd(thang));
  Wire.write(sangBcd(nam - 2000));
  Wire.endTransmission();
  i2cViet1(DIA_DS3231, 0x0F, 0x00);
}

void ensBatDau() {
  const uint8_t thu[] = {0x53, 0x52};
  for (uint8_t i = 0; i < 2; i++) {
    uint8_t id[2];
    if (!i2cDoc(thu[i], 0x00, id, 2)) continue;
    uint16_t part = (uint16_t)id[0] | ((uint16_t)id[1] << 8);
    if (part != 0x0160) continue;
    ensDia = thu[i];
    i2cViet1(ensDia, 0x10, 0xF0);
    delay(20);
    i2cViet1(ensDia, 0x10, 0x01);
    delay(20);
    i2cViet1(ensDia, 0x10, 0x02);
    Serial.print(F("ENS160 0x"));
    Serial.println(ensDia, HEX);
    return;
  }
  Serial.println(F("Khong thay ENS160"));
}

void ahtBatDau() {
  Wire.beginTransmission(DIA_AHT);
  Wire.write((uint8_t)0x71);
  if (Wire.endTransmission() != 0) {
    Serial.println(F("Khong thay AHT20"));
    return;
  }
  delay(10);
  if (Wire.requestFrom((uint8_t)DIA_AHT, (uint8_t)1) == 1) {
    uint8_t st = Wire.read();
    if ((st & 0x08) == 0) {
      Wire.beginTransmission(DIA_AHT);
      Wire.write((uint8_t)0xBE);
      Wire.write((uint8_t)0x08);
      Wire.write((uint8_t)0x00);
      Wire.endTransmission();
      delay(10);
    }
  }
  coAht = true;
  Serial.println(F("AHT20 san sang"));
}

void hutSerial() {
  if (dangPms) {
    int n = 0;
    while (congPms.available() && n < 160) {
      uint8_t b = congPms.read();
      n++;
      if (pmsN == 0) {
        if (b == 0x42) {
          pmsBuf[0] = b;
          pmsN = 1;
        }
        continue;
      }
      if (pmsN == 1) {
        if (b == 0x4D) {
          pmsBuf[1] = b;
          pmsN = 2;
        } else {
          pmsN = (b == 0x42) ? 1 : 0;
          if (pmsN == 1) pmsBuf[0] = 0x42;
        }
        continue;
      }
      pmsBuf[pmsN++] = b;
      if (pmsN < 32) continue;
      pmsN = 0;
      uint16_t sum = 0;
      for (uint8_t i = 0; i < 30; i++) sum += pmsBuf[i];
      uint16_t cs = ((uint16_t)pmsBuf[30] << 8) | pmsBuf[31];
      if (sum != cs) continue;
      tong.pm1 += ((uint16_t)pmsBuf[10] << 8) | pmsBuf[11];
      tong.pm25 += ((uint16_t)pmsBuf[12] << 8) | pmsBuf[13];
      tong.pm10 += ((uint16_t)pmsBuf[14] << 8) | pmsBuf[15];
      tong.nPm++;
    }
    return;
  }

  int n = 0;
  while (congGps.available() && n < 160) {
    char c = congGps.read();
    n++;
    if (c == '\r') continue;
    if (c == '\n') {
      dongNmea[nmeaN] = 0;
      if (nmeaN > 6) xuLyNmea(dongNmea);
      nmeaN = 0;
      continue;
    }
    if (nmeaN < sizeof(dongNmea) - 1) dongNmea[nmeaN++] = c;
    else nmeaN = 0;
  }
}

uint8_t chuHex(char c) {
  if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
  if (c >= 'A' && c <= 'F') return (uint8_t)(c - 'A' + 10);
  if (c >= 'a' && c <= 'f') return (uint8_t)(c - 'a' + 10);
  return 255;
}

bool kiemNmea(const char* s) {
  if (s[0] != '$') return false;
  const char* sao = strchr(s, '*');
  if (!sao) return false;
  uint8_t hi = chuHex(sao[1]);
  uint8_t lo = chuHex(sao[2]);
  if (hi == 255 || lo == 255) return false;
  uint8_t x = 0;
  for (const char* q = s + 1; q < sao; q++) x ^= (uint8_t)*q;
  return x == (uint8_t)((hi << 4) | lo);
}

float docSo(const char* s) {
  if (!s || !s[0]) return 0;
  bool am = false;
  if (*s == '-') {
    am = true;
    s++;
  }
  float v = 0;
  while (*s >= '0' && *s <= '9') {
    v = v * 10.0f + (float)(*s - '0');
    s++;
  }
  if (*s == '.') {
    float p = 0.1f;
    s++;
    while (*s >= '0' && *s <= '9') {
      v += (float)(*s - '0') * p;
      p *= 0.1f;
      s++;
    }
  }
  return am ? -v : v;
}

int docNguyen(const char* s) {
  int v = 0;
  if (!s) return 0;
  while (*s >= '0' && *s <= '9') {
    v = v * 10 + (*s - '0');
    s++;
  }
  return v;
}

float nmeaDo(const char* s, char huong) {
  float v = docSo(s);
  int deg = (int)(v / 100.0f);
  float phut = v - (float)deg * 100.0f;
  float d = (float)deg + phut / 60.0f;
  if (huong == 'S' || huong == 'W') d = -d;
  return d;
}

void layGioGps(const char* hhmmss, const char* ddmmyy) {
  if (strlen(hhmmss) < 6 || strlen(ddmmyy) < 6) return;
  int hh = (hhmmss[0] - '0') * 10 + (hhmmss[1] - '0');
  int mm = (hhmmss[2] - '0') * 10 + (hhmmss[3] - '0');
  int ss = (hhmmss[4] - '0') * 10 + (hhmmss[5] - '0');
  int dd = (ddmmyy[0] - '0') * 10 + (ddmmyy[1] - '0');
  int mo = (ddmmyy[2] - '0') * 10 + (ddmmyy[3] - '0');
  int yy = (ddmmyy[4] - '0') * 10 + (ddmmyy[5] - '0');
  int nam = 2000 + yy;
  congBayGio(nam, mo, dd, hh);
  gioNam = nam;
  gioThang = mo;
  gioNgay = dd;
  gioGio = hh;
  gioPhut = mm;
  gioGiay = ss;
  coGioGps = true;
}

void xuLyNmea(char* s) {
  if (!kiemNmea(s)) return;
  char* sao = strchr(s, '*');
  if (sao) *sao = 0;
  char* tach[16];
  uint8_t nTach = 0;
  char* p = s;
  if (*p == '$') p++;
  tach[nTach++] = p;
  while (*p && nTach < 16) {
    if (*p == ',') {
      *p = 0;
      tach[nTach++] = p + 1;
    }
    p++;
  }
  if (nTach < 8 || tach[0] == NULL) return;
  bool laRmc = strstr(tach[0], "RMC") != NULL;
  bool laGga = strstr(tach[0], "GGA") != NULL;
  if (laRmc && nTach >= 10 && tach[2][0] == 'A' && tach[3][0] && tach[5][0]) {
    tong.lat += nmeaDo(tach[3], tach[4][0]);
    tong.lon += nmeaDo(tach[5], tach[6][0]);
    tong.toc += docSo(tach[7]) * 1.852f;
    tong.nGps++;
    layGioGps(tach[1], tach[9]);
  } else if (laGga && nTach >= 10 && tach[6][0] && docNguyen(tach[6]) > 0) {
    tong.ve += (unsigned)docNguyen(tach[7]);
    tong.nVe++;
    if (tach[9][0]) {
      tong.alt += docSo(tach[9]);
      tong.nAlt++;
    }
  }
}

bool docAht(float* tC, float* rh) {
  if (!coAht) return false;
  Wire.beginTransmission(DIA_AHT);
  Wire.write((uint8_t)0xAC);
  Wire.write((uint8_t)0x33);
  Wire.write((uint8_t)0x00);
  if (Wire.endTransmission() != 0) return false;
  unsigned long t0 = millis();
  while (millis() - t0 < 90) hutSerial();
  if (Wire.requestFrom((uint8_t)DIA_AHT, (uint8_t)6) < 6) return false;
  uint8_t d[6];
  for (uint8_t i = 0; i < 6; i++) d[i] = Wire.read();
  if (d[0] & 0x80) return false;
  uint32_t h = ((uint32_t)d[1] << 12) | ((uint32_t)d[2] << 4) | (d[3] >> 4);
  uint32_t t = ((uint32_t)(d[3] & 0x0F) << 16) | ((uint32_t)d[4] << 8) | d[5];
  *rh = (h * 100.0f) / 1048576.0f;
  *tC = (t * 200.0f) / 1048576.0f - 50.0f;
  return true;
}

void ensBu(float tC, float rh) {
  if (!ensDia) return;
  float tK = tC + 273.15f;
  if (tK < 0) tK = 0;
  if (rh < 0) rh = 0;
  if (rh > 100) rh = 100;
  uint16_t tRaw = (uint16_t)(tK * 64.0f);
  uint16_t hRaw = (uint16_t)(rh * 512.0f);
  uint8_t buf[4];
  buf[0] = tRaw & 0xFF;
  buf[1] = tRaw >> 8;
  buf[2] = hRaw & 0xFF;
  buf[3] = hRaw >> 8;
  i2cViet(ensDia, 0x13, buf, 4);
}

void docEns() {
  if (!ensDia) return;
  uint8_t raw[6];
  if (!i2cDoc(ensDia, 0x20, raw, 6)) return;
  uint8_t validity = (raw[0] >> 2) & 0x03;
  if (!tong.ensCo || validity > tong.ensTrangThai) tong.ensTrangThai = validity;
  tong.ensCo = true;
  tong.aqi += raw[1] & 0x07;
  tong.tvoc += (unsigned long)raw[2] | ((unsigned long)raw[3] << 8);
  tong.eco2 += (unsigned long)raw[4] | ((unsigned long)raw[5] << 8);
  tong.nEns++;
}

float rsTuAdc(int adc) {
  if (adc < 8 || adc > 1015) return -1.0f;
  float v = DIEN_AP * (float)adc / 1023.0f;
  return (DIEN_AP * RL_KOHM) / v - RL_KOHM;
}

float hieuChuan(uint8_t pin, float tyLeSach) {
  float tongR = 0;
  uint8_t n = 0;
  for (uint8_t i = 0; i < 10; i++) {
    float rs = rsTuAdc(analogRead(pin));
    Serial.print('.');
    if (rs > 0) {
      tongR += rs / tyLeSach;
      n++;
    }
    delay(50);
  }
  Serial.println();
  if (n < 5) return 0;
  return tongR / n;
}

void inSo(Print& o, float v, uint8_t le) {
  if (v < 0) {
    o.print('-');
    v = -v;
  }
  float bu = 0.5f;
  for (uint8_t i = 0; i < le; i++) bu *= 0.1f;
  v += bu;
  unsigned long nguyen = (unsigned long)v;
  o.print(nguyen);
  if (!le) return;
  o.print('.');
  float lePhay = v - (float)nguyen;
  for (uint8_t i = 0; i < le; i++) {
    lePhay *= 10.0f;
    uint8_t chu = (uint8_t)lePhay;
    if (chu > 9) chu = 9;
    o.write((char)('0' + chu));
    lePhay -= chu;
  }
}

void chuanBiMq() {
  Serial.print(F("Hieu chuan MQ2"));
  r0Mq2 = (R0_MQ2_SAN > 0) ? R0_MQ2_SAN : hieuChuan(CHAN_MQ2, MQ2_SACH);
  Serial.print(F("R0 MQ2: "));
  if (r0Mq2 > 0) inSo(Serial, r0Mq2, 2);
  else Serial.print(F("loi, tam thoi khong ra ppm"));
  Serial.println();

  Serial.print(F("Hieu chuan MQ135"));
  r0Mq135 = (R0_MQ135_SAN > 0) ? R0_MQ135_SAN : hieuChuan(CHAN_MQ135, MQ135_SACH);
  Serial.print(F("R0 MQ135: "));
  if (r0Mq135 > 0) inSo(Serial, r0Mq135, 2);
  else Serial.print(F("loi, tam thoi khong ra ppm"));
  Serial.println();
}

void docMq() {
  int a2 = analogRead(CHAN_MQ2);
  int a135 = analogRead(CHAN_MQ135);
  tong.mq2Adc += (unsigned)a2;
  tong.mq135Adc += (unsigned)a135;
  tong.nMq++;
  float rs2 = rsTuAdc(a2);
  float rs135 = rsTuAdc(a135);
  if (r0Mq2 > 0 && rs2 > 0) {
    tong.mq2Ppm += MQ2_A * pow(rs2 / r0Mq2, MQ2_B);
    tong.nPpm2++;
  }
  if (r0Mq135 > 0 && rs135 > 0) {
    tong.mq135Ppm += MQ135_A * pow(rs135 / r0Mq135, MQ135_B);
    tong.nPpm135++;
  }
}

void docCamBien() {
  float tC = 0;
  float rh = 0;
  if (docAht(&tC, &rh)) {
    tong.nhiet += tC;
    tong.am += rh;
    tong.nKhi++;
    ensBu(tC, rh);
  }
  docEns();
  docMq();
}

void batPms() {
  if (gpsDaMo) {
    congGps.end();
    gpsDaMo = false;
  }
  dangPms = true;
  nmeaN = 0;
  pmsN = 0;
  congPms.begin(BAUD_PMS);
  congPms.listen();
  pmsDaMo = true;
  congPms.write(LENH_PMS_THUC, sizeof(LENH_PMS_THUC));
  congPms.write(LENH_PMS_CHAY, sizeof(LENH_PMS_CHAY));
}

void batGps() {
  if (pmsDaMo) {
    congPms.end();
    pmsDaMo = false;
  }
  dangPms = false;
  pmsN = 0;
  nmeaN = 0;
  congGps.begin(BAUD_GPS);
  congGps.listen();
  gpsDaMo = true;
}

void canhRtcNeuCan() {
  if (daCanhGio || !coGioGps) return;
  int nam, thang, ngay, gio, phut, giay;
  uint8_t st = 0;
  bool docDuoc = docRtc(nam, thang, ngay, gio, phut, giay, st);
  if (docDuoc && (st & 0x80) == 0 && nam >= 2024) {
    daCanhGio = true;
    return;
  }
  ghiRtc(gioNam, gioThang, gioNgay, gioGio, gioPhut, gioGiay);
  daCanhGio = true;
  Serial.println(F("Da dat DS3231 theo GPS, gio Ha Noi"));
}

void inDong(Print& o, const char* gio) {
  if (gio && gio[0]) o.print(gio);
  o.print(',');
  if (tong.nGps) inSo(o, tong.lat / tong.nGps, 6);
  o.print(',');
  if (tong.nGps) inSo(o, tong.lon / tong.nGps, 6);
  o.print(',');
  if (tong.nAlt) inSo(o, tong.alt / tong.nAlt, 1);
  o.print(',');
  if (tong.nGps) inSo(o, tong.toc / tong.nGps, 1);
  o.print(',');
  if (tong.nVe) o.print((tong.ve + tong.nVe / 2) / tong.nVe);
  o.print(',');
  if (tong.nPm) inSo(o, (float)tong.pm1 / tong.nPm, 1);
  o.print(',');
  if (tong.nPm) inSo(o, (float)tong.pm25 / tong.nPm, 1);
  o.print(',');
  if (tong.nPm) inSo(o, (float)tong.pm10 / tong.nPm, 1);
  o.print(',');
  if (tong.nMq) o.print(tong.mq2Adc / tong.nMq);
  o.print(',');
  if (tong.nPpm2) inSo(o, tong.mq2Ppm / tong.nPpm2, 2);
  o.print(',');
  if (tong.nMq) o.print(tong.mq135Adc / tong.nMq);
  o.print(',');
  if (tong.nPpm135) inSo(o, tong.mq135Ppm / tong.nPpm135, 2);
  o.print(',');
  if (tong.nEns) o.print((tong.aqi + tong.nEns / 2) / tong.nEns);
  o.print(',');
  if (tong.nEns) o.print(tong.tvoc / tong.nEns);
  o.print(',');
  if (tong.nEns) o.print(tong.eco2 / tong.nEns);
  o.print(',');
  if (tong.nKhi) inSo(o, tong.nhiet / tong.nKhi, 1);
  o.print(',');
  if (tong.nKhi) inSo(o, tong.am / tong.nKhi, 1);
  o.print(',');
  if (tong.ensCo) o.print(tong.ensTrangThai);
  o.println();
}

void viet2(char* b, int v) {
  b[0] = (char)('0' + (v / 10) % 10);
  b[1] = (char)('0' + v % 10);
}

void ghiThe() {
  canhRtcNeuCan();
  int nam = 0, thang = 0, ngay = 0, gio = 0, phut = 0, giay = 0;
  uint8_t st = 0;
  bool coGio = docRtc(nam, thang, ngay, gio, phut, giay, st);
  char bufGio[20];
  bufGio[0] = 0;
  if (coGio) {
    char* p = bufGio;
    viet2(p, nam / 100);
    p += 2;
    viet2(p, nam % 100);
    p += 2;
    *p++ = '-';
    viet2(p, thang);
    p += 2;
    *p++ = '-';
    viet2(p, ngay);
    p += 2;
    *p++ = ' ';
    viet2(p, gio);
    p += 2;
    *p++ = ':';
    viet2(p, phut);
    p += 2;
    *p++ = ':';
    viet2(p, giay);
    p += 2;
    *p = 0;
  }
  inDong(Serial, bufGio);
  if (!sdOk) sdOk = SD.begin(CHAN_SD_CS);
  if (!sdOk) {
    Serial.println(F("Loi the SD"));
    return;
  }
  char ten[17];
  ten[0] = 'l';
  ten[1] = 'o';
  ten[2] = 'g';
  ten[3] = '/';
  if (coGio && nam >= 2024) {
    viet2(ten + 4, nam / 100);
    viet2(ten + 6, nam % 100);
    viet2(ten + 8, thang);
    viet2(ten + 10, ngay);
    ten[12] = '.';
    ten[13] = 'c';
    ten[14] = 's';
    ten[15] = 'v';
    ten[16] = 0;
  } else {
    ten[4] = 'n';
    ten[5] = 'o';
    ten[6] = 'd';
    ten[7] = 'a';
    ten[8] = 't';
    ten[9] = 'e';
    ten[10] = '.';
    ten[11] = 'c';
    ten[12] = 's';
    ten[13] = 'v';
    ten[14] = 0;
  }
  if (!daThuMuc) {
    if (!SD.exists("log")) SD.mkdir("log");
    daThuMuc = true;
  }
  File f = SD.open(ten, FILE_WRITE);
  if (!f) {
    Serial.println(F("Khong mo duoc file CSV"));
    return;
  }
  if (f.size() == 0) {
    f.println(F("thoi_gian,vi_do,kinh_do,cao_do_m,toc_do_kmh,ve_tinh,pm1_ugm3,pm25_ugm3,pm10_ugm3,mq2_adc,mq2_ppm,mq135_adc,mq135_ppm,aqi_uba,tvoc_ppb,eco2_ppm,nhiet_do_c,do_am_pct,ens_trang_thai"));
  }
  inDong(f, bufGio);
  f.close();
}

// Doc tat ca file CSV truc tiep trong thu muc log/ qua USB Serial.
// Trong luc export, loop() tam dung; cac file tren SD khong bi thay doi.
void xuatTatCaCsv() {
  Serial.println(F("===EXPORT_START==="));
  Serial.flush();

  if (!sdOk) sdOk = SD.begin(CHAN_SD_CS);
  if (!sdOk) {
    Serial.println(F("===EXPORT_ERROR:SD_INIT_FAILED==="));
    Serial.println(F("===EXPORT_FINISHED==="));
    Serial.flush();
    return;
  }

  File thuMuc = SD.open("log");
  if (!thuMuc || !thuMuc.isDirectory()) {
    if (thuMuc) thuMuc.close();
    Serial.println(F("===EXPORT_ERROR:LOG_FOLDER_NOT_FOUND==="));
    Serial.println(F("===EXPORT_FINISHED==="));
    Serial.flush();
    return;
  }

  uint16_t soFile = 0;
  File muc = thuMuc.openNextFile();
  while (muc) {
    if (!muc.isDirectory()) {
      const char* tenFile = muc.name();
      const char* duoi = strrchr(tenFile, '.');
      bool laCsv = duoi && strlen(duoi) == 4 &&
                  (duoi[1] == 'c' || duoi[1] == 'C') &&
                  (duoi[2] == 's' || duoi[2] == 'S') &&
                  (duoi[3] == 'v' || duoi[3] == 'V');

      if (laCsv) {
        Serial.print(F("===FILE:"));
        Serial.print(tenFile);
        Serial.println(F("==="));
        Serial.flush();

        int kyTuCuoi = '\n';
        while (muc.available()) {
          int c = muc.read();
          if (c < 0) break;
          Serial.write((uint8_t)c);
          kyTuCuoi = c;
        }
        // Dam bao marker FILE_END luon bat dau tren mot dong rieng.
        if (kyTuCuoi != '\n') Serial.println();
        Serial.println(F("===FILE_END==="));
        Serial.flush();
        soFile++;
      }
    }
    muc.close();
    muc = thuMuc.openNextFile();
  }

  thuMuc.close();
  Serial.print(F("===EXPORT_FILE_COUNT:"));
  Serial.print(soFile);
  Serial.println(F("==="));
  Serial.println(F("===EXPORT_FINISHED==="));
  Serial.flush();
}

// Nhan lenh mot dong tu Serial Monitor/Python, vi du: EXPORT\n
void kiemTraLenhSerial() {
  static char lenh[16];
  static uint8_t nLenh = 0;

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      lenh[nLenh] = 0;
      if (strcmp(lenh, "EXPORT") == 0) {
        xuatTatCaCsv();
      } else if (nLenh > 0) {
        Serial.println(F("Lenh khong hop le. Gui EXPORT de xuat CSV."));
      }
      nLenh = 0;
    } else if (nLenh < sizeof(lenh) - 1) {
      lenh[nLenh++] = c;
    } else {
      nLenh = 0;
    }
  }
}

void setup() {
  Serial.begin(BAUD_SERIAL);
  delay(200);
  Serial.println(F("Hop do Nano - khong thu vien ngoai"));
  Wire.begin();
  ensBatDau();
  ahtBatDau();
  chuanBiMq();

  int nam, thang, ngay, gio, phut, giay;
  uint8_t st = 0;
  if (docRtc(nam, thang, ngay, gio, phut, giay, st)) {
    Serial.print(F("DS3231 "));
    Serial.println(nam);
  } else {
    Serial.println(F("Khong thay DS3231"));
  }

  sdOk = SD.begin(CHAN_SD_CS);
  if (!sdOk) Serial.println(F("Chua doc duoc the SD, se thu lai"));
  else Serial.println(F("The SD san sang"));

  memset(&tong, 0, sizeof(tong));
  mocPha = millis();
  mocTb = mocPha;
  mocCam = mocPha;
  batPms();
  Serial.println(F("Bat dau: 5s PMS, 5s GPS, ghi moi 30s"));
}

void loop() {
  kiemTraLenhSerial();
  unsigned long now = millis();
  if (now - mocPha >= MS_PHA) {
    mocPha = now;
    if (dangPms) batGps();
    else batPms();
  }
  hutSerial();
  if (now - mocCam >= MS_CAM) {
    mocCam = now;
    docCamBien();
  }
  if (now - mocTb >= MS_TB) {
    ghiThe();
    memset(&tong, 0, sizeof(tong));
    coGioGps = false;
    mocTb = millis();
  }
}
