#include <Wire.h>
#include <TinyGPSPlus.h>
#include <math.h>

#include <U8g2lib.h>
#include <SPI.h>

TinyGPSPlus gps;

// --------- GPS UART ---------
static const uint32_t GPS_BAUD = 38400;
static const uint32_t NMEA_BAUD = 115200;

// --------- OLED (SSD1309 2.42" SPI) ---------
#define OLED_CS   10
#define OLED_DC    9
// RESET ist abgeklemmt -> U8X8_PIN_NONE verwenden!
U8G2_SSD1309_128X64_NONAME2_F_4W_HW_SPI u8g2(U8G2_R0, OLED_CS, OLED_DC, U8X8_PIN_NONE);

// --------- Compass Auto-Detect ---------
enum CompassType { COMPASS_NONE, COMPASS_HMC5883L, COMPASS_QMC5883L };
CompassType compassType = COMPASS_NONE;
uint8_t compassAddr = 0x00;

float declination_deg = 4.0f;

bool i2cDevicePresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

// ---------- HMC5883L (0x1E) ----------
void hmcWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(0x1E);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}
int16_t hmcRead16(uint8_t reg) {
  Wire.beginTransmission(0x1E);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)0x1E, (uint8_t)2);
  int16_t v = (Wire.read() << 8) | Wire.read();
  return v;
}
void hmcInit() {
  hmcWrite(0x00, 0x70);
  hmcWrite(0x01, 0x20);
  hmcWrite(0x02, 0x00);
}
void hmcRead(int16_t &x, int16_t &y, int16_t &z) {
  x = hmcRead16(0x03);
  z = hmcRead16(0x05);
  y = hmcRead16(0x07);
}

// ---------- QMC5883L (0x0D) ----------
void qmcWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(0x0D);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}
int16_t qmcRead16LE(uint8_t reg) {
  Wire.beginTransmission(0x0D);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)0x0D, (uint8_t)2);
  uint8_t l = Wire.read();
  uint8_t h = Wire.read();
  return (int16_t)((h << 8) | l);
}
void qmcInit() {
  qmcWrite(0x0B, 0x01);
  qmcWrite(0x09, 0b00011101);
}
void qmcRead(int16_t &x, int16_t &y, int16_t &z) {
  x = qmcRead16LE(0x00);
  y = qmcRead16LE(0x02);
  z = qmcRead16LE(0x04);
}

// ---------- Heading ----------
float computeHeadingDeg(int16_t x, int16_t y) {
  float heading = atan2((float)y, (float)x) * 180.0f / PI;
  heading += declination_deg;
  while (heading < 0) heading += 360.0f;
  while (heading >= 360.0f) heading -= 360.0f;
  return heading;
}

// ---------- Heading MAGNETISCH (ohne Declination) für NMEA HDM ----------
float computeHeadingMagDeg(int16_t x, int16_t y) {
  float heading = atan2((float)y, (float)x) * 180.0f / PI;
  while (heading < 0) heading += 360.0f;
  while (heading >= 360.0f) heading -= 360.0f;
  return heading;
}

void detectCompass() {
  if (i2cDevicePresent(0x1E)) {
    compassType = COMPASS_HMC5883L;
    compassAddr = 0x1E;
    hmcInit();
  } else if (i2cDevicePresent(0x0D)) {
    compassType = COMPASS_QMC5883L;
    compassAddr = 0x0D;
    qmcInit();
  } else {
    compassType = COMPASS_NONE;
    compassAddr = 0x00;
  }
}

// -------------------- NMEA 0183 helpers --------------------
uint8_t nmeaChecksum(const char* s) {
  uint8_t c = 0;
  while (*s) c ^= (uint8_t)(*s++);
  return c;
}

void nmeaPrintSentence(const char* body) {
  uint8_t cs = nmeaChecksum(body);
  Serial.print('$');
  Serial.print(body);
  Serial.print('*');
  if (cs < 16) Serial.print('0');
  Serial.print(cs, HEX);
  Serial.print("\r\n");
}

// Format: lat ddmm.mmmm + N/S, lon dddmm.mmmm + E/W
void formatLat(double lat, char* out_ddmm, char* out_hemi) {
  char hemi = (lat >= 0) ? 'N' : 'S';
  lat = fabs(lat);
  int deg = (int)lat;
  double min = (lat - deg) * 60.0;
  sprintf(out_ddmm, "%02d%07.4f", deg, min);
  out_hemi[0] = hemi; out_hemi[1] = '\0';
}

void formatLon(double lon, char* out_dddmm, char* out_hemi) {
  char hemi = (lon >= 0) ? 'E' : 'W';
  lon = fabs(lon);
  int deg = (int)lon;
  double min = (lon - deg) * 60.0;
  sprintf(out_dddmm, "%03d%07.4f", deg, min);
  out_hemi[0] = hemi; out_hemi[1] = '\0';
}

void formatTimeUTC(char* out_hhmmss, uint32_t hh, uint32_t mm, uint32_t ss) {
  sprintf(out_hhmmss, "%02lu%02lu%02lu", (unsigned long)hh, (unsigned long)mm, (unsigned long)ss);
}

void formatDateDDMMYY(char* out_ddmmyy, uint32_t dd, uint32_t mm, uint32_t yy) {
  sprintf(out_ddmmyy, "%02lu%02lu%02lu", (unsigned long)dd, (unsigned long)mm, (unsigned long)(yy % 100));
}

void emitGNRMC() {
  char body[140];
  char t[16] = "";
  char d[16] = "";
  char lat_s[16] = "";
  char lon_s[16] = "";
  char lat_h[2] = "";
  char lon_h[2] = "";

  bool timeValid = gps.time.isValid();
  bool dateValid = gps.date.isValid();
  bool locValid  = gps.location.isValid();

  if (timeValid) formatTimeUTC(t, gps.time.hour(), gps.time.minute(), gps.time.second());
  if (dateValid) formatDateDDMMYY(d, gps.date.day(), gps.date.month(), gps.date.year());

  if (locValid) {
    formatLat(gps.location.lat(), lat_s, lat_h);
    formatLon(gps.location.lng(), lon_s, lon_h);
  }

  const char status = (locValid && timeValid) ? 'A' : 'V';
  double sog = gps.speed.isValid() ? gps.speed.knots() : 0.0;
  double cog = gps.course.isValid() ? gps.course.deg() : 0.0;

  char timeField[20];
  if (timeValid) sprintf(timeField, "%s.00", t);
  else strcpy(timeField, "");

  sprintf(
    body,
    "GNRMC,%s,%c,%s,%s,%s,%s,%.1f,%.1f,%s,,",
    timeField,
    status,
    locValid ? lat_s : "",
    locValid ? lat_h : "",
    locValid ? lon_s : "",
    locValid ? lon_h : "",
    sog,
    cog,
    dateValid ? d : ""
  );

  nmeaPrintSentence(body);
}

void emitGNGGA() {
  char body[160];
  char t[16] = "";
  char lat_s[16] = "";
  char lon_s[16] = "";
  char lat_h[2] = "";
  char lon_h[2] = "";

  bool timeValid = gps.time.isValid();
  bool locValid  = gps.location.isValid();
  bool altValid  = gps.altitude.isValid();

  if (timeValid) formatTimeUTC(t, gps.time.hour(), gps.time.minute(), gps.time.second());
  if (locValid) {
    formatLat(gps.location.lat(), lat_s, lat_h);
    formatLon(gps.location.lng(), lon_s, lon_h);
  }

  char timeField[20];
  if (timeValid) sprintf(timeField, "%s.00", t);
  else strcpy(timeField, "");

  int fix = locValid ? 1 : 0;
  int sats = gps.satellites.isValid() ? (int)gps.satellites.value() : 0;
  double hdop = gps.hdop.isValid() ? gps.hdop.hdop() : 99.9;
  double altm = altValid ? gps.altitude.meters() : 0.0;

  sprintf(
    body,
    "GNGGA,%s,%s,%s,%s,%s,%d,%02d,%.1f,%.1f,M,,M,,",
    timeField,
    locValid ? lat_s : "",
    locValid ? lat_h : "",
    locValid ? lon_s : "",
    locValid ? lon_h : "",
    fix,
    sats,
    hdop,
    altm
  );

  nmeaPrintSentence(body);
}

// --------- Kompass NMEA: HCHDM (magnetic) ---------
void emitHCHDM(float headingMagDeg, bool haveCompass) {
  if (!haveCompass || isnan(headingMagDeg)) return;
  char body[40];
  sprintf(body, "HCHDM,%.1f,M", headingMagDeg);
  nmeaPrintSentence(body);
}

// --------- OLED Rendering ---------
void drawOLED(bool haveCompass, float headingTrue,
              bool locValid, double lat, double lon,
              int sats, double spdKmh, double altm,
              bool timeValid, uint32_t hh, uint32_t mm, uint32_t ss) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);

  u8g2.setCursor(0, 12);
  u8g2.print(locValid ? "FIX " : "NOFIX ");
  if (timeValid) {
    if (hh < 10) u8g2.print('0'); u8g2.print(hh); u8g2.print(':');
    if (mm < 10) u8g2.print('0'); u8g2.print(mm); u8g2.print(':');
    if (ss < 10) u8g2.print('0'); u8g2.print(ss);
    u8g2.print(" UTC");
  } else {
    u8g2.print("--:--:-- UTC");
  }

  u8g2.setCursor(0, 26);
  u8g2.print("SAT ");
  u8g2.print(sats);
  u8g2.print("  SPD ");
  u8g2.print(spdKmh, 1);
  u8g2.print("kmh");

  u8g2.setCursor(0, 40);
  u8g2.print("ALT ");
  u8g2.print(altm, 1);
  u8g2.print("m  HDG ");
  if (haveCompass) u8g2.print(headingTrue, 1);
  else u8g2.print("---.-");
  u8g2.print((char)176);

  // ---- LAT mit N/S ----
  u8g2.setCursor(0, 54);
  u8g2.print("LAT ");
  if (locValid) {
    u8g2.print(fabs(lat), 6);
    u8g2.print(lat >= 0 ? " N" : " S");
  } else {
    u8g2.print("-----.------");
  }

  // ---- LON mit E/W ----
  u8g2.setCursor(0, 64);
  u8g2.print("LON ");
  if (locValid) {
    u8g2.print(fabs(lon), 6);
    u8g2.print(lon >= 0 ? " E" : " W");
  } else {
    u8g2.print("-----.------");
  }

  u8g2.sendBuffer();
}

void setup() {
  // Serial = NMEA Ausgabe
  Serial.begin(NMEA_BAUD);

  Wire.begin();
  Serial1.begin(GPS_BAUD);

  // OLED init (kein Reset-Pin)
  u8g2.begin();
  //delay(500);
  u8g2.setContrast(255);

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.setCursor(0, 12); u8g2.print("BE/BN-880 GPS + ");
  u8g2.setCursor(0, 24); u8g2.print("Compass + SSD1309");
  u8g2.setCursor(0, 36); u8g2.print("GPS baud "); u8g2.print(GPS_BAUD);
  u8g2.setCursor(0, 48); u8g2.print("NMEA baud "); u8g2.print(NMEA_BAUD);
  u8g2.setCursor(0, 60); u8g2.print("(Eng.) Peter Schulte");
  u8g2.sendBuffer(); 
  delay(5000);

  detectCompass();
}

uint32_t lastPrint = 0;

void loop() {
  while (Serial1.available()) {
    gps.encode(Serial1.read());
  }

  int16_t mx = 0, my = 0, mz = 0;
  bool haveCompass = (compassType != COMPASS_NONE);
  float headingTrue = NAN;   // mit declination (Anzeige)
  float headingMag  = NAN;   // ohne declination (NMEA HCHDM)

  if (haveCompass) {
    if (compassType == COMPASS_HMC5883L) hmcRead(mx, my, mz);
    else qmcRead(mx, my, mz);

    headingTrue = computeHeadingDeg(mx, my);
    headingMag  = computeHeadingMagDeg(mx, my);
  }

  if (millis() - lastPrint >= 500) {
    lastPrint = millis();

    bool locValid = gps.location.isValid();
    double lat = locValid ? gps.location.lat() : 0.0;
    double lon = locValid ? gps.location.lng() : 0.0;

    int sats = gps.satellites.isValid() ? (int)gps.satellites.value() : 0;
    double spdKmh = gps.speed.isValid() ? gps.speed.kmph() : 0.0;
    double altm = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;

    bool timeValid = gps.time.isValid();
    uint32_t hh = timeValid ? gps.time.hour() : 0;
    uint32_t mm = timeValid ? gps.time.minute() : 0;
    uint32_t ss = timeValid ? gps.time.second() : 0;

    // Klartext -> OLED
    drawOLED(haveCompass, headingTrue, locValid, lat, lon, sats, spdKmh, altm, timeValid, hh, mm, ss);

    // NMEA -> Serial
    emitGNRMC();
    emitGNGGA();
    emitHCHDM(headingMag, haveCompass);
  }
}
