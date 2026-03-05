#include <Wire.h>
#include <TinyGPSPlus.h>
#include <math.h>

#include <U8g2lib.h>
#include <SPI.h>
#include <string.h>

TinyGPSPlus gps;

// --------- GPS UART ---------
static const uint32_t GPS_BAUD  = 38400;
static const uint32_t NMEA_BAUD = 115200;

// --------- OLED (SSD1309 2.42" SPI) ---------
#define OLED_CS   10
#define OLED_DC    9
// --------- HUD / Mirror Input ---------
#define MIRROR_PIN 7   // freier Digital-Eingang für HUD

// RESET ist abgeklemmt -> U8X8_PIN_NONE verwenden!
U8G2_SSD1309_128X64_NONAME2_F_4W_HW_SPI u8g2(U8G2_R0, OLED_CS, OLED_DC, U8X8_PIN_NONE);

// --------- Compass Auto-Detect ---------
enum CompassType { COMPASS_NONE, COMPASS_HMC5883L, COMPASS_QMC5883L };
CompassType compassType = COMPASS_NONE;
uint8_t compassAddr = 0x00;

// Declination (magnetische Missweisung) -> optional!
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

// ===================== NMEA 0183 (AVR-sicher, streaming) =====================
static uint8_t nmea_cs = 0;

static inline void nmeaTrimLeft(char* s) {
  while (*s == ' ') memmove(s, s + 1, strlen(s));
}

void nmeaStart() {
  nmea_cs = 0;
  Serial.print('$');
}

void nmeaPutChar(char c) {
  nmea_cs ^= (uint8_t)c;
  Serial.print(c);
}

void nmeaPutStr(const char* s) {
  while (*s) nmeaPutChar(*s++);
}

void nmeaComma() { nmeaPutChar(','); }

void nmeaEnd() {
  Serial.print('*');
  if (nmea_cs < 16) Serial.print('0');
  Serial.print(nmea_cs, HEX);
  Serial.print("\r\n");
}

// ddmm.mmmm / dddmm.mmmm (ohne float-printf)
void nmeaFormatLatLon(double lat, double lon,
                      char* latField, char* latHemi,
                      char* lonField, char* lonHemi) {
  char lh = (lat >= 0) ? 'N' : 'S';
  char loh = (lon >= 0) ? 'E' : 'W';

  lat = fabs(lat);
  lon = fabs(lon);

  int latDeg = (int)lat;
  int lonDeg = (int)lon;

  double latMin = (lat - latDeg) * 60.0;
  double lonMin = (lon - lonDeg) * 60.0;

  char buf[16];

  // lat minutes -> mm.mmmm, immer 2-stellig vor dem Punkt
  dtostrf(latMin, 0, 4, buf);
  nmeaTrimLeft(buf);
  if (buf[1] == '.') { // z.B. "7.1234"
    char tmp[16];
    tmp[0] = '0';
    strcpy(tmp + 1, buf);
    strcpy(buf, tmp);
  }
  sprintf(latField, "%02d%s", latDeg, buf);

  // lon minutes -> mm.mmmm
  dtostrf(lonMin, 0, 4, buf);
  nmeaTrimLeft(buf);
  if (buf[1] == '.') {
    char tmp[16];
    tmp[0] = '0';
    strcpy(tmp + 1, buf);
    strcpy(buf, tmp);
  }
  sprintf(lonField, "%03d%s", lonDeg, buf);

  latHemi[0] = lh; latHemi[1] = '\0';
  lonHemi[0] = loh; lonHemi[1] = '\0';
}

void emitGNRMC() {
  // $GNRMC,hhmmss.ss,A,lat,N,lon,E,sog,cog,ddmmyy,,
  bool locValid  = gps.location.isValid();
  bool timeValid = gps.time.isValid();
  bool dateValid = gps.date.isValid();

  nmeaStart();
  nmeaPutStr("GNRMC"); nmeaComma();

  // time
  if (timeValid) {
    char t[16];
    sprintf(t, "%02u%02u%02u.00", gps.time.hour(), gps.time.minute(), gps.time.second());
    nmeaPutStr(t);
  }
  nmeaComma();

  char status = (locValid && timeValid) ? 'A' : 'V';
  nmeaPutChar(status);
  nmeaComma();

  // lat/lon
  if (locValid) {
    char latF[20], lonF[20], latH[2], lonH[2];
    nmeaFormatLatLon(gps.location.lat(), gps.location.lng(), latF, latH, lonF, lonH);
    nmeaPutStr(latF); nmeaComma();
    nmeaPutStr(latH); nmeaComma();
    nmeaPutStr(lonF); nmeaComma();
    nmeaPutStr(lonH); nmeaComma();
  } else {
    nmeaComma(); nmeaComma(); nmeaComma(); nmeaComma();
  }

  // SOG knots
  if (gps.speed.isValid()) {
    char s[16];
    dtostrf(gps.speed.knots(), 0, 1, s);
    nmeaTrimLeft(s);
    nmeaPutStr(s);
  }
  nmeaComma();

  // COG
  if (gps.course.isValid()) {
    char c[16];
    dtostrf(gps.course.deg(), 0, 1, c);
    nmeaTrimLeft(c);
    nmeaPutStr(c);
  }
  nmeaComma();

  // date
  if (dateValid) {
    char d[16];
    sprintf(d, "%02u%02u%02u", gps.date.day(), gps.date.month(), (uint8_t)(gps.date.year() % 100));
    nmeaPutStr(d);
  }

  // variation + mode leer: ",,"
  nmeaComma();
  nmeaComma();

  nmeaEnd();
}

void emitGNGGA() {
  // $GNGGA,hhmmss.ss,lat,N,lon,E,fix,sats,hdop,alt,M,,M,,
  bool locValid  = gps.location.isValid();
  bool timeValid = gps.time.isValid();

  nmeaStart();
  nmeaPutStr("GNGGA"); nmeaComma();

  if (timeValid) {
    char t[16];
    sprintf(t, "%02u%02u%02u.00", gps.time.hour(), gps.time.minute(), gps.time.second());
    nmeaPutStr(t);
  }
  nmeaComma();

  if (locValid) {
    char latF[20], lonF[20], latH[2], lonH[2];
    nmeaFormatLatLon(gps.location.lat(), gps.location.lng(), latF, latH, lonF, lonH);
    nmeaPutStr(latF); nmeaComma();
    nmeaPutStr(latH); nmeaComma();
    nmeaPutStr(lonF); nmeaComma();
    nmeaPutStr(lonH); nmeaComma();
  } else {
    nmeaComma(); nmeaComma(); nmeaComma(); nmeaComma();
  }

  nmeaPutChar(locValid ? '1' : '0'); // fix quality
  nmeaComma();

  // sats
  if (gps.satellites.isValid()) {
    char s[6];
    sprintf(s, "%02u", gps.satellites.value());
    nmeaPutStr(s);
  } else {
    nmeaPutStr("00");
  }
  nmeaComma();

  // hdop
  if (gps.hdop.isValid()) {
    char h[16];
    dtostrf(gps.hdop.hdop(), 0, 1, h);
    nmeaTrimLeft(h);
    nmeaPutStr(h);
  } else {
    nmeaPutStr("99.9");
  }
  nmeaComma();

  // altitude
  if (gps.altitude.isValid()) {
    char a[16];
    dtostrf(gps.altitude.meters(), 0, 1, a);
    nmeaTrimLeft(a);
    nmeaPutStr(a);
  } else {
    nmeaPutStr("0.0");
  }
  nmeaComma();
  nmeaPutChar('M');
  nmeaComma();

  // geoid sep leer, unit M, then empty
  nmeaComma();
  nmeaPutChar('M');
  nmeaComma();
  nmeaComma();

  nmeaEnd();
}

// --------- SOG als VTG (Course/Speed over Ground) ---------
void emitGNVTG() {
  // $GNVTG,cog,T,,M,sog,N,spd,K
  nmeaStart();
  nmeaPutStr("GNVTG"); nmeaComma();

  // COG true
  if (gps.course.isValid()) {
    char c[16];
    dtostrf(gps.course.deg(), 0, 1, c);
    nmeaTrimLeft(c);
    nmeaPutStr(c);
  }
  nmeaComma();
  nmeaPutChar('T');
  nmeaComma();

  // empty magnetic course
  nmeaComma();
  nmeaPutChar('M');
  nmeaComma();

  // speed knots
  if (gps.speed.isValid()) {
    char s[16];
    dtostrf(gps.speed.knots(), 0, 1, s);
    nmeaTrimLeft(s);
    nmeaPutStr(s);
  }
  nmeaComma();
  nmeaPutChar('N');
  nmeaComma();

  // speed km/h
  if (gps.speed.isValid()) {
    char k[16];
    dtostrf(gps.speed.kmph(), 0, 1, k);
    nmeaTrimLeft(k);
    nmeaPutStr(k);
  }
  nmeaComma();
  nmeaPutChar('K');

  nmeaEnd();
}

// --------- Kompass NMEA: HCHDM (magnetic) ---------
void emitHCHDM(float headingMagDeg, bool haveCompass) {
  if (!haveCompass || isnan(headingMagDeg)) return;

  nmeaStart();
  nmeaPutStr("HCHDM"); nmeaComma();

  char h[16];
  dtostrf(headingMagDeg, 0, 1, h);
  nmeaTrimLeft(h);
  nmeaPutStr(h);

  nmeaComma();
  nmeaPutChar('M');
  nmeaEnd();
}

// ---- GPS Koordinaten als Grad/Min/Sek (DMS) ohne Float-Printf (AVR-sicher) ----
// Ausgabe: 52°31'12.3" N  bzw. 013°24'17.8" E
void formatDMS(double degVal, bool isLat, char* out, size_t outSize) {
  char hemi = isLat ? ((degVal >= 0) ? 'N' : 'S')
                    : ((degVal >= 0) ? 'E' : 'W');

  double a = fabs(degVal);

  int deg = (int)a;
  double mFloat = (a - deg) * 60.0;
  int min = (int)mFloat;

  double secFloat = (mFloat - min) * 60.0;
  int sec10 = (int)lround(secFloat * 10.0);

  if (sec10 >= 600) { sec10 = 0; min++; }
  if (min >= 60)    { min = 0; deg++; }

  int sec = sec10 / 10;
  int tenth = sec10 % 10;

  if (isLat) {
    snprintf(out, outSize, "%02d%c%02d'%02d.%1d\" %c",
             deg, (char)176, min, sec, tenth, hemi);
  } else {
    snprintf(out, outSize, "%03d%c%02d'%02d.%1d\" %c",
             deg, (char)176, min, sec, tenth, hemi);
  }
}

// ======= Kompassrose (Grafik) =======
static inline float deg2rad(float d) { return d * (float)PI / 180.0f; }

void drawCompassRose(int cx, int cy, int r, float headingDeg) {
  u8g2.drawCircle(cx, cy, r, U8G2_DRAW_ALL);
  u8g2.drawCircle(cx, cy, r - 1, U8G2_DRAW_ALL);

  for (int a = 0; a < 360; a += 10) {
    float ar = deg2rad((float)a - 90.0f);
    int x1 = cx + (int)lroundf(cosf(ar) * (r - 1));
    int y1 = cy + (int)lroundf(sinf(ar) * (r - 1));

    int inner = (a % 30 == 0) ? (r - 6) : (r - 3);
    int x2 = cx + (int)lroundf(cosf(ar) * inner);
    int y2 = cy + (int)lroundf(sinf(ar) * inner);

    u8g2.drawLine(x1, y1, x2, y2);
  }

  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(cx - 2, cy - r + 8, "N");
  u8g2.drawStr(cx + r - 8, cy + 3, "E");
  u8g2.drawStr(cx - 2, cy + r - 2, "S");
  u8g2.drawStr(cx - r + 2, cy + 3, "W");

  float hr = deg2rad(headingDeg - 90.0f);
  int tipX = cx + (int)lroundf(cosf(hr) * (r - 7));
  int tipY = cy + (int)lroundf(sinf(hr) * (r - 7));

  int baseX = cx + (int)lroundf(cosf(hr) * (r - 16));
  int baseY = cy + (int)lroundf(sinf(hr) * (r - 16));

  float pr = hr + (float)PI / 2.0f;
  int wing = 5;
  int leftX  = baseX + (int)lroundf(cosf(pr) * wing);
  int leftY  = baseY + (int)lroundf(sinf(pr) * wing);
  int rightX = baseX - (int)lroundf(cosf(pr) * wing);
  int rightY = baseY - (int)lroundf(sinf(pr) * wing);

  u8g2.drawLine(cx, cy, baseX, baseY);
  u8g2.drawTriangle(tipX, tipY, leftX, leftY, rightX, rightY);

  u8g2.drawDisc(cx, cy, 2, U8G2_DRAW_ALL);

  char buf[10];
  snprintf(buf, sizeof(buf), "%03d%c", ((int)lroundf(headingDeg)) % 360, (char)176);
  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(cx - 12, cy + r + 9, buf);
}

// --------- OLED Rendering (Text + Kompassrose) ---------
void drawOLED(bool haveCompass, float headingTrue,
              bool locValid, double lat, double lon,
              int sats, double spdKmh, double altm,
              bool timeValid, uint32_t hh, uint32_t mm, uint32_t ss) {
  u8g2.clearBuffer();

  u8g2.setFont(u8g2_font_6x12_tf);

  // Zeile 1: Fix + Zeit
  u8g2.setCursor(0, 12);
  u8g2.print(locValid ? "FIX " : "NOFIX ");
  if (timeValid) {
    if (hh < 10) u8g2.print('0'); u8g2.print(hh); u8g2.print(':');
    if (mm < 10) u8g2.print('0'); u8g2.print(mm); u8g2.print(':');
    if (ss < 10) u8g2.print('0'); u8g2.print(ss);
    u8g2.setCursor(40, 26);
    u8g2.print(" UTC");
  } else {
    u8g2.print("--:--:-- UTC");
  }

  // Zeile 2: Sats
  u8g2.setCursor(0, 26);
  u8g2.print("SAT ");
  u8g2.print(sats);

  // Zeile 3: Speed
  u8g2.setCursor(0, 40);
  u8g2.print("SPD ");
  u8g2.print(spdKmh, 1);
  u8g2.print(" kmh");

  // Zeile 4/5: Position in DMS
  if (locValid) {
    char latStr[24];
    char lonStr[24];
    formatDMS(lat, true,  latStr, sizeof(latStr));
    formatDMS(lon, false, lonStr, sizeof(lonStr));

    u8g2.setCursor(0, 54);
    u8g2.print(latStr);

    u8g2.setCursor(0, 64);
    u8g2.print(lonStr);
  } else {
    u8g2.setCursor(0, 64);
    u8g2.print("POS ----");
  }

  // Rechts: Kompassrose
  const int cx = 98;
  const int cy = 30;
  const int r  = 22;

  if (haveCompass && !isnan(headingTrue)) {
    drawCompassRose(cx, cy, r, headingTrue);
  } else {
    u8g2.drawCircle(cx, cy, r, U8G2_DRAW_ALL);
    u8g2.setFont(u8g2_font_5x8_tf);
    u8g2.drawStr(cx - 12, cy + 3, "NO HDG");
  }

  u8g2.sendBuffer();
}

void setup() {
  // Serial = NMEA Ausgabe (für SignalK)
  Serial.begin(NMEA_BAUD);
  Wire.begin();

  pinMode(MIRROR_PIN, INPUT_PULLUP);

  // Startzustand: normales Display
  u8g2.setFlipMode(0);

  Serial1.begin(GPS_BAUD);

  // OLED init
  u8g2.begin();
  delay(300);
  u8g2.setContrast(255);

  // Bootscreen
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.setCursor(0, 12); u8g2.print("BE/BN-880 GPS+Kompass");
  u8g2.setCursor(0, 24); u8g2.print("OLED SSD1309");
  u8g2.setCursor(0, 36); u8g2.print("GPS baud ");  u8g2.print(GPS_BAUD);
  u8g2.setCursor(0, 48); u8g2.print("NMEA baud "); u8g2.print(NMEA_BAUD);
  u8g2.setCursor(0, 60); u8g2.print("(C) Peter Schulte");
  u8g2.sendBuffer();
  delay(5000);

  detectCompass();
}

uint32_t lastPrint = 0;

void loop() {
  // --------- Spiegelverkehrt (Links <-> Rechts) ---------
  static bool lastMirror = false;
  bool mirror = (digitalRead(MIRROR_PIN) == HIGH);

  if (mirror != lastMirror) {
    u8g2.setFlipMode(mirror ? 1 : 0);
    lastMirror = mirror;
  }

  // GPS decode
  while (Serial1.available()) {
    gps.encode(Serial1.read());
  }

  // Compass read
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

  // Periodic output
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

    // OLED
    drawOLED(haveCompass, headingTrue, locValid, lat, lon, sats, spdKmh, altm, timeValid, hh, mm, ss);

    // NMEA
    emitGNRMC();
    emitGNGGA();
    emitGNVTG();
    emitHCHDM(headingMag, haveCompass);
  }
}