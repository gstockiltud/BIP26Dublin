/*
 * ╔══════════════════════════════════════════════════════════════╗
 * ║         BIP26 GNSS SIGNAL QUALITY GAUGE                     ║
 * ║         TU Dublin  |  SENSATE-X 2.0                         ║
 * ╠══════════════════════════════════════════════════════════════╣
 * ║  Hardware:                                                   ║
 * ║    Tenstar TS-ESP32-S3                                       ║
 * ║    Waveshare LC29H(AA) GNSS Module                          ║
 * ║    ST7789 TFT Display (135x240, landscape 240x135)           ║
 * ║    Onboard WS2812 RGB LED                                    ║
 * ╠══════════════════════════════════════════════════════════════╣
 * ║  Display layout:                                             ║
 * ║    Large semicircular HDOP dial (speedometer style)          ║
 * ║    Colour coded: Green=Excellent  Amber=Good  Red=Poor       ║
 * ║    Satellites used / in view                                 ║
 * ║    Estimated accuracy in metres                              ║
 * ║    Fix type and UTC time                                     ║
 * ╠══════════════════════════════════════════════════════════════╣
 * ║  Pin Definitions (Mike Gill / Tenstar documentation)         ║
 * ║    TFT_CS=7  TFT_DC=39  TFT_RST=40  TFT_BL=45              ║
 * ║    SPI_SCK=36  SPI_MISO=37  SPI_MOSI=35                     ║
 * ║    GPS_RX=17  GPS_TX=18   RGB_LED=33                         ║
 * ╠══════════════════════════════════════════════════════════════╣
 * ║  Board Settings:                                             ║
 * ║    Upload Mode: UART0 / Hardware CDC                         ║
 * ║    USB Mode:    Hardware CDC and JTAG                        ║
 * ║    Flash Size:  4MB (32Mb)                                   ║
 * ╚══════════════════════════════════════════════════════════════╝
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Adafruit_NeoPixel.h>
#include <TinyGPSPlus.h>
#include <math.h>

// ── Pin Definitions ────────────────────────────────────────────
#define TFT_CS         7
#define TFT_DC        39
#define TFT_RST       40
#define TFT_backlight 45
#define SPI_SCK       36
#define SPI_MISO      37
#define SPI_MOSI      35
#define I2C_SDA       42
#define I2C_SCL       41
#define LED_PIN       33
#define NUM_LEDS       1
#define GPS_RX        17
#define GPS_TX        18
#define GPS_BAUD      115200

// ── Screen (landscape rotation 3) ─────────────────────────────
#define SCREEN_W  240
#define SCREEN_H  135

// ── Colours (RGB565) ───────────────────────────────────────────
#define COL_BG       0x0000   // Black
#define COL_WHITE    0xFFFF
#define COL_GOLD     0xFEA0
#define COL_CYAN     0x07FF
#define COL_GREY     0x4208
#define COL_DKGREY   0x2104
#define COL_LTGREY   0x8410
#define COL_NAVY     0x000F
#define COL_GREEN    0x07E0   // Excellent HDOP
#define COL_LGREEN   0x47E0   // Good HDOP
#define COL_AMBER    0xFD20   // Moderate HDOP
#define COL_ORANGE   0xFC00   // Poor HDOP
#define COL_RED      0xF800   // Very poor HDOP

// ── Dial geometry ──────────────────────────────────────────────
// Semicircle dial centred at (cx, cy), radius r
// Spans from 210° to 330° (left to right, open at top)
// Standard trig: 0° = right, angles increase anti-clockwise
// We want dial to sweep left(210°) → right(330°) so:
//   HDOP 0  → 210° (left, green)
//   HDOP 10 → 330° (right, red)
#define DIAL_CX   90     // Centre x of dial
#define DIAL_CY   80     // Centre y of dial
#define DIAL_R    62     // Outer radius
#define DIAL_R2   44     // Inner radius (arc width = R - R2)
#define NEEDLE_R  58     // Needle length

// ── Objects ────────────────────────────────────────────────────
Adafruit_ST7789   tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
Adafruit_NeoPixel led = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
TinyGPSPlus       gps;
HardwareSerial    GPSSerial(1);

// ── State ──────────────────────────────────────────────────────
unsigned long lastUpdate = 0;
const unsigned long REFRESH_MS = 1000;
bool     gpsFixed  = false;
uint16_t ledHue    = 0;
float    lastHdop  = -1;   // Track last needle position for erase

// ── HDOP → colour mapping ──────────────────────────────────────
uint16_t hdopColour(float h) {
  if (h < 1.0)  return COL_GREEN;
  if (h < 2.0)  return COL_LGREEN;
  if (h < 5.0)  return COL_AMBER;
  if (h < 10.0) return COL_ORANGE;
  return COL_RED;
}

// ── HDOP → quality label ───────────────────────────────────────
const char* hdopLabel(float h) {
  if (h < 1.0)  return "IDEAL";
  if (h < 2.0)  return "EXCELLENT";
  if (h < 5.0)  return "GOOD";
  if (h < 10.0) return "MODERATE";
  return "POOR";
}

// ── Accuracy estimate (rough: HDOP x typical horiz error) ─────
float accuracyMetres(float h) {
  return h * 3.0;   // ~3m base error for LC29H
}

// ── Draw dial arc in segments, each coloured by HDOP range ────
// Dial sweeps from startDeg to endDeg (degrees, clockwise from right)
// We use Adafruit fillTriangle to approximate arc segments
void drawDialArc() {
  // Draw coloured arc segments
  // Each segment covers a range of HDOP values
  // Angle mapping: HDOP 0→10 maps to 210°→330° (120° sweep)
  // We draw 60 thin segments

  const int SEGMENTS = 60;
  const float START_DEG = 210.0;
  const float END_DEG   = 330.0;
  const float SWEEP     = END_DEG - START_DEG;

  for (int s = 0; s < SEGMENTS; s++) {
    float a1 = START_DEG + (SWEEP * s)       / SEGMENTS;
    float a2 = START_DEG + (SWEEP * (s + 1)) / SEGMENTS;
    float hdopVal = 10.0 * s / SEGMENTS;
    uint16_t col = hdopColour(hdopVal);

    // Convert to radians
    float r1 = a1 * DEG_TO_RAD;
    float r2 = a2 * DEG_TO_RAD;

    // Outer and inner arc points
    int x1o = DIAL_CX + (int)(DIAL_R  * cos(r1));
    int y1o = DIAL_CY + (int)(DIAL_R  * sin(r1));
    int x2o = DIAL_CX + (int)(DIAL_R  * cos(r2));
    int y2o = DIAL_CY + (int)(DIAL_R  * sin(r2));
    int x1i = DIAL_CX + (int)(DIAL_R2 * cos(r1));
    int y1i = DIAL_CY + (int)(DIAL_R2 * sin(r1));
    int x2i = DIAL_CX + (int)(DIAL_R2 * cos(r2));
    int y2i = DIAL_CY + (int)(DIAL_R2 * sin(r2));

    tft.fillTriangle(x1o, y1o, x2o, y2o, x1i, y1i, col);
    tft.fillTriangle(x2o, y2o, x2i, y2i, x1i, y1i, col);
  }

  // Draw tick marks at key HDOP values: 0, 1, 2, 5, 10
  float keyHdop[] = { 0, 1, 2, 5, 10 };
  const char* keyLabel[] = { "0", "1", "2", "5", "10" };
  for (int k = 0; k < 5; k++) {
    float angle = (START_DEG + SWEEP * keyHdop[k] / 10.0) * DEG_TO_RAD;
    int tx1 = DIAL_CX + (int)((DIAL_R + 2)  * cos(angle));
    int ty1 = DIAL_CY + (int)((DIAL_R + 2)  * sin(angle));
    int tx2 = DIAL_CX + (int)((DIAL_R2 - 2) * cos(angle));
    int ty2 = DIAL_CY + (int)((DIAL_R2 - 2) * sin(angle));
    tft.drawLine(tx1, ty1, tx2, ty2, COL_WHITE);

    // Label — offset outward
    int lx = DIAL_CX + (int)((DIAL_R + 10) * cos(angle)) - 4;
    int ly = DIAL_CY + (int)((DIAL_R + 10) * sin(angle)) - 3;
    tft.setTextSize(1);
    tft.setTextColor(COL_LTGREY);
    tft.setCursor(lx, ly);
    tft.print(keyLabel[k]);
  }
}

// ── Draw the needle at a given HDOP value ──────────────────────
void drawNeedle(float hdop, uint16_t colour) {
  const float START_DEG = 210.0;
  const float SWEEP     = 120.0;
  float clamped = constrain(hdop, 0.0, 10.0);
  float angle   = (START_DEG + SWEEP * clamped / 10.0) * DEG_TO_RAD;

  int nx = DIAL_CX + (int)(NEEDLE_R * cos(angle));
  int ny = DIAL_CY + (int)(NEEDLE_R * sin(angle));

  // Draw needle as a thick line
  tft.drawLine(DIAL_CX, DIAL_CY, nx, ny, colour);
  tft.drawLine(DIAL_CX + 1, DIAL_CY, nx + 1, ny, colour);
  tft.drawLine(DIAL_CX, DIAL_CY + 1, nx, ny + 1, colour);

  // Centre dot
  tft.fillCircle(DIAL_CX, DIAL_CY, 4, colour);
}

// ── Erase old needle by redrawing arc over it ──────────────────
void eraseNeedle(float hdop) {
  // Redraw the arc segment under the old needle
  // Simplest approach: redraw a black needle then re-draw the arc
  drawNeedle(hdop, COL_BG);
  // Redraw arc segments near old needle position
  const float START_DEG = 210.0;
  const float SWEEP     = 120.0;
  const int SEGMENTS = 60;
  float clamped = constrain(hdop, 0.0, 10.0);
  int nearSeg = (int)(SEGMENTS * clamped / 10.0);
  int s0 = max(0, nearSeg - 3);
  int s1 = min(SEGMENTS - 1, nearSeg + 3);
  for (int s = s0; s <= s1; s++) {
    float a1 = START_DEG + (SWEEP * s)       / SEGMENTS;
    float a2 = START_DEG + (SWEEP * (s + 1)) / SEGMENTS;
    float hdopVal = 10.0 * s / SEGMENTS;
    uint16_t col = hdopColour(hdopVal);
    float r1 = a1 * DEG_TO_RAD;
    float r2 = a2 * DEG_TO_RAD;
    int x1o = DIAL_CX + (int)(DIAL_R  * cos(r1));
    int y1o = DIAL_CY + (int)(DIAL_R  * sin(r1));
    int x2o = DIAL_CX + (int)(DIAL_R  * cos(r2));
    int y2o = DIAL_CY + (int)(DIAL_R  * sin(r2));
    int x1i = DIAL_CX + (int)(DIAL_R2 * cos(r1));
    int y1i = DIAL_CY + (int)(DIAL_R2 * sin(r1));
    int x2i = DIAL_CX + (int)(DIAL_R2 * cos(r2));
    int y2i = DIAL_CY + (int)(DIAL_R2 * sin(r2));
    tft.fillTriangle(x1o, y1o, x2o, y2o, x1i, y1i, col);
    tft.fillTriangle(x2o, y2o, x2i, y2i, x1i, y1i, col);
  }
  // Restore centre dot black
  tft.fillCircle(DIAL_CX, DIAL_CY, 4, COL_BG);
}

// ── Draw static screen elements ────────────────────────────────
void drawStaticElements() {
  tft.fillScreen(COL_BG);

  // Header
  tft.fillRect(0, 0, SCREEN_W, 14, COL_NAVY);
  tft.drawFastHLine(0, 14, SCREEN_W, COL_GOLD);
  tft.setTextSize(1);
  tft.setTextColor(COL_GOLD);
  tft.setCursor(4, 3);
  tft.print("GNSS SIGNAL QUALITY");
  tft.setTextColor(COL_LTGREY);
  tft.setCursor(140, 3);
  tft.print("SENSATE-X 2.0");

  // Draw dial arc
  drawDialArc();

  // HDOP label under centre
  tft.setTextColor(COL_LTGREY);
  tft.setTextSize(1);
  tft.setCursor(DIAL_CX - 14, DIAL_CY + 18);
  tft.print("HDOP");

  // Right panel divider
  tft.drawFastVLine(SCREEN_W / 2 + 25, 16, SCREEN_H - 16, COL_GREY);

  // Right panel static labels
  tft.setTextColor(COL_LTGREY);
  tft.setTextSize(1);
  tft.setCursor(155, 20);  tft.print("QUALITY:");
  tft.setCursor(155, 45);  tft.print("SATS USED:");
  tft.setCursor(155, 60);  tft.print("SATS VIEW:");
  tft.setCursor(155, 75);  tft.print("ACCURACY:");
  tft.setCursor(155, 90);  tft.print("FIX TYPE:");
  tft.setCursor(155, 108); tft.print("UTC:");

  // Bottom border
  tft.drawFastHLine(0, SCREEN_H - 1, SCREEN_W, COL_NAVY);
}

// ── Update right panel data ────────────────────────────────────
void updatePanel() {
  // Clear right panel data area
  tft.fillRect(155, 28, 85, 95, COL_BG);

  tft.setTextSize(1);

  if (gps.hdop.isValid() && gpsFixed) {
    float h = gps.hdop.hdop();
    uint16_t col = hdopColour(h);

    // Quality label
    tft.setTextColor(col);
    tft.setCursor(155, 30);
    tft.print(hdopLabel(h));

    // HDOP value large
    tft.setTextSize(2);
    tft.setTextColor(col);
    char buf[8];
    snprintf(buf, sizeof(buf), "%.1f", h);
    tft.setCursor(DIAL_CX - 14, DIAL_CY + 28);
    tft.fillRect(DIAL_CX - 16, DIAL_CY + 26, 36, 18, COL_BG);
    tft.print(buf);
    tft.setTextSize(1);

    // Satellites used
    tft.setTextColor(COL_WHITE);
    tft.setCursor(155, 50);
    if (gps.satellites.isValid()) {
      char sb[4];
      snprintf(sb, sizeof(sb), "%d", gps.satellites.value());
      tft.print(sb);
    } else {
      tft.print("--");
    }

    // Satellites in view (from total NMEA)
    tft.setCursor(155, 65);
    tft.setTextColor(COL_CYAN);
    if (gps.satellites.isValid()) {
      char sb[4];
      snprintf(sb, sizeof(sb), "%d", gps.satellites.value());
      tft.print(sb);
    } else {
      tft.print("--");
    }

    // Accuracy
    tft.setTextColor(col);
    tft.setCursor(155, 80);
    char ab[12];
    snprintf(ab, sizeof(ab), "~%.0f m", accuracyMetres(h));
    tft.print(ab);

    // Fix type
    tft.setTextColor(COL_GREEN);
    tft.setCursor(155, 95);
    tft.print("3D FIX");

  } else {
    // No fix
    tft.setTextColor(COL_GREY);
    tft.setCursor(155, 30); tft.print("NO FIX");
    tft.setCursor(155, 50); tft.print("--");
    tft.setCursor(155, 65); tft.print("--");
    tft.setCursor(155, 80); tft.print("--");
    tft.setCursor(155, 95); tft.setTextColor(COL_ORANGE); tft.print("SEARCHING");

    // Show waiting HDOP value as max
    tft.setTextSize(2);
    tft.setTextColor(COL_GREY);
    tft.fillRect(DIAL_CX - 16, DIAL_CY + 26, 36, 18, COL_BG);
    tft.setCursor(DIAL_CX - 14, DIAL_CY + 28);
    tft.print("--");
    tft.setTextSize(1);
  }

  // UTC time (always show if available)
  tft.fillRect(155, 113, 85, 12, COL_BG);
  tft.setTextSize(1);
  if (gps.time.isValid()) {
    char tb[12];
    snprintf(tb, sizeof(tb), "%02d:%02d:%02d",
             gps.time.hour(), gps.time.minute(), gps.time.second());
    tft.setTextColor(COL_CYAN);
    tft.setCursor(155, 114);
    tft.print(tb);
  } else {
    tft.setTextColor(COL_GREY);
    tft.setCursor(155, 114);
    tft.print("--:--:--");
  }
}

// ── Update needle ──────────────────────────────────────────────
void updateNeedle() {
  float hdop = gpsFixed && gps.hdop.isValid() ? gps.hdop.hdop() : 10.0;
  hdop = constrain(hdop, 0.0, 10.0);

  if (abs(hdop - lastHdop) > 0.05 || lastHdop < 0) {
    if (lastHdop >= 0) eraseNeedle(lastHdop);
    drawNeedle(hdop, hdopColour(hdop));
    lastHdop = hdop;
  }
}

// ── RGB LED ────────────────────────────────────────────────────
void updateLED() {
  if (gpsFixed) {
    // Colour matches HDOP quality
    float h = gps.hdop.isValid() ? gps.hdop.hdop() : 10.0;
    uint16_t col565 = hdopColour(h);
    // Convert RGB565 to RGB888 roughly
    uint8_t r = ((col565 >> 11) & 0x1F) << 3;
    uint8_t g = ((col565 >> 5)  & 0x3F) << 2;
    uint8_t b = ((col565)       & 0x1F) << 3;
    led.setPixelColor(0, led.Color(r / 4, g / 4, b / 4));
  } else {
    // Slow red pulse — no fix
    uint8_t bri = abs((int)(millis() % 2000) - 1000) / 10;
    led.setPixelColor(0, led.Color(bri, 0, 0));
  }
  led.show();
}

// ── initDisplay ────────────────────────────────────────────────
void initDisplay() {
  pinMode(TFT_backlight, OUTPUT);
  digitalWrite(TFT_backlight, HIGH);
  tft.init(135, 240);
  tft.setRotation(3);
  tft.fillScreen(COL_BG);
  tft.setTextColor(COL_WHITE);
  tft.setTextSize(1);
}

// ── Setup ──────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, TFT_CS);
  Wire.begin(I2C_SDA, I2C_SCL);

  // Display first
  initDisplay();
  drawStaticElements();

  // Show waiting message in centre
  tft.setTextColor(COL_GREY);
  tft.setCursor(DIAL_CX - 30, DIAL_CY - 10);
  tft.print("Waiting...");

  // NeoPixel after display
  led.begin();
  led.setBrightness(30);
  led.setPixelColor(0, led.Color(20, 0, 0));
  led.show();

  // GPS
  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("BIP26 GNSS Signal Quality Gauge ready.");
}

// ── Loop ───────────────────────────────────────────────────────
void loop() {
  while (GPSSerial.available()) {
    gps.encode(GPSSerial.read());
  }

  gpsFixed = gps.location.isValid();

  updateLED();

  if (millis() - lastUpdate >= REFRESH_MS) {
    lastUpdate = millis();
    updateNeedle();
    updatePanel();
  }
}
