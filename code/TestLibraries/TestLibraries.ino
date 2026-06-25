/* * ==============================================================================
 * STMP26 SENSATE - MASTER HARDWARE BUS & LIBRARY DIAGNOSTIC
 * Target Hardware: TenStar ESP32-S3 Payload Configuration
 * Graphics Core: Adafruit_GFX + Adafruit_ST7789
 * ==============================================================================
 * 🛠️ LAB ENVIRONMENT PRE-REQUISITES:
 * If this code gives a "No such file or directory" error on compile, it means you 
 * are missing a library! Open the Library Manager (icon on the far left) and 
 * install: TinyGPS++, Adafruit ST7789, Adafruit NeoPixel, Adafruit BMP280, 
 * and QMI8658 (by Lahav Gahali).
 * ==============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>      // Core graphics library
#include <Adafruit_ST7789.h>   // Hardware-specific display driver
#include <Adafruit_NeoPixel.h> // Onboard RGB status handling
#include <TinyGPSPlus.h>       // NMEA Sentence parser engine
#include <Adafruit_BMP280.h>   // Barometric pressure and temperature sensor
#include <QMI8658.h>           // 6-Axis Inertial Measurement Unit (IMU) by Lahav Gahali
#include <math.h>

// ── Physical Pin Mappings (Hardwired layout constraints) ────────
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

#define SCREEN_W  240
#define SCREEN_H  135

// Global Hardware Object Setup
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
Adafruit_BMP280 bmp; 
QMI8658 imu;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("========================================");
  Serial.println("STMP26 Master Hardware Audit Initialization");
  Serial.println("========================================");

  // 1. Initialize Hardware SPI Bus Pins explicitly for the display link
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, TFT_CS);
  
  // Power up the display backlight panel
  pinMode(TFT_backlight, OUTPUT);
  digitalWrite(TFT_backlight, HIGH); 
  
  // 2. Initialize the visual screen display environment
  tft.init(SCREEN_H, SCREEN_W);
  tft.setRotation(3); // Landscape Mode
  tft.fillScreen(0x0000); // Clear to Black
  tft.setCursor(0, 0);
  tft.setTextColor(0xFFFF); // White
  tft.setTextSize(2);
  
  tft.println("STMP26 Library Audit:");
  tft.println("--------------------");

  // 3. Fire up the Inter-Integrated Circuit (I2C) peripheral bus lines
  Wire.begin(I2C_SDA, I2C_SCL, 100000); // Standard 100kHz clock

  // ── Software Status Passes ───────────────────────────────────
  // If the code compiles to this point, these libraries are successfully installed!
  printStatus("TinyGPS++ Lib", true, 1);
  printStatus("ST7789 Display Lib", true, 2); 
  printStatus("NeoPixel Lib", true, 3);

  // ── Live I2C Sensor Hardware Component Bus Scans ──────────────
  
  // BMP280 Hardware Scan
  Serial.print("Testing BMP280 on I2C... ");
  if (bmp.begin(0x76)) {
    printStatus("BMP280 Sensor (I2C)", true, 4);
  } else {
    printStatus("BMP280 Sensor (I2C)", false, 4);
  }

  // QMI8658 IMU Hardware Scan
  Serial.print("Testing QMI8658 on I2C... ");
  // Using the clean layout matching the Lahav Gahali library profile
  if (imu.begin(Wire, 0x6B)) {
    printStatus("QMI8658 IMU (I2C)", true, 5);
  } else {
    printStatus("QMI8658 IMU (I2C)", false, 5);
  }

  Serial.println("\n>>> Environment Evaluation Complete. <<<");
}

void loop() {
  // Static checker script - stays here to hold results on the screen
}

// Unified layout routing helper to print status messages out across Serial and TFT channels
void printStatus(String name, bool isFound, int rowNum) {
  int yPos = 25 + (rowNum * 18);
  Serial.print("Target Status: " + name + " -> ");
  
  if (isFound) {
    Serial.println("[PASSED]");
    tft.setCursor(0, yPos);
    tft.setTextColor(0x07E0); // Green
    tft.print("[OK] ");
    tft.setTextColor(0xFFFF); // White
    tft.print(name);
  } else {
    Serial.println("[⚠️ PHYSICAL HARDWARE NOT FOUND]");
    tft.setCursor(0, yPos);
    tft.setTextColor(0xF800); // Red
    tft.print("[!!] ");
    tft.setTextColor(0x7BEF); // Gray out names of uncommunicative hardware components
    tft.print(name);
  }
}