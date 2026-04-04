/*
* ---------------------------------------------------------------------------------------
* Project Name: Start Project
* Author: [Your Name]
* Date: [Insert Date]
* Version: 0.1
* Description: Simple start project to initialize all onboard modules:
* - TFT Display, BMP280, QMI8658C, RGB-LED
* - Select "ESP32S3 Dev Module" in the Arduino Board Manager.
* - Ensure all required libraries are installed.
*
* Hardware: ESP32-S3 FH4R2, 1.14" TFT Display, BMP280, QMI8658C, WS2812 RGB-LED
*
* Initial Setup: To enter boot mode and programm your ESP32S2:
* 1. Connect the ESP32S3 via USB.
* 2. Hold the "BOOT" button.
* 3. While holding "BOOT", press and release the "RST" button.
* 4. Release "BOOT".
*
* Warning: The RGB LED and QMI8658C sensor cannot be used simultaneously
* when connected to GPIO33, due to a conflict in the QMI library.
* ---------------------------------------------------------------------------------------
*/
/*
* ---------------------------------------------------------------------------------------
* Libraries
* ---------------------------------------------------------------------------------------
*/
// Add or remove as needed
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h> // RGB_LED
#include <Adafruit_GFX.h> // Display
#include <Adafruit_ST7789.h> // Display
#include <Adafruit_BMP280.h> // BMP
#include <SensorQMI8658.hpp> // QMI
/*
* ---------------------------------------------------------------------------------------
* Pin Definitions
* ---------------------------------------------------------------------------------------
*/
// RGB LED
// (GPIO33 conflicts with QMI8658C INT pin, do not use parallel)
#define LED_PIN 33
#define NUM_LEDS 1
// TFT Display Pins
#define TFT_CS 7
#define TFT_DC 39
#define TFT_RST 40
#define TFT_backlight 45
// SPI Pins
#define SPI_SCK 36
#define SPI_MISO 37
#define SPI_MOSI 35
// I2C Pins for Sensors
#define I2C_SDA 42
#define I2C_SCL 41
// Sensor Address
#define BMP_Addr 0x77
#define QMI_Addr 0x6B
/*
* ---------------------------------------------------------------------------------------

* Global Objects/Varibles
* ---------------------------------------------------------------------------------------
*/
// TFT-Display
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
// RGB LED
Adafruit_NeoPixel RGB_LED = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
// BMP Sensor (Temperature and Pressure Sensor)
Adafruit_BMP280 bmp;
// QMI Sensor (6-axis Gyroscope & Accelerometer)
SensorQMI8658 qmi;
// Data structures for QMI Sensor readings
IMUdata acc;
IMUdata gyr;
// Example Variables
int randomCounter = 0;
int randomTime = 500;
/*
* ---------------------------------------------------------------------------------------
* Function Prototypes
* ---------------------------------------------------------------------------------------
*/
void initDisplay();
void initBMP();
void initQMI();
/*
* ---------------------------------------------------------------------------------------
* Setup Function — runs once at startup
* @ brief Example Setup Code. Add and adjust Code as needed:
* ---------------------------------------------------------------------------------------
*/
void setup() {
 // Start serial interface
 Serial.begin(115200);
 // write on serial interface
 Serial.println("Starting...");
 // Start SPI and I2C communication
 SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, TFT_CS);
 Wire.begin(I2C_SDA,I2C_SCL);
 // Initialize TFT Display
 initDisplay();
 // write on TFT Display
 // Cursor position (x,y)
 tft.setCursor(0, 0);
 tft.print("Starting...");
 // Initialize BMP280
 initBMP();
 // Initialize QMI8658C
 initQMI();
 Serial.println("Setup finished");

 // Wait for 1000ms
 delay(1000);
}
/*
* ---------------------------------------------------------------------------------------
* Loop Function — runs repeatedly
* @brief Example Code. Put your own Code here:
* ---------------------------------------------------------------------------------------
*/
//Page 6 of 8
void loop() {
 // Refresh Display
 tft.fillScreen(ST77XX_BLACK);
 // print message on Display (x,y)
 if(randomCounter <= 140){
 tft.setCursor(randomCounter+20, randomCounter);
 tft.print("SENSATE-X");
 randomCounter += 10;
 // Wait for Random Time
 delay(randomTime);
 }
 else{
 randomCounter = rand() % 140;
 tft.setRotation(rand() % 8);
 randomTime = rand() % 500 + 100;
 }
}
/*
* ---------------------------------------------------------------------------------------
* Functions
* ---------------------------------------------------------------------------------------
*/
/**
* @brief Initializes the TFT display, sets rotation, background color, text color and
size.
*/
void initDisplay(){
 // Turn on TFT backlight
 pinMode(TFT_backlight, OUTPUT);
 digitalWrite(TFT_backlight, HIGH);
 // Initialize TFT Display
 tft.init(135, 240); // Width x Height
 tft.setRotation(3); // Adjust as needed
 tft.fillScreen(ST77XX_BLACK);
 tft.setTextColor(ST77XX_WHITE);
 tft.setTextSize(2); // Adjust as needed
}
/**
* @brief Initializes the BMP280 sensor and handles failure if not found.
*
* @note Uses I2C address defined by BMP_ADDR.
*/
void initBMP(){
 // Initialize BMP280 Sensor (address is 0x77)
 if (!bmp.begin(BMP_Addr)) {
 // if Sensor could not be found print error
 tft.setCursor(0, 20);
 tft.print("BMP not found");
 Serial.println("BMP not found");
 // endless loop
 while (1);
 }
}
/**
* @brief Initializes the QMI8658C IMU sensor and configures accelerometer and gyroscope.
*
* @note Uses I2C address defined by QMI_ADDR.
*/
void initQMI(){
 // Initialize QMI8658C Sensor (address is 0x6B)
 if (!qmi.begin(Wire,QMI_Addr,I2C_SDA,I2C_SCL)) {
 // if Sensor could not be found print error
 tft.setCursor(0, 20);
 tft.print("QMI not found");
 Serial.println("QMI not found");
 // endless loop
 while (1);
 }

 // Configure QMI sensor
 qmi.enableGyroscope();
 qmi.enableAccelerometer();
 qmi.configAccelerometer(
 SensorQMI8658::ACC_RANGE_4G,
 SensorQMI8658::ACC_ODR_1000Hz,
 SensorQMI8658::LPF_MODE_0);
 qmi.configGyroscope(
 SensorQMI8658::GYR_RANGE_64DPS,
 SensorQMI8658::GYR_ODR_896_8Hz,
 SensorQMI8658::LPF_MODE_3);
}
/*
* ---------------------------------------------------------------------------------------
* End of file
* ---------------------------------------------------------------------------------------
*/