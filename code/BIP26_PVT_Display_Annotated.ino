// ============================================================
//  BIP26_PVT_Display_Annotated.ino
//  BIP26 SENSATE-X 2.0  |  TU Dublin Tallaght Campus
// ============================================================
//
//  WHAT THIS PROGRAM DOES
//  ----------------------
//  This program reads GNSS data from the Waveshare LC29H(AA)
//  receiver and displays it on the Tenstar TFT screen.
//
//  PVT stands for Position, Velocity, Time — the three core
//  outputs of any GNSS receiver:
//
//    POSITION  →  Latitude, Longitude, Altitude
//    VELOCITY  →  Speed and direction of travel
//    TIME      →  Accurate UTC date and time from satellites
//
//  The LC29H sends data as NMEA sentences over a serial (UART)
//  connection. We use the TinyGPSPlus library to parse these
//  sentences automatically — we just feed it one character at
//  a time and it handles the rest.
//
//  HARDWARE REQUIRED
//  -----------------
//    Tenstar TS-ESP32-S3 development board
//    Waveshare LC29H(AA) GNSS HAT
//    External GNSS antenna connected to the LC29H IPX connector
//    USB cable for power and programming
//
//  WIRING (three wires only)
//  -------------------------
//    LC29H Pin 6  (GND)    →  Tenstar GND
//    LC29H Pin 8  (TXD0)   →  Tenstar GPIO 17  (front label A1)
//    LC29H Pin 10 (RXD0)   →  Tenstar GPIO 18  (front label A0)
//
//    The LC29H is powered by its own USB cable — do NOT connect
//    3.3V from the Tenstar to the LC29H.
//
//  LIBRARIES NEEDED (install via Sketch → Manage Libraries)
//  ---------------------------------------------------------
//    Adafruit ST7735 and ST7789 Library
//    Adafruit GFX Library
//    TinyGPSPlus
//
//  ARDUINO IDE SETTINGS (Tools menu)
//  ----------------------------------
//    Board:        ESP32S3 Dev Module
//    Flash Size:   4MB (32Mb)
//    Upload Mode:  UART0 / Hardware CDC
//    USB Mode:     Hardware CDC and JTAG
//    USB CDC On Boot: Enabled
//
//  AFTER UPLOADING
//  ---------------
//    Press the RST button on the Tenstar board to start the
//    sketch. The display will show "Waiting for fix..." until
//    the LC29H acquires satellite signals — this can take
//    1 to 2 minutes, especially on a cold start.
//
// ============================================================

// ============================================================
//  SECTION 1 — LIBRARIES
//  ============================================================
//  #include tells the compiler to add a library to your sketch.
//  Libraries are collections of pre-written code that handle
//  complex tasks so you do not have to write them yourself.

#include <Arduino.h>        // Core Arduino functions (pinMode, digitalWrite etc.)
#include <SPI.h>            // SPI communication protocol — used by the TFT display
#include <Wire.h>           // I2C communication protocol — used by onboard sensors
#include <Adafruit_GFX.h>   // Graphics library: lines, rectangles, text drawing
#include <Adafruit_ST7789.h>// Driver for the ST7789 TFT display chip on the Tenstar
#include <TinyGPSPlus.h>    // Parses NMEA sentences from the LC29H GPS module

// ============================================================
//  SECTION 2 — PIN DEFINITIONS
//  ============================================================
//  #define gives a name to a number so the code is easier to
//  read. Instead of writing 45 everywhere we write TFT_backlight,
//  which makes the code self-documenting.
//
//  These pin numbers come from the Tenstar board documentation
//  (Mike Gill's setup guide) and have been confirmed working.

// TFT Display pins — these connect the ESP32-S3 to the display chip
#define TFT_CS         7    // Chip Select: tells the display "this message is for you"
#define TFT_DC        39    // Data/Command: HIGH = sending data, LOW = sending a command
#define TFT_RST       40    // Reset: briefly pulled LOW to restart the display chip
#define TFT_backlight 45    // Backlight control: HIGH turns the screen light on

// SPI bus pins — the communication channel between the ESP32-S3 and the TFT
// These are NOT the default ESP32-S3 SPI pins — the Tenstar uses custom routing
#define SPI_SCK       36    // Serial Clock: the timing signal for SPI communication
#define SPI_MISO      37    // Master In Slave Out: data coming back from display (rarely used)
#define SPI_MOSI      35    // Master Out Slave In: data going from ESP32-S3 to display

// I2C bus pins — used for the BMP280 pressure sensor and QMI8658C IMU
// Not actively used in this sketch but good practice to define them
#define I2C_SDA       42    // Serial Data line for I2C
#define I2C_SCL       41    // Serial Clock line for I2C

// GPS UART pins — the serial connection to the Waveshare LC29H
#define GPS_RX        17    // Receives NMEA data FROM the LC29H (yellow wire, front label A1)
#define GPS_TX        18    // Sends commands TO the LC29H (blue wire, front label A0)

// ============================================================
//  SECTION 3 — CONSTANTS
//  ============================================================
//  Constants are values that do not change while the program runs.
//  Grouping them here makes it easy to adjust behaviour without
//  hunting through the code.

#define GPS_BAUD    115200  // Speed of the serial link to the LC29H (bits per second)
                            // Both ends MUST use the same baud rate or gibberish results

#define SCREEN_W       240  // TFT screen width in landscape orientation (pixels)
#define SCREEN_H       135  // TFT screen height in landscape orientation (pixels)
                            // The physical display is 135x240 portrait, but setRotation(3)
                            // gives us a 240 wide x 135 tall landscape layout

#define REFRESH_MS    1000  // How often to update the display (milliseconds)
                            // 1000ms = 1 second — matches the LC29H's 1Hz NMEA output rate

// ============================================================
//  SECTION 4 — COLOUR DEFINITIONS
//  ============================================================
//  The ST7789 display uses 16-bit RGB565 colour format.
//  Each colour is stored as a 16-bit number: 5 bits red,
//  6 bits green, 5 bits blue.
//
//  Format: 0xRRRRRGGGGGGBBBBB
//
//  You can convert any RGB colour to RGB565 using an online
//  converter such as: http://www.barth-dev.de/online/rgb565-color-picker/

#define BLACK   0x0000  // R=0,  G=0,  B=0   — used for background
#define WHITE   0xFFFF  // R=31, G=63, B=31  — bright white text
#define NAVY    0x000F  // R=0,  G=0,  B=15  — dark blue header background
#define CYAN    0x07FF  // R=0,  G=63, B=31  — bright cyan for time display
#define GOLD    0xFEA0  // R=31, G=53, B=0   — gold for header text and altitude
#define GREEN   0x07E0  // R=0,  G=63, B=0   — bright green for good HDOP / fix OK
#define AMBER   0xFD20  // R=31, G=41, B=0   — amber for moderate HDOP
#define RED     0xF800  // R=31, G=0,  B=0   — red for poor HDOP / no fix
#define GREY    0x8410  // R=16, G=32, B=16  — grey for labels and dividers
#define LTGREY  0xC618  // R=24, G=48, B=24  — lighter grey for secondary labels
#define DKGREY  0x2104  // R=4,  G=8,  B=4   — very dark grey for subtle backgrounds

// ============================================================
//  SECTION 5 — OBJECTS
//  ============================================================
//  An object is a variable that represents a piece of hardware
//  and provides functions (methods) to control it.
//
//  For example, tft.fillScreen(BLACK) tells the TFT object to
//  fill the entire screen with black — you do not need to know
//  how SPI communication works to use it.

// TFT display object
// The constructor takes pin numbers to tell it how it is connected
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// TinyGPSPlus object — parses NMEA sentences
// Call gps.encode(character) to feed it GPS data one byte at a time
// It updates its internal fields (gps.location, gps.time etc.) automatically
TinyGPSPlus gps;

// HardwareSerial object for UART communication with the LC29H
// The ESP32-S3 has three hardware UARTs: Serial (USB), Serial1, Serial2
// We use Serial1 so it does not conflict with the USB Serial Monitor
HardwareSerial GPSSerial(1);

// ============================================================
//  SECTION 6 — GLOBAL VARIABLES
//  ============================================================
//  Variables declared here are accessible from all functions.
//  Keep this section small — only variables that need to be
//  shared between setup() and loop() go here.

unsigned long lastUpdate = 0;
// millis() returns the number of milliseconds since the board started.
// We store the time of the last display update here so we can calculate
// when REFRESH_MS milliseconds have passed and it is time to update again.
// unsigned long can hold numbers up to about 4.29 billion — enough for
// about 49 days of continuous running before it wraps around.

// ============================================================
//  SECTION 7 — FUNCTION DECLARATIONS
//  ============================================================
//  In C/C++, a function must be declared before it can be called.
//  Declaring it here (just the name and parameters, not the body)
//  tells the compiler the function exists even before it is defined
//  later in the file.

void initDisplay();   // Sets up the TFT display hardware
void drawScreen();    // Draws all PVT data onto the display

// ============================================================
//  SETUP FUNCTION
//  ============================================================
//  setup() runs ONCE when the board powers on or is reset.
//  Use it to initialise hardware, configure settings, and
//  draw any elements that only need to appear once.

void setup() {

  // Start the USB Serial port for debug output to the Serial Monitor
  // 115200 baud — make sure the Serial Monitor baud rate matches this
  Serial.begin(115200);
  Serial.println("BIP26 PVT Display starting...");

  // IMPORTANT: SPI.begin() must be called BEFORE tft.init()
  // This initialises the SPI bus with the custom Tenstar pin numbers.
  // The default ESP32-S3 SPI pins are different — without this call
  // the display will not work.
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, TFT_CS);

  // Start the I2C bus (used by BMP280 and QMI8658C sensors on the board)
  // Not needed for GPS display but good practice to initialise it
  Wire.begin(I2C_SDA, I2C_SCL);

  // Initialise and clear the TFT display
  // This is in a separate function to keep setup() tidy
  initDisplay();

  // Draw the initial screen layout — shows "Waiting for fix..." message
  drawScreen();

  // Start GPS communication on UART1
  // Parameters: baud rate, data format, RX pin, TX pin
  // SERIAL_8N1 = 8 data bits, No parity, 1 stop bit (standard setting)
  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);

  Serial.println("Setup complete. Waiting for GPS fix...");
  Serial.println("Tip: Open Serial Monitor at 115200 baud to see NMEA data.");
}

// ============================================================
//  LOOP FUNCTION
//  ============================================================
//  loop() runs REPEATEDLY and FOREVER after setup() completes.
//  It is called thousands of times per second.
//  Keep it fast — avoid long delays inside loop().

void loop() {

  // ── Read GPS data ────────────────────────────────────────
  //
  // GPSSerial.available() returns the number of characters
  // waiting to be read from the GPS module.
  //
  // We read and process EVERY available character on each
  // call to loop(). This is important because NMEA sentences
  // arrive continuously and we must not miss any characters.
  //
  // gps.encode(c) feeds one character at a time to TinyGPSPlus.
  // Internally, TinyGPSPlus:
  //   1. Assembles characters into complete NMEA sentences
  //   2. Validates the checksum of each sentence
  //   3. Parses the fields and updates its data fields
  //   4. Returns true when a complete sentence was processed
  //
  // After this while loop completes, gps.location, gps.time
  // etc. are updated with the latest available data.

  while (GPSSerial.available()) {
    char c = GPSSerial.read();
    gps.encode(c);
  }

  // ── Update display once per second ───────────────────────
  //
  // millis() - lastUpdate gives the number of milliseconds
  // since the display was last refreshed.
  // When this exceeds REFRESH_MS (1000ms = 1 second) we
  // redraw the screen with the latest GPS data.
  //
  // This non-blocking timing technique (using millis) is
  // preferred over delay(1000) because delay() stops
  // EVERYTHING — including reading GPS data — for 1 second,
  // which would cause us to miss incoming NMEA sentences.

  if (millis() - lastUpdate >= REFRESH_MS) {
    lastUpdate = millis();  // Record when this update happened
    drawScreen();           // Redraw the TFT with latest data

    // Also print a summary to the Serial Monitor for debugging
    // This is useful to see data even before the display is working
    if (gps.location.isValid()) {
      Serial.print("LAT: "); Serial.print(gps.location.lat(), 6);
      Serial.print("  LON: "); Serial.print(gps.location.lng(), 6);
      Serial.print("  ALT: "); Serial.print(gps.altitude.meters(), 1);
      Serial.println(" m");
    }

    // gps.charsProcessed() counts total bytes received from GPS
    // If this stays at zero after 10 seconds, check your wiring
    if (gps.charsProcessed() == 0) {
      Serial.println("WARNING: No data from GPS — check wiring and jumper position");
    }
  }
}

// ============================================================
//  FUNCTION: initDisplay()
//  ============================================================
//  Sets up the TFT display ready for use.
//  Called once from setup().
//
//  The order of operations here is important:
//    1. Turn backlight on
//    2. Initialise the display chip with its physical dimensions
//    3. Set the rotation (landscape)
//    4. Clear the screen

void initDisplay() {

  // The backlight is controlled by a GPIO pin connected to the
  // LED driver circuit on the Tenstar board.
  // HIGH = backlight on, LOW = backlight off (screen dark)
  pinMode(TFT_backlight, OUTPUT);
  digitalWrite(TFT_backlight, HIGH);

  // tft.init() sends the startup sequence to the ST7789 display chip.
  // Parameters are the PHYSICAL dimensions of the display panel:
  //   135 pixels wide (portrait width)
  //   240 pixels tall (portrait height)
  // These must match the actual panel size — wrong values cause
  // a corrupted or shifted image.
  tft.init(135, 240);

  // setRotation() controls how the image is oriented on screen.
  // 0 = portrait (135 wide x 240 tall)
  // 1 = landscape, rotated 90° clockwise (240 wide x 135 tall)
  // 2 = portrait, upside down
  // 3 = landscape, rotated 270° clockwise (240 wide x 135 tall)
  // We use rotation 3 for landscape with the USB-C connector on the right.
  tft.setRotation(3);

  // Fill the entire screen with black to clear any previous content
  tft.fillScreen(BLACK);

  // Disable automatic text wrapping — text that goes off the right
  // edge of the screen is simply cut off rather than wrapping to
  // the next line, which would push other content out of place.
  tft.setTextWrap(false);
}

// ============================================================
//  FUNCTION: drawScreen()
//  ============================================================
//  Draws all PVT data onto the TFT display.
//  Called every second from loop().
//
//  The screen is divided into zones:
//
//    Y  0- 16  Header bar (navy background, gold title)
//    Y 17- 32  UTC Time and Date (cyan)
//    Y 33- 33  Grey divider line
//    Y 34- 77  Position (latitude, longitude, altitude)
//    Y 78- 78  Grey divider line
//    Y 79-106  Velocity (speed and course)
//    Y107-107  Grey divider line
//    Y108-134  Status bar (satellites, HDOP, fix indicator)
//
//  Each zone is cleared with fillRect() before redrawing.
//  This avoids having to clear the whole screen (which causes
//  a visible flash) while still updating the data cleanly.

void drawScreen() {

  // ── HEADER BAR ───────────────────────────────────────────
  // Draw a navy rectangle across the top of the screen
  // fillRect(x, y, width, height, colour)
  tft.fillRect(0, 0, SCREEN_W, 16, NAVY);

  // Draw a gold horizontal line under the header
  // drawFastHLine(x, y, width, colour) — faster than drawLine for horizontal
  tft.drawFastHLine(0, 16, SCREEN_W, GOLD);

  // Print the title in the header
  // setTextSize(1) = smallest built-in font (6x8 pixels per character)
  tft.setTextSize(1);
  tft.setTextColor(GOLD);
  tft.setCursor(4, 4);               // setCursor(x, y) — x from left, y from top
  tft.print("GNSS PVT DISPLAY");

  // Print a subtitle on the right side of the header
  tft.setTextColor(GREY);
  tft.setCursor(130, 4);
  tft.print("SENSATE-X 2.0");

  // ── TIME AND DATE (the T in PVT) ─────────────────────────
  // Clear this zone before redrawing — prevents old text ghosting
  // fillRect(x, y, width, height, colour)
  tft.fillRect(0, 17, SCREEN_W, 15, BLACK);

  tft.setTextSize(1);

  // gps.time.isValid() returns true once TinyGPSPlus has received
  // and parsed a complete, valid time fix from the NMEA stream.
  // Before a fix, this returns false and we show a waiting message.
  //
  // UTC (Coordinated Universal Time) is the time standard used by
  // GNSS. Ireland is UTC+0 in winter and UTC+1 (IST) in summer.
  // The GPS receiver always outputs UTC — your code must add the
  // local offset if you want to show local time.

  if (gps.time.isValid() && gps.date.isValid()) {

    // Build a formatted time/date string using snprintf
    // snprintf(buffer, max_length, format_string, values...)
    // Format specifiers: %02d = decimal integer, at least 2 digits, zero-padded
    char timeBuf[36];
    snprintf(timeBuf, sizeof(timeBuf),
             "%02d:%02d:%02d UTC    %02d/%02d/%04d",
             gps.time.hour(),    // 0-23
             gps.time.minute(),  // 0-59
             gps.time.second(),  // 0-59
             gps.date.day(),     // 1-31
             gps.date.month(),   // 1-12
             gps.date.year());   // e.g. 2026

    tft.setTextColor(CYAN);
    tft.setCursor(4, 20);
    tft.print(timeBuf);

  } else {
    // No valid time yet — show a waiting message
    tft.setTextColor(GREY);
    tft.setCursor(4, 20);
    tft.print("Waiting for time fix...");
  }

  // Grey divider line between time and position zones
  tft.drawFastHLine(0, 33, SCREEN_W, GREY);

  // ── POSITION (the P in PVT) ──────────────────────────────
  // Clear the position zone
  tft.fillRect(0, 34, SCREEN_W, 43, BLACK);
  tft.setTextSize(1);

  // gps.location.isValid() returns true once a full position fix
  // is calculated from at least 3 satellites (2D) or 4+ (3D).
  // The LC29H typically achieves a fix within 60 seconds outdoors.

  if (gps.location.isValid()) {

    // ── Latitude ─────────────────────────────────────────
    // Latitude measures north-south position.
    // Range: -90° (South Pole) to +90° (North Pole)
    // Dublin is approximately 53.33° North
    // gps.location.lat() returns decimal degrees (e.g. 53.333456)
    // Positive = North, Negative = South

    tft.setTextColor(GREY);
    tft.setCursor(2, 36);
    tft.print("LAT:");

    // fabs() = floating point absolute value (removes minus sign)
    // We show N/S separately instead of using negative numbers
    char latBuf[18];
    snprintf(latBuf, sizeof(latBuf), "%10.6f %c",
             fabs(gps.location.lat()),
             gps.location.lat() >= 0 ? 'N' : 'S');

    tft.setTextColor(WHITE);
    tft.setCursor(30, 36);
    tft.print(latBuf);

    // ── Longitude ─────────────────────────────────────────
    // Longitude measures east-west position.
    // Range: -180° (West) to +180° (East)
    // Dublin is approximately -6.27° (6.27° West)
    // Positive = East, Negative = West

    tft.setTextColor(GREY);
    tft.setCursor(2, 49);
    tft.print("LON:");

    char lonBuf[18];
    snprintf(lonBuf, sizeof(lonBuf), "%10.6f %c",
             fabs(gps.location.lng()),
             gps.location.lng() >= 0 ? 'E' : 'W');

    tft.setTextColor(WHITE);
    tft.setCursor(30, 49);
    tft.print(lonBuf);

    // ── Altitude ──────────────────────────────────────────
    // Altitude is measured above Mean Sea Level (AMSL).
    // TU Dublin Tallaght campus is approximately 80-90 m AMSL.
    // gps.altitude.isValid() may remain false slightly longer
    // than gps.location.isValid() — altitude needs a 3D fix.

    tft.setTextColor(GREY);
    tft.setCursor(2, 62);
    tft.print("ALT:");

    if (gps.altitude.isValid()) {
      char altBuf[18];
      snprintf(altBuf, sizeof(altBuf), "%.1f m AMSL",
               gps.altitude.meters());
      tft.setTextColor(GOLD);
      tft.setCursor(30, 62);
      tft.print(altBuf);
    } else {
      tft.setTextColor(GREY);
      tft.setCursor(30, 62);
      tft.print("Acquiring 3D fix...");
    }

  } else {
    // No position fix yet
    tft.setTextColor(GREY);
    tft.setCursor(4, 52);
    tft.print("Acquiring position fix...");
    tft.setCursor(4, 64);
    tft.print("(antenna near window?)");
  }

  tft.drawFastHLine(0, 78, SCREEN_W, GREY);

  // ── VELOCITY (the V in PVT) ───────────────────────────────
  // Clear the velocity zone
  tft.fillRect(0, 79, SCREEN_W, 27, BLACK);
  tft.setTextSize(1);

  // ── Speed ─────────────────────────────────────────────────
  // Speed is calculated by the receiver from successive position
  // fixes. Even when static, the receiver reports a small non-zero
  // speed (typically 0.1 to 0.5 km/h) due to position noise.
  // In practice, speeds below 1 km/h can be treated as stationary.
  //
  // The LC29H reports speed in knots natively (NMEA standard).
  // TinyGPSPlus converts it for us:
  //   gps.speed.knots() — nautical speed
  //   gps.speed.kmph()  — kilometres per hour
  //   gps.speed.mph()   — miles per hour

  tft.setTextColor(GREY);
  tft.setCursor(2, 82);
  tft.print("SPEED:");

  if (gps.speed.isValid()) {
    char spdBuf[28];
    snprintf(spdBuf, sizeof(spdBuf), "%.1f km/h   %.1f knots",
             gps.speed.kmph(),
             gps.speed.knots());
    tft.setTextColor(WHITE);
    tft.setCursor(46, 82);
    tft.print(spdBuf);
  } else {
    tft.setTextColor(GREY);
    tft.setCursor(46, 82);
    tft.print("--");
  }

  // ── Course ────────────────────────────────────────────────
  // Course Over Ground (COG) is the direction of travel in
  // degrees measured clockwise from True North.
  //   0°  = heading North
  //   90° = heading East
  //   180°= heading South
  //   270°= heading West
  //
  // Note: course is only meaningful when the receiver is moving.
  // At very low speeds the course reading is unreliable.

  tft.setTextColor(GREY);
  tft.setCursor(2, 95);
  tft.print("COURSE:");

  if (gps.course.isValid()) {
    char crsBuf[14];
    snprintf(crsBuf, sizeof(crsBuf), "%.1f%c True North",
             gps.course.deg(),
             (char)248);   // ASCII 248 = degree symbol °
    tft.setTextColor(WHITE);
    tft.setCursor(54, 95);
    tft.print(crsBuf);
  } else {
    tft.setTextColor(GREY);
    tft.setCursor(54, 95);
    tft.print("--");
  }

  tft.drawFastHLine(0, 107, SCREEN_W, GREY);

  // ── STATUS BAR ────────────────────────────────────────────
  // Clear the status zone and show quality indicators
  tft.fillRect(0, 108, SCREEN_W, SCREEN_H - 108, BLACK);
  tft.setTextSize(1);

  // ── Satellites ────────────────────────────────────────────
  // gps.satellites.value() returns the number of satellites
  // being USED in the current position fix (from the GSA sentence).
  // This is different from satellites IN VIEW (from GSV sentences)
  // which may be more. A fix needs at least 4 satellites.

  tft.setTextColor(GREY);
  tft.setCursor(2, 111);
  tft.print("SATS:");

  tft.setTextColor(WHITE);
  tft.setCursor(36, 111);
  if (gps.satellites.isValid()) {
    char satBuf[4];
    snprintf(satBuf, sizeof(satBuf), "%2d", gps.satellites.value());
    tft.print(satBuf);
  } else {
    tft.print("--");
  }

  // ── HDOP ──────────────────────────────────────────────────
  // HDOP = Horizontal Dilution of Precision
  // A measure of how well the visible satellites are geometrically
  // positioned to give an accurate horizontal fix.
  //
  // Low HDOP = satellites spread across the sky = good geometry = accurate fix
  // High HDOP = satellites clustered together = poor geometry = less accurate
  //
  // HDOP interpretation:
  //   < 1.0  = Ideal
  //   1-2    = Excellent
  //   2-5    = Good (suitable for most applications)
  //   5-10   = Moderate
  //   > 10   = Poor
  //
  // We colour-code it green/amber/red to give an instant visual indication.

  tft.setTextColor(GREY);
  tft.setCursor(72, 111);
  tft.print("HDOP:");

  if (gps.hdop.isValid()) {
    float h = gps.hdop.hdop();

    // Choose colour based on HDOP quality
    if      (h < 2.0) tft.setTextColor(GREEN);
    else if (h < 5.0) tft.setTextColor(AMBER);
    else               tft.setTextColor(RED);

    char hdopBuf[6];
    snprintf(hdopBuf, sizeof(hdopBuf), "%.1f", h);
    tft.setCursor(108, 111);
    tft.print(hdopBuf);
  } else {
    tft.setTextColor(GREY);
    tft.setCursor(108, 111);
    tft.print("--");
  }

  // ── Fix type ──────────────────────────────────────────────
  // Shows whether we have a valid 3D fix or not.
  // 3D fix = latitude + longitude + altitude (4+ satellites needed)
  // 2D fix = latitude + longitude only (3 satellites, less accurate)
  // No fix = receiver is still searching for signals

  tft.setTextColor(GREY);
  tft.setCursor(148, 111);
  tft.print("FIX:");

  tft.setCursor(178, 111);
  if (gps.location.isValid()) {
    tft.setTextColor(GREEN);
    tft.print(" 3D");
  } else {
    tft.setTextColor(RED);
    tft.print(" NO");
  }

  // ── Diagnostic footer ─────────────────────────────────────
  // gps.charsProcessed() = total bytes received from GPS since boot
  // gps.sentencesWithFix() = NMEA sentences that contained a valid fix
  //
  // This is extremely useful for debugging:
  // If charsProcessed = 0 after 10 seconds → check wiring
  // If charsProcessed > 0 but no fix → receiver is working,
  //   just waiting for enough satellites — give it more time

  tft.setTextColor(DKGREY);
  tft.setCursor(2, 124);
  char diagBuf[36];
  snprintf(diagBuf, sizeof(diagBuf), "Chars: %lu  Fixes: %lu",
           gps.charsProcessed(),
           gps.sentencesWithFix());
  tft.print(diagBuf);
}

// ============================================================
//  END OF SKETCH
// ============================================================
//
//  THINGS TO TRY NEXT
//  ------------------
//  1. Add a second serial print in loop() to see raw NMEA in
//     the Serial Monitor: Serial.println(GPSSerial.readStringUntil('\n'));
//
//  2. Add a speed threshold: if speed < 1.0 km/h, show "STATIC"
//     instead of the small noise value
//
//  3. Display the number of Galileo satellites separately by
//     parsing $GAGSV sentences (see BIP26_GalileoTrackerV3.ino)
//
//  4. Add the BMP280 temperature and pressure to the display
//     using the Adafruit_BMP280 library
//
//  5. Try calculating your distance from a fixed reference point
//     (e.g. TU Dublin) using the TinyGPSPlus distanceBetween()
//     and courseTo() functions
//
// ============================================================
