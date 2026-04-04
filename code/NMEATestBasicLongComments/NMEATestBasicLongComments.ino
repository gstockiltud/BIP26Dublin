/*
 * ================================================================
 * BIP26 GPS NMEA Reader - Test Program
 * ================================================================
 * 
 * TenStar ESP32-S3 + Waveshare LC29H GPS Module
 * 
 * Hardware Connections:
 * - GPS TX (Yellow wire)  → ESP32 GPIO17 (RX)
 * - GPS RX (Blue wire)    → ESP32 GPIO18 (TX)  
 * - GPS GND (Black wire)  → ESP32 GND
 * 
 * This program displays raw NMEA sentences from the GPS module.
 * 
 * CRITICAL CONFIGURATION VALUES:
 * - GPS Baud Rate: 115200 (NOT the standard 9600!)
 * - Serial Monitor Baud: 115200
 * - GPS UART Pins: GPIO17 (RX), GPIO18 (TX)
 * 
 * Author: BIP26 Team
 * Date: April 2026
 * ================================================================
 */

void setup() {
  // Initialize USB Serial for communication with computer
  // 115200 = baud rate (bits per second)
  // This is the speed of ESP32 → Computer communication
  Serial.begin(115200);
  
  delay(1000);  // Wait 1 second for Serial to initialize
  
  // Print startup banner
  Serial.println("========================================");
  Serial.println("BIP26 GPS NMEA Reader");
  Serial.println("TenStar ESP32-S3 + Waveshare LC29H GPS");
  Serial.println("========================================");
  Serial.println();
  Serial.println("Configuration:");
  Serial.println("  GPS Baud: 115200 baud");
  Serial.println("  RX: GPIO17");
  Serial.println("  TX: GPIO18");
  Serial.println();
  Serial.println("Waiting for GPS data...\n");
  
  // Initialize Serial1 for GPS communication
  // Serial1.begin(baud_rate, format, RX_pin, TX_pin)
  // 
  // Parameters explained:
  // 115200        = Baud rate (bits per second)
  //                 CRITICAL: Your LC29H GPS is configured for 115200
  //                 NOT the standard 9600 that most GPS tutorials show!
  // 
  // SERIAL_8N1    = Data format (8 data bits, No parity, 1 stop bit)
  //                 8 = 8 data bits per byte (standard)
  //                 N = No parity bit (error checking disabled)
  //                 1 = 1 stop bit (marks end of byte)
  //                 This is the most common UART format
  // 
  // 17            = RX pin (ESP32 receives data on GPIO17)
  //                 Yellow wire from GPS TX connects here
  //                 GPS transmits → ESP32 receives
  // 
  // 18            = TX pin (ESP32 transmits data on GPIO18)
  //                 Blue wire from GPS RX connects here
  //                 ESP32 transmits → GPS receives
  //                 (We don't send commands in this program, but pin still configured)
  Serial1.begin(115200, SERIAL_8N1, 17, 18);
  
  delay(2000);  // Wait 2 seconds for GPS to start sending data
}

void loop() {
  // Check if GPS has sent any data
  // Serial1.available() returns the number of bytes waiting to be read
  // Returns 0 if no data available
  if (Serial1.available()) {
    
    // Read one complete line from GPS (until newline character '\n')
    // NMEA sentences end with \r\n (carriage return + line feed)
    // readStringUntil('\n') reads everything up to and including '\n'
    String nmea = Serial1.readStringUntil('\n');
    
    // Remove any whitespace (spaces, \r, \n) from beginning and end
    // This cleans up the NMEA sentence for display
    nmea.trim();
    
    // Print the NMEA sentence to Serial Monitor
    // You should see sentences like:
    // $GNGGA,123519,5317.377,N,00621.258,W,1,12,1.0,35.2,M,53.0,M,,*5E
    // $GNGLL,5317.377320,N,00621.258036,W,084318.000,A,A*5E
    // $GBGSV,5,1,19,05,04,112,,13,03,077,,10,01,059,,27,01,037,,1*74
    Serial.println(nmea);
  }
  
  // Loop repeats continuously, reading and displaying GPS data as it arrives
  // No delay needed - loop runs as fast as possible to catch all GPS data
}

/*
 * ================================================================
 * UNDERSTANDING THE NUMBERS:
 * ================================================================
 * 
 * BAUD RATE (115200):
 * -------------------
 * - Baud = symbols (bits) transmitted per second
 * - 115200 baud = 115,200 bits per second
 * - This is the communication speed between ESP32 and GPS
 * - Both devices MUST use the same baud rate or you get gibberish!
 * 
 * Why 115200?
 * - Your specific LC29H GPS module is configured for this speed
 * - Faster than standard 9600 baud (12× faster!)
 * - Allows more data to be transmitted (useful for multi-GNSS)
 * - Common "high speed" baud rate for GPS modules
 * 
 * Other common baud rates:
 * - 9600   = Standard GPS (slow but reliable)
 * - 38400  = Some GPS modules use this
 * - 57600  = Medium speed
 * - 115200 = Your GPS uses this (fast)
 * - 230400 = Very fast (rare)
 * 
 * 
 * SERIAL FORMAT (SERIAL_8N1):
 * ---------------------------
 * This describes how each byte is transmitted:
 * 
 * 8 = 8 data bits
 *     - Each character/byte is sent as 8 bits
 *     - Standard for modern serial communication
 *     - Allows 256 different values (0-255)
 *     - Enough for all ASCII characters
 * 
 * N = No parity
 *     - Parity is an error-checking bit
 *     - "N" means no parity bit is used
 *     - Alternatives: Even parity (E), Odd parity (O)
 *     - GPS uses no parity (NMEA has checksums instead)
 * 
 * 1 = 1 stop bit
 *     - Stop bit marks the end of each byte
 *     - Gives receiver time to prepare for next byte
 *     - Alternative: 2 stop bits (slower, more reliable)
 *     - 1 stop bit is standard
 * 
 * Visual representation of one byte transmission:
 * 
 * START | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | STOP
 *   ↑     └────────── 8 data bits ──────────┘     ↑
 *   │                                              │
 * Start bit                                    Stop bit
 * (always 0)                                   (always 1)
 * 
 * Total bits per byte: 1 start + 8 data + 1 stop = 10 bits
 * 
 * At 115200 baud:
 * - 115200 bits/sec ÷ 10 bits/byte = 11,520 bytes/sec
 * - 11,520 bytes/sec ÷ 80 chars/NMEA = ~144 NMEA sentences/sec (theoretical max)
 * - GPS typically sends ~10-20 sentences/sec (1 Hz update rate)
 * 
 * 
 * GPIO PIN NUMBERS (17, 18):
 * --------------------------
 * GPIO = General Purpose Input/Output
 * 
 * 17 = GPIO17 (RX - Receive)
 *      - ESP32 receives data on this pin
 *      - Connected to GPS TX (transmit) pin
 *      - Yellow wire in your setup
 *      - Data flows: GPS TX → ESP32 GPIO17 RX
 * 
 * 18 = GPIO18 (TX - Transmit)
 *      - ESP32 transmits data on this pin
 *      - Connected to GPS RX (receive) pin
 *      - Blue wire in your setup
 *      - Data flows: ESP32 GPIO18 TX → GPS RX
 *      - (Not used in this program - we only read from GPS)
 * 
 * Important notes about GPIO numbers:
 * - GPIO number ≠ physical pin number (on most ESP32 boards)
 * - On TenStar: GPIO17 happens to be Pin 17 (lucky match!)
 * - On TenStar: GPIO18 happens to be Pin 18 (lucky match!)
 * - Always use GPIO numbers in code, not physical pin positions
 * 
 * Why these specific GPIOs?
 * - These are the pins where UART1 is connected on your TenStar board
 * - Different ESP32 boards may use different GPIO for UART
 * - Verified by testing - these work on your hardware
 * 
 * 
 * SERIAL vs SERIAL1:
 * ------------------
 * ESP32 has multiple UART (serial) interfaces:
 * 
 * Serial  = UART0 (USB connection to computer)
 *           - Used for Serial Monitor
 *           - Baud: 115200 (matches Serial Monitor setting)
 *           - You see output here
 * 
 * Serial1 = UART1 (GPIO pins for external devices)
 *           - Used for GPS module
 *           - Baud: 115200 (matches GPS configuration)
 *           - RX on GPIO17, TX on GPIO18
 * 
 * Serial2 = UART2 (available but not used in this project)
 * 
 * 
 * DATA FLOW:
 * ----------
 * GPS Module (LC29H)
 *    ↓
 *    Transmits NMEA at 115200 baud via yellow wire
 *    ↓
 * ESP32 GPIO17 (Serial1 RX)
 *    ↓
 *    ESP32 receives and processes data
 *    ↓
 * ESP32 USB (Serial TX)
 *    ↓
 *    Transmits to computer at 115200 baud
 *    ↓
 * Computer Serial Monitor (set to 115200 baud)
 *    ↓
 *    You see NMEA sentences!
 * 
 * ================================================================
 */