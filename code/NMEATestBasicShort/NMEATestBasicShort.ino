/*
 * BIP26 GPS NMEA Reader - Test Program
 * TenStar ESP32-S3 + Waveshare LC29H GPS Module
 * 
 * Hardware Connections:
 * - GPS TX (Yellow)  → ESP32 GPIO17 (RX)
 * - GPS RX (Blue)    → ESP32 GPIO18 (TX)  
 * - GPS GND (Black)  → ESP32 GND
 * 
 * CRITICAL: GPS Baud Rate is 115200 (NOT 9600!)
 * CRITICAL: Set Serial Monitor to 115200 baud
 */

void setup() {
  // USB Serial to computer (115200 baud)
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("========================================");
  Serial.println("BIP26 GPS NMEA Reader");
  Serial.println("TenStar ESP32-S3 + Waveshare LC29H GPS");
  Serial.println("========================================");
  Serial.println();
  Serial.println("Configuration:");
  Serial.println("  GPS Baud: 115200");
  Serial.println("  RX: GPIO17");
  Serial.println("  TX: GPIO18");
  Serial.println();
  Serial.println("Waiting for GPS data...\n");
  
  // GPS Serial connection
  // Parameters: baud, format, RX pin, TX pin
  // 115200 = baud rate (bits per second) - MUST match GPS configuration
  // SERIAL_8N1 = 8 data bits, No parity, 1 stop bit (standard format)
  // 17 = RX pin (ESP32 receives from GPS TX via yellow wire)
  // 18 = TX pin (ESP32 transmits to GPS RX via blue wire)
  Serial1.begin(115200, SERIAL_8N1, 17, 18);
  
  delay(2000);
}

void loop() {
  // Read and display GPS data
  if (Serial1.available()) {
    String nmea = Serial1.readStringUntil('\n');
    nmea.trim();
    Serial.println(nmea);
  }
}

/*
 * UNDERSTANDING THE KEY NUMBERS:
 * 
 * 115200 (Baud Rate):
 * - Communication speed in bits per second
 * - Your LC29H GPS is configured for 115200 (12× faster than standard 9600)
 * - BOTH devices must use same baud rate or you get gibberish
 * - At 115200 baud: ~11,520 bytes/sec transmission rate
 * 
 * SERIAL_8N1 (Data Format):
 * - 8 = 8 data bits per byte (standard, allows 0-255 values)
 * - N = No parity bit (GPS uses checksums instead)
 * - 1 = 1 stop bit (marks end of byte)
 * - Each byte transmission: START + 8 data bits + STOP = 10 bits total
 * 
 * GPIO 17, 18 (UART Pins):
 * - GPIO17 = RX (Receive) - ESP32 receives data from GPS
 * - GPIO18 = TX (Transmit) - ESP32 sends data to GPS (not used here)
 * - These are the UART1 pins on your TenStar board
 * - GPIO number ≠ physical pin (but on TenStar they match: Pin 17 = GPIO17)
 * 
 * Serial vs Serial1:
 * - Serial  = UART0 (USB to computer) - you see this in Serial Monitor
 * - Serial1 = UART1 (GPIO pins to GPS) - receives NMEA data
 * - Both run at 115200 baud but on different physical connections
 * 
 * Data Flow:
 * GPS (115200 baud) → GPIO17 → ESP32 → USB → Computer (115200 baud)
 * 
 * Expected Output:
 * $GNGLL,5317.377320,N,00621.258036,W,084318.000,A,A*5E
 * $GPGGA,084318.000,5317.377320,N,00621.258036,W,1,08,1.2,35.2,M...
 * $GBGSV,5,1,19,05,04,112,,13,03,077,,10,01,059,,27,01,037,,1*74
 * 
 * Troubleshooting:
 * - Gibberish? Check Serial Monitor is set to 115200 baud
 * - No data? Check wire connections (especially yellow wire to GPIO17)
 * - Still nothing? Press RST button on ESP32 to restart
 */