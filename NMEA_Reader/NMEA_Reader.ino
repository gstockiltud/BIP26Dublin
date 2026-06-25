/* * ==============================================================================
 * STMP26 SENSATE GPS NMEA Reader - FINAL WORKING LAB REFERENCE
 * Target Hardware: TenStar ESP32-S3 + Waveshare LC29H GPS Module
 * Recommended Software Environment: Arduino IDE 2.x.x
 * ==============================================================================
 * * 🛠️ ENGINEERING RULES OF THE ROAD (READ BEFORE FLASHING):
 * * 1. THE ENVIRONMENT VARIABILITY FACTOR: 
 * Depending on whether you are running Windows (and if it's a personal laptop 
 * or a locked-down institution machine), a Mac, or Linux, the Arduino IDE 2.x 
 * environment may interact differently with your hardware. If your port 
 * disappears or throws an "Access Denied" error, check your OS device drivers.
 * * 2. THE FIRST LAW OF ENGINEERING SKEPTICISM:
 * As an engineer, you must assume absolutely nothing works until YOU have personally 
 * tested it and verified the data stream. It does not matter if your lecturer 
 * swears it works, it does not matter if a generative AI confidently tells you 
 * it works, and it definitely does not matter if your lab partner says "it worked 
 * on my machine five minutes ago." Trust your oscilloscope, trust your serial 
 * monitor, and verify everything!
 * * ==============================================================================
 */

void setup() {
  // Initialize the primary USB serial communication link back to your computer.
  // This allows the ESP32-S3 to send text to the Arduino IDE Serial Monitor.
  Serial.begin(115200);  
  
  // A brief delay to allow the hardware serial interface to stabilize after power-up.
  delay(1000);
  
  // Print diagnostic headers to the Serial Monitor so we know the firmware booted successfully.
  Serial.println("========================================");
  Serial.println("STMP26 GPS NMEA Reader");
  Serial.println("TenStar ESP32-S3 + Waveshare LC29H GPS");
  Serial.println("========================================");
  Serial.println();
  Serial.println("Configuration:");
  Serial.println("  GPS Baud: 115200");
  Serial.println("  RX: GPIO17");
  Serial.println("  TX: GPIO18");
  Serial.println();
  Serial.println("Waiting for GPS data...\n");
  
  // Initialize Hardware Serial 1 (Serial1) to communicate directly with the GPS.
  // CRITICAL PARAMETERS EXPLAINED:
  // - 115200: The default baud rate of the Waveshare LC29H (Note: standard GPS modules use 9600).
  // - SERIAL_8N1: 8 data bits, no parity, 1 stop bit (the standard serial protocol configuration).
  // - 17: The ESP32 GPIO pin assigned as RX (Receiving data FROM the GPS module's TX pin).
  // - 18: The ESP32 GPIO pin assigned as TX (Transmitting data TO the GPS module's RX pin).
  Serial1.begin(115200, SERIAL_8N1, 17, 18);
  
  // Give the serial link to the GPS module a brief moment to open cleanly before reading.
  delay(2000);
}

void loop() {
  // Check if there are bytes waiting in the hardware buffer received from the GPS module.
  if (Serial1.available()) {
    
    // Read the incoming characters from the GPS stream until a newline ('\n') is hit.
    // This captures exactly one full standard NMEA sentence at a time.
    String nmea = Serial1.readStringUntil('\n');
    
    // Strip away any trailing whitespace or hidden carriage return characters (\r) 
    // to keep our print output clean and readable.
    nmea.trim();
    
    // Echo the cleaned-up NMEA sentence ($GNGGA, $GNGSV, etc.) over the USB link 
    // directly onto your laptop's Serial Monitor.
    Serial.println(nmea);
  }
}
