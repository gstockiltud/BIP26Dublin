/*
 * ================================================================
 * QUECTEL LC29H FIRMWARE VERSION CHECKER (COMPLETE)
 * ================================================================
 * 
 * Purpose: Queries and displays the firmware version of the
 *          Waveshare LC29H(AA) GPS module
 * 
 * Hardware: TenStar ESP32-S3 + Waveshare LC29H GPS module
 * 
 * Connections:
 *   GPS TX (Yellow wire)  → ESP32 GPIO17 (RX)
 *   GPS RX (Blue wire)    → ESP32 GPIO18 (TX)
 *   GPS GND (Black wire)  → ESP32 GND
 * 
 * BIP26 SENSATE-X Programme
 * TU Dublin, April 2026
 * ================================================================
 */

// ================================================================
// CONFIGURATION
// ================================================================

#define GPS_RX_PIN 17
#define GPS_TX_PIN 18
#define USB_BAUD 115200
#define GPS_BAUD 115200
#define STARTUP_DELAY 3000
#define RETRY_INTERVAL 5000

// ================================================================
// GLOBAL VARIABLES
// ================================================================

bool versionReceived = false;
int queryAttempts = 0;
const int MAX_ATTEMPTS = 5;
unsigned long lastQuery = 0;
int nmeaCount = 0;

// ================================================================
// FORWARD DECLARATIONS
// ================================================================

void printBanner();
void queryFirmwareVersion();
void parseFirmwareVersion(String sentence);
void interpretVersion(String version, String date);

// ================================================================
// SETUP
// ================================================================

void setup() {
  Serial.begin(USB_BAUD);
  delay(1000);
  
  printBanner();
  
  Serial.println("[1/3] Initializing GPS Serial connection...");
  Serial.print("      GPS RX Pin: GPIO");
  Serial.println(GPS_RX_PIN);
  Serial.print("      GPS TX Pin: GPIO");
  Serial.println(GPS_TX_PIN);
  Serial.print("      GPS Baud Rate: ");
  Serial.println(GPS_BAUD);
  
  Serial1.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("      ✓ GPS Serial initialized");
  Serial.println();
  
  Serial.println("[2/3] Waiting for GPS module to boot...");
  Serial.print("      Please wait ");
  Serial.print(STARTUP_DELAY / 1000);
  Serial.println(" seconds...");
  
  for (int i = 0; i < STARTUP_DELAY / 1000; i++) {
    delay(1000);
    Serial.print("      .");
  }
  Serial.println(" Done!");
  Serial.println();
  
  while (Serial1.available()) {
    Serial1.read();
  }
  
  Serial.println("[3/3] Querying firmware version...");
  queryFirmwareVersion();
  lastQuery = millis();
}

// ================================================================
// LOOP
// ================================================================

void loop() {
  if (Serial1.available()) {
    String response = Serial1.readStringUntil('\n');
    response.trim();
    
    if (response.startsWith("$PQTMVERNO")) {
      versionReceived = true;
      Serial.println();
      Serial.println("      ✓ Firmware version response received!");
      Serial.println();
      parseFirmwareVersion(response);
      
    } else if (response.startsWith("$")) {
      nmeaCount++;
      
      if (nmeaCount % 10 == 0) {
        Serial.print("      [GPS transmitting - ");
        Serial.print(nmeaCount);
        Serial.println(" NMEA sentences, waiting for version...]");
      }
      
    } else if (response.length() > 0) {
      Serial.print("      Unknown: ");
      Serial.println(response);
    }
  }
  
  if (!versionReceived && (millis() - lastQuery > RETRY_INTERVAL)) {
    if (queryAttempts < MAX_ATTEMPTS) {
      Serial.println();
      Serial.print("      Retrying (attempt ");
      Serial.print(queryAttempts + 1);
      Serial.print(" of ");
      Serial.print(MAX_ATTEMPTS);
      Serial.println(")...");
      
      if (nmeaCount > 0) {
        Serial.print("      ℹ️  GPS working (");
        Serial.print(nmeaCount);
        Serial.println(" NMEA sentences received)");
      }
      
      queryFirmwareVersion();
      lastQuery = millis();
      
    } else {
      Serial.println();
      Serial.println("════════════════════════════════════════════════");
      Serial.println("⚠️  NO FIRMWARE VERSION AFTER 5 ATTEMPTS");
      Serial.println("════════════════════════════════════════════════");
      Serial.println();
      
      if (nmeaCount > 0) {
        Serial.println("✓ GPS is working correctly!");
        Serial.print("  Received ");
        Serial.print(nmeaCount);
        Serial.println(" NMEA sentences.");
        Serial.println();
        Serial.println("GPS did not respond to $PQTMVERNO query.");
        Serial.println();
        Serial.println("This is OK - your GPS works fine.");
        Serial.println("You can proceed with GPS projects.");
      } else {
        Serial.println("⚠️  GPS not responding!");
        Serial.println();
        Serial.println("Check:");
        Serial.println("  • GPS powered (USB connected)");
        Serial.println("  • Yellow: GPS TX → GPIO17");
        Serial.println("  • Blue: GPS RX → GPIO18");
        Serial.println("  • Black: GND → GND");
        Serial.println("  • Jumpers in B-B position");
      }
      
      Serial.println();
      Serial.println("Press RESET to try again.");
      Serial.println();
      
      while(1) delay(1000);
    }
  }
}

// ================================================================
// QUERY FIRMWARE VERSION
// ================================================================

void queryFirmwareVersion() {
  Serial.println("      Sending: $PQTMVERNO*58");
  
  while (Serial1.available()) {
    Serial1.read();
  }
  
  Serial1.println("$PQTMVERNO*58");
  queryAttempts++;
  
  Serial.println("      Waiting for response...");
  Serial.println("      (NMEA sentences filtered)");
}

// ================================================================
// PARSE FIRMWARE VERSION
// ================================================================

void parseFirmwareVersion(String sentence) {
  Serial.println("════════════════════════════════════════════════");
  Serial.println("✓ FIRMWARE VERSION RECEIVED!");
  Serial.println("════════════════════════════════════════════════");
  Serial.println();
  
  Serial.println("Raw response:");
  Serial.print("  ");
  Serial.println(sentence);
  Serial.println();
  
  int comma1 = sentence.indexOf(',');
  int comma2 = sentence.indexOf(',', comma1 + 1);
  int comma3 = sentence.indexOf(',', comma2 + 1);
  int asterisk = sentence.indexOf('*');
  
  if (comma1 > 0 && comma2 > 0 && comma3 > 0 && asterisk > 0) {
    String version = sentence.substring(comma1 + 1, comma2);
    String date = sentence.substring(comma2 + 1, comma3);
    String time = sentence.substring(comma3 + 1, asterisk);
    String checksum = sentence.substring(asterisk + 1);
    
    Serial.println("Parsed information:");
    Serial.println("┌──────────────────────────────────────────────┐");
    Serial.print("│ Firmware Version:  ");
    Serial.print(version);
    for (int i = version.length(); i < 27; i++) Serial.print(" ");
    Serial.println("│");
    
    Serial.print("│ Build Date:        ");
    Serial.print(date);
    for (int i = date.length(); i < 27; i++) Serial.print(" ");
    Serial.println("│");
    
    Serial.print("│ Build Time:        ");
    Serial.print(time);
    for (int i = time.length(); i < 27; i++) Serial.print(" ");
    Serial.println("│");
    
    Serial.print("│ Checksum:          ");
    Serial.print(checksum);
    for (int i = checksum.length(); i < 27; i++) Serial.print(" ");
    Serial.println("│");
    Serial.println("└──────────────────────────────────────────────┘");
    Serial.println();
    
    if (nmeaCount > 0) {
      Serial.print("ℹ️  Filtered ");
      Serial.print(nmeaCount);
      Serial.println(" NMEA sentences to find this.");
      Serial.println();
    }
    
    interpretVersion(version, date);
    
  } else {
    Serial.println("⚠️  Could not parse firmware version");
    Serial.println("    Format may be different than expected");
  }
  
  Serial.println();
  Serial.println("════════════════════════════════════════════════");
  Serial.println("TEST COMPLETE");
  Serial.println("════════════════════════════════════════════════");
  Serial.println();
  Serial.println("Summary:");
  Serial.println("  • Firmware version: Received ✓");
  Serial.print("  • NMEA sentences: ");
  Serial.print(nmeaCount);
  Serial.println(" filtered");
  Serial.println();
  Serial.println("Next:");
  Serial.println("  • Press RESET to test again");
  Serial.println("  • Upload different program");
  Serial.println("  • Continue with BIP26");
  Serial.println();
  
  while(1) delay(1000);
}

// ================================================================
// INTERPRET VERSION FOR OSNMA
// ================================================================

void interpretVersion(String version, String date) {
  Serial.println("OSNMA Support Analysis:");
  Serial.println("────────────────────────────────────────────────");
  
  int year = 0;
  if (date.length() >= 4) {
    year = date.substring(0, 4).toInt();
  }
  
  if (year >= 2023) {
    Serial.println("✓ Firmware date: 2023 or later");
    Serial.println("  → Likely includes OSNMA support");
    Serial.println();
    Serial.println("Next step:");
    Serial.println("  • Run OSNMA verification program");
    Serial.println("  • Check for $PQTMGALIONOAUTH sentences");
  } else if (year > 0 && year < 2023) {
    Serial.println("⚠️  Firmware date: Before 2023");
    Serial.println("  → May NOT include OSNMA support");
    Serial.println();
    Serial.println("Options:");
    Serial.println("  • Test OSNMA verification");
    Serial.println("  • Firmware upgrade may be needed");
  } else {
    Serial.println("ℹ️  Could not determine date");
    Serial.println("  • Run OSNMA verification to check");
  }
  
  Serial.println();
  
  if (version.indexOf("AA") >= 0 || version.indexOf("OSNMA") >= 0) {
    Serial.println("ℹ️  Version contains 'AA' or 'OSNMA'");
    Serial.println("   → OSNMA-aware firmware");
  }
}

// ================================================================
// PRINT BANNER
// ================================================================

void printBanner() {
  Serial.println();
  Serial.println("════════════════════════════════════════════════");
  Serial.println("   QUECTEL LC29H FIRMWARE VERSION CHECKER");
  Serial.println("════════════════════════════════════════════════");
  Serial.println();
  Serial.println("Queries Waveshare LC29H GPS firmware version.");
  Serial.println();
  Serial.println("NMEA sentences automatically filtered.");
  Serial.println();
  Serial.println("Hardware required:");
  Serial.println("  • TenStar ESP32-S3 (USB connected)");
  Serial.println("  • Waveshare LC29H GPS (USB powered)");
  Serial.println("  • Wires:");
  Serial.println("    Yellow: GPS TX → ESP32 GPIO17");
  Serial.println("    Blue:   GPS RX → ESP32 GPIO18");
  Serial.println("    Black:  GND → GND");
  Serial.println("  • Jumpers in B-B position");
  Serial.println();
  Serial.println("BIP26 SENSATE-X Programme");
  Serial.println("TU Dublin, April 2026");
  Serial.println("════════════════════════════════════════════════");
  Serial.println();
}