#include <TinyGPS++.h>
#include <RadioLib.h>
#include <HardwareSerial.h>

// Define custom SPI pins
#define CUSTOM_MOSI 12
#define CUSTOM_MISO 13
#define CUSTOM_SCLK 11
#define CUSTOM_NSS 10
#define CUSTOM_DIO1 2
#define CUSTOM_NRST 4
#define CUSTOM_BUSY 3

// Define custom UART pins
#define RX_PIN 9   // RX (GPS)
#define TX_PIN 10  // TX (GPS)


// Create a custom SPI instance
SPIClass customSPI(HSPI);  // Use HSPI or VSPI depending on your setup

// Create an SX1280 instance with custom SPI
SX1280 radio = new Module(CUSTOM_NSS, CUSTOM_DIO1, CUSTOM_NRST, CUSTOM_BUSY, customSPI);

// UART settings
HardwareSerial mySerial(2);  // UART2 for GPS

// TinyGPS++ object
TinyGPSPlus gps;

volatile bool transmittedFlag = false;

unsigned long lastSendTime = 0;
const unsigned long sendInterval = 1000;  // 1초



// This function is called when a complete packet is transmitted
#if defined(ESP8266) || defined(ESP32)
ICACHE_RAM_ATTR
#endif
void setFlag(void) {
  transmittedFlag = true;
}

void setup() {
  // UART initialization
  mySerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);  // Baud rate 115200, RX/TX pins
  Serial.begin(115200);                                // For debugging

  Serial.println(F("[Setup] Initializing LoRa and UART..."));

  // Initialize custom SPI
  customSPI.begin(CUSTOM_SCLK, CUSTOM_MISO, CUSTOM_MOSI, CUSTOM_NSS);

  // Initialize SX1280 with default settings
  Serial.print(F("[SX1280] Initializing ... "));
  int state = radio.begin();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) { delay(10); }
  }

  // Set the function that will be called when packet transmission is finished
  radio.setPacketSentAction(setFlag);

  Serial.println(F("[Setup] Initialization complete."));
}

void loop() {
  // Parse GPS data
  while (mySerial.available() > 0) {
    gps.encode(mySerial.read());
  }

  unsigned long currentMillis = millis();
  if (currentMillis - lastSendTime >= sendInterval) {
    lastSendTime = currentMillis;
    sendParsedGPSData();  // 주기적으로 GPS 데이터 전송
  }

  // 전송 완료 처리
  if (transmittedFlag) {
    transmittedFlag = false;
    Serial.println(F("[SX1280] Transmission finished!"));
    radio.finishTransmit();
  }
  
}

void sendParsedGPSData() {
  String gpsData = "";

  // Latitude and Longitude
  if (gps.location.isValid()) {
    gpsData += String(gps.location.lat(), 6) + ",";
    gpsData += String(gps.location.lng(), 6) + ",";
  } else {
    gpsData += "INVALID,INVALID,";
  }

  // Time
  if (gps.time.isValid()) {
    if (gps.time.hour() < 10) gpsData += "0";
    gpsData += String(gps.time.hour()) + ":";
    if (gps.time.minute() < 10) gpsData += "0";
    gpsData += String(gps.time.minute()) + ":";
    if (gps.time.second() < 10) gpsData += "0";
    gpsData += String(gps.time.second()) + ",";
  } else {
    gpsData += "INVALID,";
  }

  // Satellite count
  if (gps.satellites.isValid()) {
    gpsData += String(gps.satellites.value());
  }

  // Log data to Serial for debugging
  Serial.println(F("[GPS] Parsed data: "));
  Serial.println(gpsData);

  // Transmit GPS data over LoRa
  Serial.print(F("[SX1280] Sending GPS data over LoRa: "));
  Serial.println(gpsData);
  int state = radio.startTransmit(gpsData);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("[SX1280] Transmission started successfully."));
  } else {
    Serial.print(F("[SX1280] Transmission failed, code "));
    Serial.println(state);
  }
}
