//git kikangman

#include <RadioLib.h>
#include <HardwareSerial.h>

#define CUSTOM_MOSI 12
#define CUSTOM_MISO 13
#define CUSTOM_SCLK 11
#define CUSTOM_NSS  10
#define CUSTOM_DIO1 2
#define CUSTOM_NRST 4
#define CUSTOM_BUSY 3

#define RX_GNSS 9
#define TX_GNSS 8

SPIClass customSPI(HSPI);
SX1280 radio = new Module(CUSTOM_NSS, CUSTOM_DIO1, CUSTOM_NRST, CUSTOM_BUSY, customSPI);
HardwareSerial GNSS(1);

// --- 전역 변수 및 상태 ---
unsigned long lastStatusQuery = 0;
unsigned long lastByteTime = 0;
bool surveyInComplete = false;

const unsigned long GAP_TIMEOUT_MS = 50;

uint8_t rtcmBuffer[1024];
size_t rtcmBufferIndex = 0;

volatile bool transmittedFlag = false;
ICACHE_RAM_ATTR void setFlag() { transmittedFlag = true; }

void transmitRtcmChunked(uint8_t* data, size_t length) {
  if (length < 6) return;

  uint16_t msgID = ((data[3] << 4) | (data[4] >> 4)) & 0x0FFF;
  uint8_t totalChunks = ceil((float)length / (240 - 4));
  uint8_t txBuffer[240];

  for (uint8_t seq = 0; seq < totalChunks; seq++) {
    size_t offset = seq * (240 - 4);
    size_t chunkSize = min((size_t)236, length - offset);

    txBuffer[0] = (msgID >> 8) & 0xFF;
    txBuffer[1] = msgID & 0xFF;
    txBuffer[2] = seq;
    txBuffer[3] = totalChunks;

    memcpy(txBuffer + 4, data + offset, chunkSize);

    int state = radio.startTransmit(txBuffer, chunkSize + 4);
    if (state == RADIOLIB_ERR_NONE) {
      while (!transmittedFlag);
      transmittedFlag = false;
      radio.finishTransmit();
      Serial.printf("[LoRa] ✅ Chunk %d/%d sent\n", seq + 1, totalChunks);
    } else {
      Serial.printf("[LoRa] ❌ Chunk %d send failed: %d\n", seq, state);
      break;
    }
    delay(10);
  }
}

void setup() {
  Serial.begin(115200);
  GNSS.begin(115200, SERIAL_8N1, RX_GNSS, TX_GNSS);
  delay(1000);
  Serial.println("[Setup] Starting...");

  customSPI.begin(CUSTOM_SCLK, CUSTOM_MISO, CUSTOM_MOSI, CUSTOM_NSS);
  if (radio.begin() != RADIOLIB_ERR_NONE) {
    Serial.println("[SX1280] Init failed");
    while (true);
  }

  radio.setOutputPower(13);
  radio.setFrequency(2400.0);
  radio.setBandwidth(812.5);
  radio.setSpreadingFactor(7);
  radio.setPacketSentAction(setFlag);
  Serial.println("[Setup] LoRa Ready");

  GNSS.print("$PQTMCFGRCVRMODE,W,2*29\r\n");
  GNSS.print("$PQTMCFGSVIN,W,1,180,2.5,0,0,0*2D\r\n");
  GNSS.print("$PAIR432,1*3B\r\n");
  GNSS.print("$PQTMSAVEPAR*5A\r\n");
  Serial.println("[Setup] GNSS Base & Survey-In 설정 완료");
}

void loop() {
  unsigned long now = millis();

  if (now - lastStatusQuery > 10000) {
    GNSS.print("$PQTMSVINSTATUS,1*3A\r\n");
    lastStatusQuery = now;
    Serial.println("[DEBUG] Requesting survey-in status");
  }

  // --- NMEA 수신 라인 우선 처리 ---
  while (GNSS.available()) {
    String line = GNSS.readStringUntil('\n');
    line.trim();
    if (line.startsWith("$PQTMSVINSTATUS")) {
      Serial.println("[Survey-In] " + line);
      int statusIdx = line.indexOf(',', 3);
      if (statusIdx != -1) {
        int status = line.charAt(statusIdx + 1) - '0';
        if (status == 2 && !surveyInComplete) {
          surveyInComplete = true;
          Serial.println("[Survey-In] ✅ 완료됨! 기준국 위치가 반영됨");
          GNSS.print("$PQTMSAVEPAR*5A\r\n");
        }
      }
    }
  }

  // RTCM 수신 처리
  static uint8_t buffer[512];
  static size_t index = 0;
  static bool inPacket = false;
  static uint16_t expectedLength = 0;

  while (GNSS.available()) {
    uint8_t byteIn = GNSS.read();
    lastByteTime = now;

    if (!inPacket) {
      if (byteIn == 0xD3) {
        buffer[0] = byteIn;
        index = 1;
        inPacket = true;
      }
    } else {
      buffer[index++] = byteIn;

      if (index == 3) {
        expectedLength = ((buffer[1] & 0x03) << 8) | buffer[2];
      }

      if (index == expectedLength + 6) {
        if (rtcmBufferIndex + index < sizeof(rtcmBuffer)) {
          memcpy(rtcmBuffer + rtcmBufferIndex, buffer, index);
          rtcmBufferIndex += index;
        }
        inPacket = false;
        index = 0;
        expectedLength = 0;
      }

      if (index >= sizeof(buffer)) {
        Serial.println("[RTCM] Temp buffer overflow");
        inPacket = false;
        index = 0;
      }
    }
  }

  if (surveyInComplete && rtcmBufferIndex > 0 && now - lastByteTime > GAP_TIMEOUT_MS) {
    Serial.printf("[RTCM] Sending %d bytes...\n", rtcmBufferIndex);
    transmitRtcmChunked(rtcmBuffer, rtcmBufferIndex);
    rtcmBufferIndex = 0;
  }
}