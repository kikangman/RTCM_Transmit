//git kikangman
/*
cd /Users/kikang/Desktop/ki/summershot/RTK/RTCM_Transmit
./save_push.sh "??"
*/

#include <HardwareSerial.h>
#include <RadioLib.h>
#include "RTCM1005Parser.h"

#define RX_GNSS 9
#define TX_GNSS 8

#define CUSTOM_MOSI 12
#define CUSTOM_MISO 13
#define CUSTOM_SCLK 11
#define CUSTOM_NSS 10
#define CUSTOM_DIO1 2
#define CUSTOM_NRST 4
#define CUSTOM_BUSY 3

RTCM1005Parser parser

HardwareSerial GNSS(2);
SPIClass customSPI(HSPI);
SX1280 radio = new Module(CUSTOM_NSS, CUSTOM_DIO1, CUSTOM_NRST, CUSTOM_BUSY, customSPI);

volatile bool transmittedFlag = false;
volatile bool coordReceived = false;

ICACHE_RAM_ATTR void setTransmitFlag() {
  transmittedFlag = true;
}

ICACHE_RAM_ATTR void setCoordReceivedFlag() {
  coordReceived = true;
}

const size_t MAX_LORA_PAYLOAD = 240;
const size_t HEADER_SIZE = 4;
uint8_t txBuffer[MAX_LORA_PAYLOAD];

uint8_t rtcmBuffer[1024];
size_t rtcmBufferIndex = 0;
unsigned long lastByteTime = 0;
const unsigned long GAP_TIMEOUT_MS = 200;

bool surveyComplete = false;
bool suppressSVIN = false;
String gnssLine = "";

void sendCommandWithChecksum(const String& cmdBody) {
  byte checksum = 0;
  for (int i = 0; i < cmdBody.length(); i++) checksum ^= cmdBody[i];
  char command[128];
  snprintf(command, sizeof(command), "$%s*%02X\r\n", cmdBody.c_str(), checksum);
  GNSS.print(command);
  Serial.print("Sent: ");
  Serial.print(command);
}

void transmitRtcmChunked(uint8_t* data, size_t length) {
  if (length < 6) return;
  Serial.printf("\n[RTCM] Sending %zu bytes:\n", length);

  radio.setPacketReceivedAction(NULL);
  radio.setPacketSentAction(setTransmitFlag);
  radio.standby();
  delay(2);

  uint16_t msgID = ((data[3] << 4) | (data[4] >> 4)) & 0x0FFF;
  uint8_t totalChunks = ceil((float)length / (MAX_LORA_PAYLOAD - HEADER_SIZE));

  for (uint8_t seq = 0; seq < totalChunks; seq++) {
    size_t offset = seq * (MAX_LORA_PAYLOAD - HEADER_SIZE);
    size_t chunkSize = min((MAX_LORA_PAYLOAD - HEADER_SIZE), length - offset);

    txBuffer[0] = (msgID >> 8) & 0xFF;
    txBuffer[1] = msgID & 0xFF;
    txBuffer[2] = seq;
    txBuffer[3] = totalChunks;
    if (seq == totalChunks - 1) txBuffer[3] |= 0x80;  // 마지막 청크 플래그

    memcpy(txBuffer + HEADER_SIZE, data + offset, chunkSize);

    int state = radio.startTransmit(txBuffer, chunkSize + HEADER_SIZE);
    if (state == RADIOLIB_ERR_NONE) {
      unsigned long waitStart = millis();
      while (!transmittedFlag && millis() - waitStart < 500)
        ;
      transmittedFlag = false;
      radio.finishTransmit();
      //Serial.printf("[LoRa] ✅ Chunk %d/%d sent\n", seq + 1, totalChunks);
    } else {
      Serial.printf("[LoRa] ❌ Chunk %d send failed: %d\n", seq, state);
      break;
    }
    delay(10);
  }

  radio.startReceive();
  radio.setPacketReceivedAction(setCoordReceivedFlag);
  //Serial.println("[Base] 수신 대기 시작");
}

void setup() {
  Serial.begin(115200);
  GNSS.begin(115200, SERIAL_8N1, RX_GNSS, TX_GNSS);
  delay(2000);

  customSPI.begin(CUSTOM_SCLK, CUSTOM_MISO, CUSTOM_MOSI, CUSTOM_NSS);
  if (radio.begin() != RADIOLIB_ERR_NONE) {
    Serial.println("[SX1280] Init fail");
    while (true)
      ;
  }

  radio.standby();
  radio.setOutputPower(13);
  radio.setFrequency(2400.0);
  radio.setBandwidth(812.5);
  radio.setSpreadingFactor(7);
  radio.setPacketSentAction(setTransmitFlag);
  radio.setPacketReceivedAction(setCoordReceivedFlag);
  radio.startReceive();

  Serial.println("[Setup] LoRa Ready");

  sendCommandWithChecksum("PAIR432,-1");
  delay(300);
  sendCommandWithChecksum("PAIR434,1");
  delay(300);
  sendCommandWithChecksum("PQTMCFGRCVRMODE,W,2");
  delay(300);
  sendCommandWithChecksum("PQTMCFGSVIN,W,1,100,2.0,0,0,0");
  delay(300);
  sendCommandWithChecksum("PQTMCFGMSGRATE,W,PQTMSVINSTATUS,2,1");
  delay(300);
  sendCommandWithChecksum("PQTMSAVEPAR");
  delay(300);
}

void loop() {
  static uint8_t buffer[512];
  static size_t index = 0;
  static bool inPacket = false;
  static uint16_t expectedLength = 0;

  unsigned long now = millis();

  // RTCM 수신 처리
  while (GNSS.available()) {
    uint8_t c = GNSS.read();

    lastByteTime = now;

    if (!inPacket) {
      if (c == 0xD3) {
        buffer[0] = c;
        index = 1;
        inPacket = true;
      } else {
        if (c == '\n') {
          if (gnssLine.startsWith("$PQTMSVINSTATUS") && !suppressSVIN) {
            Serial.println("[SVIN 수신] " + gnssLine);
          }

          if (!surveyComplete && gnssLine.startsWith("$PQTMSVINSTATUS") && gnssLine.indexOf(",2,") > 0) {
            surveyComplete = true;
            Serial.println("✅ Survey-In 완료!");
            sendCommandWithChecksum("PQTMCFGMSGRATE,W,PQTMSVINSTATUS,0,1");
            delay(300);
            sendCommandWithChecksum("PAIR432,1");
            delay(300);
            sendCommandWithChecksum("PAIR434,1");
            delay(300);
            sendCommandWithChecksum("PQTMSAVEPAR");
            delay(300);
          }
          gnssLine = "";
        } else {
          gnssLine += (char)c;
        }
      }
    } else {
      buffer[index++] = c;
      if (index == 3) {
        expectedLength = ((buffer[1] & 0x03) << 8) | buffer[2];
        if (expectedLength == 0 || expectedLength > 480) {
          Serial.println("[RTCM] Invalid length");
          inPacket = false;
          index = 0;
          continue;
        }
      }

      if (index == expectedLength + 6) {
        // 메시지 ID 추출
        if (index >= 6) {
          uint16_t msgID = ((buffer[3] << 4) | (buffer[4] >> 4)) & 0x0FFF;

          if (msgID == 1005) {
            Serial.print("[RTCM]: ");
            for (size_t i = 0; i < index; i++) {
              if (buffer[i] < 0x10) Serial.print('0');
              Serial.print(buffer[i], HEX);
              Serial.print(' ');
            }
            Serial.println();
            if (parser.parseMessage(buffer, index)) {
              Serial.print("[Parsed Lat] ");
              Serial.println(parser.getLatitude(), 8);
              Serial.print("[Parsed Lon] ");
              Serial.println(parser.getLongitude(), 8);
              Serial.print("[Parsed Alt] ");
              Serial.println(parser.getAltitude(), 3);
            } else {
              Serial.println("[RTCM] ❌ 파싱 실패");
            }
          }
        }

        if (surveyComplete && rtcmBufferIndex + index < sizeof(rtcmBuffer)) {
          memcpy(rtcmBuffer + rtcmBufferIndex, buffer, index);
          rtcmBufferIndex += index;
        }
        inPacket = false;
        index = 0;
        expectedLength = 0;
      }

      if (index >= sizeof(buffer)) {
        Serial.println("[RTCM] Buffer overflow");
        inPacket = false;
        index = 0;
      }
    }
  }

  if (surveyComplete && rtcmBufferIndex > 0 && now - lastByteTime > GAP_TIMEOUT_MS) {
    suppressSVIN = true;  // SVIN 메시지 출력 중지
    transmitRtcmChunked(rtcmBuffer, rtcmBufferIndex);
    rtcmBufferIndex = 0;
  }

  // Rover 좌표 수신 처리
  if (surveyComplete && coordReceived) {
    coordReceived = false;

    uint8_t locBuffer[64] = { 0 };
    int len = radio.readData(locBuffer, sizeof(locBuffer));

    if (len == RADIOLIB_ERR_NONE) {
      int actualLen = radio.getPacketLength();
      locBuffer[actualLen] = '\0';
      String msg = String((char*)locBuffer);
      Serial.println("[RAW] " + msg);

      int t1 = msg.indexOf(',');
      int t2 = msg.indexOf(',', t1 + 1);
      int t3 = msg.indexOf(',', t2 + 1);

      if (t1 > 0 && t2 > t1 && t3 > t2) {
        String timeStr = msg.substring(0, t1);
        double lat = msg.substring(t1 + 1, t2).toDouble();
        double lon = msg.substring(t2 + 1, t3).toDouble();
        int fix = msg.substring(t3 + 1).toInt();

        // Serial.print("[Rover Time] ");
        // Serial.println(timeStr);
        Serial.print("[Rover Lat] ");
        Serial.println(lat, 8);
        Serial.print("[Rover Lon] ");
        Serial.println(lon, 8);
        Serial.print("[Rover Fix] ");
        Serial.println(fix);
      } else {
        Serial.println("[LoRa] ⚠️ 메시지 파싱 실패");
      }
    } else {
      Serial.printf("[LoRa] ❌ 좌표 수신 실패 (code %d)\n", len);
    }

    radio.startReceive();
  }
}