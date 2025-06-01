#include <HardwareSerial.h>

#define RX_GNSS 9   // GNSS TX → MCU RX
#define TX_GNSS 8   // GNSS RX → MCU TX

HardwareSerial GNSS(2);

enum State { WAIT_FOR_D3, READ_LEN_1, READ_LEN_2, READ_BODY };
State state = WAIT_FOR_D3;

uint16_t payloadLen = 0;
uint16_t totalLen = 0;
uint16_t count = 0;
uint8_t message[1024];

void setup() {
  Serial.begin(115200);
  GNSS.begin(115200, SERIAL_8N1, RX_GNSS, TX_GNSS);
  delay(1000);
  GNSS.print("$PQTMVERNO*58\r\n");  // 펌웨어 버전 요청
}

void loop() {
  while (GNSS.available()) {
    uint8_t c = GNSS.read();

    switch (state) {
      case WAIT_FOR_D3:
        if (c == 0xD3) {
          message[0] = c;
          count = 1;
          state = READ_LEN_1;
        }
        break;

      case READ_LEN_1:
        message[count++] = c;
        payloadLen = (c & 0x03) << 8;
        state = READ_LEN_2;
        break;

      case READ_LEN_2:
        message[count++] = c;
        payloadLen |= c;
        totalLen = 3 + payloadLen + 3;
        if (totalLen > sizeof(message)) {
          state = WAIT_FOR_D3;
        } else {
          state = READ_BODY;
        }
        break;

      case READ_BODY:
        message[count++] = c;
        if (count >= totalLen) {
          for (uint16_t i = 0; i < count; i++) {
            if (message[i] < 0x10) Serial.print("0");
            Serial.print(message[i], HEX);
            Serial.print(" ");
          }
          Serial.println();
          state = WAIT_FOR_D3;
          count = 0;
        }
        break;
    }
  }
}