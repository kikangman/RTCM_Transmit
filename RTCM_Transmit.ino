//git kikangman
/*
cd /Users/kikang/Desktop/ki/summershot/RTK/RTCM_Transmit
./save_push.sh
*/

#include <HardwareSerial.h>

#define RX_GNSS 9  // GNSS TX → MCU RX
#define TX_GNSS 8  // GNSS RX → MCU TX

HardwareSerial GNSS(2);

unsigned long lastCheckTime = 0;
bool surveyInStarted = false;
bool surveyComplete = false;
String gnssLine = "";

bool parsingRtcm = false;
int rtcmLength = 0;
int rtcmIndex = 0;
uint8_t rtcmBuffer[1024];

void setup() {
  Serial.begin(115200);
  GNSS.begin(115200, SERIAL_8N1, RX_GNSS, TX_GNSS);
  delay(3000);

  // RTCM 출력 비활성화
  sendCommandWithChecksum("PAIR432,-1");
  delay(300);
  sendCommandWithChecksum("PAIR434,0");
  delay(300);

  // Base 모드 설정
  sendCommandWithChecksum("PQTMCFGRCVRMODE,W,2");
  delay(300);

  // Survey-In 시작
  sendCommandWithChecksum("PQTMCFGSVIN,W,1,100,2.0,0,0,0");
  delay(300);

  // 상태 메시지 출력 5초마다
  sendCommandWithChecksum("PQTMCFGMSGRATE,W,PQTMSVINSTATUS,5,1");
  delay(300);

  sendCommandWithChecksum("PQTMSAVEPAR");
  delay(300);

}

void loop() {
  while (GNSS.available()) {
    uint8_t c = GNSS.read();

    // RTCM 메시지 시작 감지
    if (!parsingRtcm && c == 0xD3) {
      parsingRtcm = true;
      rtcmIndex = 0;
      rtcmBuffer[rtcmIndex++] = c;
    }
    else if (parsingRtcm) {
      rtcmBuffer[rtcmIndex++] = c;

      if (rtcmIndex == 3) {
        // RTCM 길이는 바이트 2,3의 하위 10비트
        rtcmLength = ((rtcmBuffer[1] & 0x03) << 8) | rtcmBuffer[2];
        rtcmLength += 6; // 헤더(3) + CRC(3)
      }

      if (rtcmIndex >= rtcmLength && rtcmLength > 0) {
        // RTCM 메시지 완성 → HEX 출력
        for (int i = 0; i < rtcmIndex; i++) {
          if (rtcmBuffer[i] < 0x10) Serial.print("0");
          Serial.print(rtcmBuffer[i], HEX);
          Serial.print(" ");
        }
        Serial.println();
        parsingRtcm = false;
        rtcmLength = 0;
        rtcmIndex = 0;
      }
    }
    else {
      // 일반 ASCII 메시지는 그대로 출력
      Serial.write(c);

      if (c == '\n') {
        // Survey-In 완료 체크
        if (!surveyComplete && gnssLine.startsWith("$PQTMSVINSTATUS") && gnssLine.indexOf(",2,") > 0) {
          surveyComplete = true;
          Serial.println("✅ Survey-In 완료!");

          sendCommandWithChecksum("PQTMCFGMSGRATE,W,PQTMSVINSTATUS,0,1");
          delay(300);
          sendCommandWithChecksum("PAIR432,1"); // RTCM 켜기
          delay(300);
          sendCommandWithChecksum("PAIR434,1"); // 1005 포함
          delay(300);
          sendCommandWithChecksum("PQTMSAVEPAR");
          delay(300);
        }
        gnssLine = "";
      } else {
        gnssLine += (char)c;
      }
    }
  }


}

void sendCommandWithChecksum(const String &cmdBody) {
  byte checksum = 0;
  for (int i = 0; i < cmdBody.length(); i++) {
    checksum ^= cmdBody[i];
  }

  char command[128];
  snprintf(command, sizeof(command), "$%s*%02X\r\n", cmdBody.c_str(), checksum);

  GNSS.print(command);
  Serial.print("Sent: ");
  Serial.print(command);
}