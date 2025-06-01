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



void setup() {
  Serial.begin(115200);
  GNSS.begin(115200, SERIAL_8N1, RX_GNSS, TX_GNSS);
  delay(3000);

  // // RTCM 출력 비활성화 명령 전송  -1끄기 / 0켜기
  sendCommandWithChecksum("PAIR432,-1");
  delay(300);
  // 1005 0끄기 / 1켜기
  sendCommandWithChecksum("PAIR434,0");
  delay(300);

  // sendCommandWithChecksum("PQTMVERNO");
  // delay(300);

  //base 모드로 설정
  sendCommandWithChecksum("PQTMCFGRCVRMODE,W,2");
  delay(300);

  //Survey-In 180초 2.0정밀도 이내
  sendCommandWithChecksum("PQTMCFGSVIN,W,1,180,2.0,0,0,0");
  delay(300);

  //Survey-In 진행 상태를 주기적으로 확인 5초에 한번
  sendCommandWithChecksum("PQTMCFGMSGRATE,W,PQTMSVINSTATUS,5,1");
  delay(300);

  //설정 저장
  sendCommandWithChecksum("PQTMSAVEPAR");
  delay(300);
  surveyInStarted = false;
}


void loop() {
  // GNSS 응답 출력


  while (GNSS.available()) {
    char c = GNSS.read();
    Serial.write(c);
  }

  if (surveyInStarted && !surveyComplete && millis() - lastCheckTime >= 10000) {
    lastCheckTime = millis();
    Serial.println("Checking Survey-In status...");
    // sendCommandWithChecksum("PQTMCFGSVIN,R");
  }

  // 응답 확인은 여기서 문자열 파싱해서 surveyComplete = true; 처리 가능
  // 지금은 수동으로 시리얼로 확인
}

// 체크섬 붙여서 명령 전송
void sendCommandWithChecksum(const String &cmdBody) {
  byte checksum = 0;

  for (int i = 0; i < cmdBody.length(); i++) {
    checksum ^= cmdBody[i];
  }

  char command[128];
  snprintf(command, sizeof(command), "$%s*%02X\r\n", cmdBody.c_str(), checksum);

  GNSS.print(command);     // GNSS로 전송
  Serial.print("Sent: ");  // 디버깅 출력
  Serial.print(command);
}