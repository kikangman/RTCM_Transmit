#ifndef RTCM1005Parser_h
#define RTCM1005Parser_h

#include <Arduino.h>

class RTCM1005Parser {
public:
  bool parseMessage(const uint8_t* data, size_t len);
  double getLatitude();
  double getLongitude();
  double getAltitude();

private:
  double ecefX, ecefY, ecefZ;
  double latitude, longitude, altitude;
  void ecefToGeodetic();
  int64_t extractSignedBits(const uint8_t* data, int startBit, int bitLength);
};

#endif