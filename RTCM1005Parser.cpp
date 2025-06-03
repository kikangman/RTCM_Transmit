#include "RTCM1005Parser.h"
#include <math.h>

bool RTCM1005Parser::parseMessage(const uint8_t* data, size_t len) {
  if (len < 20 || data[0] != 0xD3) return false;

  int msgType = extractSignedBits(data, 24, 12);
  if (msgType != 1005) return false;

  ecefX = extractSignedBits(data, 58, 38) * 0.0001;
  ecefY = extractSignedBits(data, 98, 38) * 0.0001;
  ecefZ = extractSignedBits(data, 138, 38) * 0.0001;

  ecefToGeodetic();
  return true;
}

void RTCM1005Parser::ecefToGeodetic() {
  const double a = 6378137.0;
  const double f = 1.0 / 298.257223563;
  const double b = a * (1 - f);
  const double e2 = 1 - (b * b) / (a * a);

  double x = ecefX;
  double y = ecefY;
  double z = ecefZ;

  double lon = atan2(y, x);
  double p = sqrt(x * x + y * y);
  double lat = atan2(z, p * (1 - e2));
  double lat_prev;

  do {
    lat_prev = lat;
    double sinLat = sin(lat);
    double N = a / sqrt(1 - e2 * sinLat * sinLat);
    lat = atan2(z + e2 * N * sinLat, p);
  } while (fabs(lat - lat_prev) > 1e-11);

  double sinLat = sin(lat);
  double N = a / sqrt(1 - e2 * sinLat * sinLat);
  double alt = p / cos(lat) - N;

  latitude = lat * 180.0 / PI;
  longitude = lon * 180.0 / PI;
  altitude = alt;
}

int64_t RTCM1005Parser::extractSignedBits(const uint8_t* data, int startBit, int bitLength) {
  int64_t value = 0;
  for (int i = 0; i < bitLength; i++) {
    int byteIndex = (startBit + i) / 8;
    int bitIndex = 7 - ((startBit + i) % 8);
    value = (value << 1) | ((data[byteIndex] >> bitIndex) & 0x01);
  }

  // Sign extension for two's complement
  if (value & ((int64_t)1 << (bitLength - 1))) {
    value -= ((int64_t)1 << bitLength);
  }

  return value;
}

double RTCM1005Parser::getLatitude() {
  return latitude;
}

double RTCM1005Parser::getLongitude() {
  return longitude;
}

double RTCM1005Parser::getAltitude() {
  return altitude;
}