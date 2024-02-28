#include "Arduino.h"
#include "RFbeamVld1.h"

void getTargetSamples(unsigned long baud);
void radarPowerControl(bool enable);

constexpr uint8_t rxPin {27};
constexpr uint8_t txPin {33};
RFbeamVld1 radar(Serial2, rxPin, txPin, radarPowerControl);

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("Starting\n\n");

  Serial.println("Initializing V-LD1 at 115200 Baud");
  auto reponcePtr = radar.begin();
  assert((reponcePtr->statusCode == RFbeamVld1::Vld1ReplyStatus_t::OK) && "INIT Command Failed @ 115200 Baud");
  Serial.printf("Version String: %s\n\n", reponcePtr->version.c_str());

  //Serial.println("Taking Target Samples at 115200 Baud");
  getTargetSamples(115200);

  Serial.println("Setting V-LD1 to 2000000 Baud");
  reponcePtr = radar.setBaud(RFbeamVld1::Vld1Baud_t::BAUD_2000000);
  assert((reponcePtr->statusCode == RFbeamVld1::Vld1ReplyStatus_t::OK) && "INIT Command Failed @ 2000000 Baud");
  Serial.printf("Version String: %s\n\n", reponcePtr->version.c_str());

  //Serial.println("Taking Target Samples at 2000000 Baud");
  getTargetSamples(2000000);

  Serial.println("\nDisconnecting from V-LD1");
  reponcePtr = radar.disconnect();
  assert((reponcePtr->statusCode == RFbeamVld1::Vld1ReplyStatus_t::OK) && "GBYE Command Failed");
  Serial.println("Disconnect Successful");

}

void loop() {
}

void getTargetSamples(unsigned long baud) {
  const uint32_t numSamples = 10;

  Serial.printf("Taking Target Samples at %lu Baud\n", baud);
  uint32_t start = millis();
  for (uint32_t i = 0; i < numSamples; i++) {
    auto reponcePtr = radar.getRadarFrameData(RFbeamVld1::doneMask | RFbeamVld1::pdatMask);
    assert((reponcePtr->statusCode == RFbeamVld1::Vld1ReplyStatus_t::OK) && "GFND Command Failed");
    float magnitude = reponcePtr->pdataData.magnitude / 100.0;
    Serial.printf("Target Data: Frame Number: %d, Distance: %.3f, Magnitude: %.2f\n", reponcePtr->frameCoun, reponcePtr->pdataData.distance, magnitude);
  }
  uint32_t period = millis() - start;
  uint32_t average = period / numSamples;
  Serial.printf("Average Period: %dms / Sample at %lu Baud, Sampling Target Data Only\n\n", average, baud);

  Serial.printf("Taking Target + FFT Samples at %lu Baud\n", baud);
  start = millis();
  for (uint32_t i = 0; i < numSamples; i++) {
    auto reponcePtr = radar.getRadarFrameData(RFbeamVld1::doneMask | RFbeamVld1::pdatMask | RFbeamVld1::rfftMask);
    assert((reponcePtr->statusCode == RFbeamVld1::Vld1ReplyStatus_t::OK) && "GFND Command Failed");
    float magnitude = reponcePtr->pdataData.magnitude / 100.0;
    Serial.printf("Target Data: Frame Number: %d, Distance: %.3f, Magnitude: %.2f\n", reponcePtr->frameCoun, reponcePtr->pdataData.distance, magnitude);
  }
  period = millis() - start;
  average = period / numSamples;
  Serial.printf("Average Period: %dms / Sample at %lu Baud, Sampling Target Data + FFT Data\n\n", average, baud);
}

void radarPowerControl(bool enable) {
  constexpr uint8_t radarPowerPin {14};
  static bool firstTime = true;

  if (firstTime) {
    pinMode(radarPowerPin, OUTPUT);
    digitalWrite(radarPowerPin, HIGH);
    firstTime = false;
  }

  if (enable) {
    digitalWrite(radarPowerPin, LOW);
  } else {
    digitalWrite(radarPowerPin, HIGH);
  }
}
