/*
 * SerialStream.h
 *
 *  Created on: Feb 2, 2024
 *      Author: TR001221
 */

#ifndef SERIALCHANNEL_H_
#define SERIALCHANNEL_H_

#include <Arduino.h>

class SerialChannel {
public:
	enum ChannelStatus {
		OK, TIME_OUT
	};

	SerialChannel(HardwareSerial &port) : port(port) {
	}

	void begin(uint8_t rxPin, uint8_t txPin, uint32_t baud = 115200);
	void updateBaud(uint32_t baud);
	void flush(uint32_t waitTime = 500);
	ChannelStatus receiveData(void *data, size_t size, size_t *returned_size = nullptr);
	size_t sendData(const void *data, size_t size);

private:
	static constexpr size_t chunkSize = 128;
	static constexpr uint32_t timeOutPeriod = 1000;
	HardwareSerial &port;
	static void rxError(hardwareSerial_error_t e) {
	}
};

#endif /* SERIALCHANNEL_H_ */
