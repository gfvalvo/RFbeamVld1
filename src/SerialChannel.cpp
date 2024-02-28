/*
 * SerialChannel.cpp
 *
 *  Created on: Feb 2, 2024
 *      Author: TR001221
 */

#include "SerialChannel.h"
#include <algorithm>

void SerialChannel::begin(uint8_t rxPin, uint8_t txPin, uint32_t baud) {
	port.end();
	port.setRxBufferSize(2500);
	port.begin(baud, SERIAL_8E1, rxPin, txPin);
	//port.begin(baud, SERIAL_8E1);
	port.onReceiveError(rxError);
}

void SerialChannel::updateBaud(uint32_t baud) {
	port.begin(baud, SERIAL_8E1);
}

void SerialChannel::flush(uint32_t waitTime) {
	port.flush();
	vTaskDelay(waitTime);		// wait expected time for all in-process data to come in
	while (port.read() >= 0) {
	}
}

size_t SerialChannel::sendData(const void *data, size_t size) {
	const uint8_t * dataPtr = reinterpret_cast<const uint8_t *> (data);
	return port.write(dataPtr, size);
}

SerialChannel::ChannelStatus SerialChannel::receiveData(void *data, size_t size, size_t *returned_size) {
	size_t dummyByteCount = 0;
	if (returned_size == nullptr) {
		returned_size = &dummyByteCount;
	}
	*returned_size = 0;

	unsigned char *dataPtr = reinterpret_cast<unsigned char *> (data);
	uint32_t startTime = millis();
	while (size > 0) {
		size_t minimumTransferSize = std::min(size, chunkSize);
		while (true) {
			size_t bytesAvailable = port.available();
			if (bytesAvailable >= minimumTransferSize) {
				size_t transferSize = std::min(size, bytesAvailable);
				port.readBytes(dataPtr, transferSize);
				dataPtr += transferSize;
				size -= transferSize;
				*returned_size += transferSize;
				break;
			}
			if (millis() - startTime > timeOutPeriod) {
				return TIME_OUT;
			}
			vTaskDelay(1);
		}
	}
	return OK;
}

