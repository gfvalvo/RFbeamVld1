/*
 * Vld1.cpp
 *
 *  Created on: Jan 30, 2024
 *      Author: GFV
 */

#include "RFbeamVld1.h"
#include <algorithm>

std::map<const RFbeamVld1::Vld1Command_t, const RFbeamVld1::CommandInfo_t> RFbeamVld1::commandMap {
		{ Vld1Command_t::INIT, { "INIT", initPayloadSize, { Vld1Reply_t::RESP, Vld1Reply_t::VERS } } },
		{ Vld1Command_t::GNFD, { "GNFD", gnfdPayloadSize, { Vld1Reply_t::RESP, Vld1Reply_t::RADC, Vld1Reply_t::RFFT, Vld1Reply_t::PDAT, Vld1Reply_t::DONE } } },
		{ Vld1Command_t::GRPS, { "GRPS", grpsPayloadSize, { Vld1Reply_t::RESP, Vld1Reply_t::RPST } } },
		{ Vld1Command_t::SRPS, { "SRPS", srpsPayloadSize, { Vld1Reply_t::RESP } } },
		{ Vld1Command_t::RFSE, { "RFSE", rfsePayloadSize, { Vld1Reply_t::RESP } } },
		{ Vld1Command_t::GBYE, { "GBYE", gbyePayloadSize, { Vld1Reply_t::RESP } } },
		{ Vld1Command_t::RRAI, { "RRAI", rraiPayloadSize, { Vld1Reply_t::RESP } } },
		{ Vld1Command_t::THOF, { "THOF", thofPayloadSize, { Vld1Reply_t::RESP } } },
		{ Vld1Command_t::MIRA, { "MIRA", miraPayloadSize, { Vld1Reply_t::RESP } } },
		{ Vld1Command_t::MARA, { "MARA", maraPayloadSize, { Vld1Reply_t::RESP } } },
		{ Vld1Command_t::RAVG, { "RAVG", ravgPayloadSize, { Vld1Reply_t::RESP } } },
		{ Vld1Command_t::TGFI, { "TGFI", tgfiPayloadSize, { Vld1Reply_t::RESP } } },
		{ Vld1Command_t::PREC, { "PREC", precPayloadSize, { Vld1Reply_t::RESP } } },
		{ Vld1Command_t::TXPW, { "TXPW", txpwPayloadSize, { Vld1Reply_t::RESP } } },
		{ Vld1Command_t::INTN, { "INTN", intnPayloadSize, { Vld1Reply_t::RESP } } },
		{ Vld1Command_t::SRDF, { "SRDF", srdfPayloadSize, { Vld1Reply_t::RESP } } },
		{ Vld1Command_t::JBTL, { "JBTL", jbtlPayloadSize, { Vld1Reply_t::RESP } } }
};

std::map<const RFbeamVld1::Vld1Reply_t, const RFbeamVld1::replyInfo_t> RFbeamVld1::replyMap {
		{ Vld1Reply_t::RESP, { "RESP", respPayloadSize } },
		{ Vld1Reply_t::VERS, { "VERS", versPayloadSize } },
		{ Vld1Reply_t::RADC, { "RADC", radcPayloadSize } },
		{ Vld1Reply_t::RFFT, { "RFFT", rfftPayloadSize } },
		{ Vld1Reply_t::PDAT, { "PDAT", pdatPayloadSize } },
		{ Vld1Reply_t::DONE, { "DONE", donePayloadSize } },
		{ Vld1Reply_t::RPST, { "RPST", rpstPayloadSize } }
};

std::map<const RFbeamVld1::Vld1Baud_t, const uint32_t> RFbeamVld1::baudMap {
		{ Vld1Baud_t::BAUD_115200, 115200 },
		{ Vld1Baud_t::BAUD_460800, 460800 },
		{ Vld1Baud_t::BAUD_921600, 921600 },
		{ Vld1Baud_t::BAUD_2000000, 2000000 }
};

RFbeamVld1::RFbeamVld1(HardwareSerial &c, uint8_t rxPin, uint8_t txPin, PowerControlFunction pwrCntl) : channel(c), rxPin(rxPin), txPin(txPin), powerControl(pwrCntl) {
}

RFbeamVld1::Vld1ResponcePtr RFbeamVld1::begin() {
	TaskFunction *actualTask;
	uint8_t payload { 0x00 };

	powerControl(false);
	channel.begin(rxPin, txPin);
	currentBaud = Vld1Baud_t::BAUD_115200;
	vTaskDelay(500);
	powerControl(true);

	pendingCommandQueue = xQueueCreate(pendingCommandQueueSize,
			sizeof(CommandRequest_t));
	assert((pendingCommandQueue != 0) && "Couldn't create pendingCommandQueue");

	responceQueue = xQueueCreate(responceQueueSize,
			sizeof(const Vld1Responce_t*));
	assert((responceQueue != 0) && "Couldn't create responceQueue");

	actualTask = new TaskFunction { [this]() {
		this->processCommands();
	} };
	BaseType_t returnCode = xTaskCreatePinnedToCore(taskLauncher,
			"Process Commands", 4096, reinterpret_cast<void*>(actualTask), 6,
			NULL, CONFIG_ARDUINO_RUNNING_CORE);
	assert((returnCode == pdTRUE) && "Couldn't create processReplies");

	channel.flush(100);
	sendCommand(Vld1Command_t::INIT, &payload);
	Vld1ResponcePtr reponcePtr;
	reponcePtr = getRadarReply();
	log_i("Command: %d, Status Code: %d", reponcePtr->commandSent,
			reponcePtr->statusCode);
	log_i("Command: %d, Version String: %s", reponcePtr->commandSent,
			reponcePtr->version.c_str());
	return reponcePtr;
}

void RFbeamVld1::sendCommand(Vld1Command_t cmd, const void *dataPtr) {
	const uint8_t *bytePtr = reinterpret_cast<const uint8_t*>(dataPtr);
	CommandRequest_t nextCommand { cmd, bytePtr };
	BaseType_t result = xQueueSendToBack(pendingCommandQueue, &nextCommand, 0);
	if (result != pdTRUE) {
		log_e("Command Queue Full");
	}
}

RFbeamVld1::Vld1ResponcePtr RFbeamVld1::getRadarReply(bool waitForReply) {
	Vld1ResponcePtr responcePtr { new Vld1Responce_t };
	const Vld1Responce_t *queueData { nullptr };
	TickType_t waitTime = (waitForReply) ? portMAX_DELAY : 0;
	auto status = xQueueReceive(responceQueue, &queueData, waitTime);
	if (status == pdTRUE) {
		responcePtr.reset(queueData);
	}
	return responcePtr;
}

RFbeamVld1::Vld1ResponcePtr RFbeamVld1::setBaud(Vld1Baud_t newBaud,
		bool waitForReply) {
	RFbeamVld1::Vld1ResponcePtr reponcePtr;
	sendCommand(Vld1Command_t::INIT, &newBaud);
	reponcePtr = getRadarReply(waitForReply);
	return reponcePtr;
}

RFbeamVld1::Vld1ResponcePtr RFbeamVld1::getRadarFrameData(
		uint8_t dataRequestMask, bool waitForReply) {
	sendCommand(Vld1Command_t::GNFD, &dataRequestMask);
	return getRadarReply(waitForReply);
}

RFbeamVld1::Vld1ResponcePtr RFbeamVld1::getRadarRadarParameters(
		bool waitForReply) {
	sendCommand(Vld1Command_t::GRPS, nullptr);
	return getRadarReply(waitForReply);
}

RFbeamVld1::Vld1ResponcePtr RFbeamVld1::setRadarRadarParameters(
		const RadarDataStructure_t &radarDataStructure, bool waitForReply) {
	sendCommand(Vld1Command_t::SRPS, &radarDataStructure);
	return getRadarReply(waitForReply);
}

RFbeamVld1::Vld1ResponcePtr RFbeamVld1::disconnect(bool waitForReply) {
	sendCommand(Vld1Command_t::GBYE, nullptr);
	return getRadarReply(waitForReply);
}

RFbeamVld1::Vld1ResponcePtr RFbeamVld1::setDistanceRange(
		Vld1DistanceRange_t range, bool waitForReply) {
	sendCommand(Vld1Command_t::RRAI, &range);
	return getRadarReply(waitForReply);
}

RFbeamVld1::Vld1ResponcePtr RFbeamVld1::setThresholdOffset(uint8_t offset,
		bool waitForReply) {
	offset = std::max(offset, static_cast<uint8_t>(20));
	offset = std::min(offset, static_cast<uint8_t>(90));
	sendCommand(Vld1Command_t::THOF, &offset);
	return getRadarReply(waitForReply);
}

RFbeamVld1::Vld1ResponcePtr RFbeamVld1::setMinRangeFilter(uint16_t range,
		bool waitForReply) {
	range = std::max(range, static_cast<uint16_t>(1));
	range = std::min(range, static_cast<uint16_t>(510));
	sendCommand(Vld1Command_t::MIRA, &range);
	return getRadarReply(waitForReply);
}

RFbeamVld1::Vld1ResponcePtr RFbeamVld1::setMaxRangeFilter(uint16_t range,
		bool waitForReply) {
	range = std::max(range, static_cast<uint16_t>(2));
	range = std::min(range, static_cast<uint16_t>(511));
	sendCommand(Vld1Command_t::MARA, &range);
	return getRadarReply(waitForReply);
}

RFbeamVld1::Vld1ResponcePtr RFbeamVld1::setAvgCount(uint8_t count,
		bool waitForReply) {
	count = std::max(count, static_cast<uint8_t>(20));
	count = std::min(count, static_cast<uint8_t>(90));
	sendCommand(Vld1Command_t::RAVG, &count);
	return getRadarReply(waitForReply);
}

RFbeamVld1::Vld1ResponcePtr RFbeamVld1::setTargetFilter(TargetFilter_t filter,
		bool waitForReply) {
	sendCommand(Vld1Command_t::TGFI, &filter);
	return getRadarReply(waitForReply);
}

RFbeamVld1::Vld1ResponcePtr RFbeamVld1::setPrecisionMode(
		PrecisionMode_t precision, bool waitForReply) {
	sendCommand(Vld1Command_t::PREC, &precision);
	return getRadarReply(waitForReply);
}

RFbeamVld1::Vld1ResponcePtr RFbeamVld1::setTxPower(uint8_t power,
		bool waitForReply) {
	power = std::min(power, static_cast<uint8_t>(31));
	sendCommand(Vld1Command_t::TXPW, &power);
	return getRadarReply(waitForReply);
}

RFbeamVld1::Vld1ResponcePtr RFbeamVld1::setInegrationCount(uint8_t count,
		bool waitForReply) {
	count = std::max(count, static_cast<uint8_t>(1));
	count = std::min(count, static_cast<uint8_t>(100));
	sendCommand(Vld1Command_t::INTN, &count);
	return getRadarReply(waitForReply);
}

RFbeamVld1::Vld1ResponcePtr RFbeamVld1::setShortRangeFilter(
		ShortRangeFilter_t filter, bool waitForReply) {
	sendCommand(Vld1Command_t::SRDF, &filter);
	return getRadarReply(waitForReply);
}

void RFbeamVld1::processCommands() {
	for (;;) {
		MessageHeader_t commandBytes;
		uint8_t gnfdPayloadByte;
		bool gnfdCommand { false };
		CommandRequest_t nextCommmand;
		std::vector<Vld1Reply_t> repliesExpected;
		repliesExpected.reserve(CommandInfo_t::maxReplies);

		xQueueReceive(pendingCommandQueue, &nextCommmand, portMAX_DELAY);
		Vld1Command_t cmd = nextCommmand.command;
		const uint8_t *dataPtr = nextCommmand.payloadPtr;

		const auto &cmdInfo = commandMap.at(cmd);
		memcpy(&commandBytes.opCode, &cmdInfo.opcode[0], opCodeLength);
		commandBytes.payloadLength = cmdInfo.payloadLength;

		if (cmd == Vld1Command_t::GNFD) {
			gnfdCommand = true;
			gnfdPayloadByte = *dataPtr;
		}

		for (auto nextReply : cmdInfo.replyCodes) {
			if (nextReply == static_cast<Vld1Reply_t>(0)) {
				break;
			}

			if (gnfdCommand) {
				switch (nextReply) {
					case Vld1Reply_t::RESP:
						repliesExpected.push_back(nextReply);
						break;

					case Vld1Reply_t::RADC:
						if ((gnfdPayloadByte & radcMask) != 0) {
							repliesExpected.push_back(nextReply);
						}
						break;

					case Vld1Reply_t::RFFT:
						if ((gnfdPayloadByte & rfftMask) != 0) {
							repliesExpected.push_back(nextReply);
						}
						break;

					case Vld1Reply_t::PDAT:
						if ((gnfdPayloadByte & pdatMask) != 0) {
							repliesExpected.push_back(nextReply);
						}
						break;

					case Vld1Reply_t::DONE:
						if ((gnfdPayloadByte & doneMask) != 0) {
							repliesExpected.push_back(nextReply);
						}
						break;

					default:
						break;
				}
			} else {
				repliesExpected.push_back(nextReply);
			}
		}

		channel.sendData(&commandBytes, headerLength);
		if (commandBytes.payloadLength > 0) {
			channel.sendData(dataPtr, commandBytes.payloadLength);
		}

		std::unique_ptr<Vld1Responce_t> responseData { new Vld1Responce_t };
		assert((responseData != nullptr) && "responseData NULL");
		responseData->commandSent = cmd;
		responseData->statusCode = Vld1ReplyStatus_t::OK;
		if (cmd == Vld1Command_t::GNFD) {
			responseData->gnfdPayloadByte = gnfdPayloadByte;
		}
		bool flushBuffer { false };

		for (auto nextReply : repliesExpected) {
			log_i("Reply: %d", nextReply);

			MessageHeader_t receivedHeader;
			if (channel.receiveData(&receivedHeader, headerLength)
					!= SerialChannel::OK) {
				log_e("Timeout Reading Header");
				responseData->statusCode = Vld1ReplyStatus_t::READ_TIMEOUT;
				flushBuffer = true;
				break;
			}

			auto const &replyMessage = replyMap.at(nextReply);
			if (memcmp(receivedHeader.opCode, replyMessage.messageName,
					opCodeLength) != 0) {
				log_e("OpCode Mismatch");
				responseData->statusCode = Vld1ReplyStatus_t::BAD_OPCODE;
				flushBuffer = true;
				break;
			}

			if (receivedHeader.payloadLength != replyMessage.payloadLength) {
				if (!((nextReply == Vld1Reply_t::PDAT)
						&& (receivedHeader.payloadLength == 0))) {
					log_e("Payload Size Mismatch");
					responseData->statusCode =
							Vld1ReplyStatus_t::BAD_PAYLOAD_LENGTH;
					flushBuffer = true;
					break;
				} else {
					//log_i("Zero payload on PDAT is OK");
				}
			}

			switch (nextReply) {
				case Vld1Reply_t::RESP: {
					if (channel.receiveData(&(responseData->statusCode),
							receivedHeader.payloadLength) != SerialChannel::OK) {
						log_e("Timeout Reading RESP Payload");
						responseData->statusCode = Vld1ReplyStatus_t::READ_TIMEOUT;
						flushBuffer = true;
						break;
					}
					if (responseData->statusCode != Vld1ReplyStatus_t::OK) {
						log_e("VLD1 Reported Command Error");
						flushBuffer = true;
					}
					break;
				}

				case Vld1Reply_t::VERS: {
					char versionString[replyMessage.payloadLength];
					if (channel.receiveData(versionString,
							receivedHeader.payloadLength) != SerialChannel::OK) {
						log_e("Timeout Reading VERS Payload");
						responseData->statusCode = Vld1ReplyStatus_t::READ_TIMEOUT;
						flushBuffer = true;
						break;
					}
					responseData->version = versionString;
					break;
				}

				case Vld1Reply_t::RADC:
					responseData->radcData.reset(
							new std::array<RadcArray_t, radcArraySize>);
					assert((responseData->radcData != nullptr) && "radcData NULL");
					if (channel.receiveData(responseData->radcData.get(),
							radcPayloadSize) != SerialChannel::OK) {
						log_e("Timeout Reading RADC Payload");
						responseData->statusCode = Vld1ReplyStatus_t::READ_TIMEOUT;
						flushBuffer = true;
					}
					break;

				case Vld1Reply_t::RFFT:
					responseData->rfftMagData.reset(
							new std::array<RfftMagArray_t, rfftMagArrySize>);
					assert(
							(responseData->rfftMagData != nullptr)
									&& "rfftMagData NULL");
					responseData->rfftThreshData.reset(
							new std::array<RfftThreshArray_t, rfftThreshArrySize>);
					assert(
							(responseData->rfftThreshData != nullptr)
									&& "rfftThreshData NULL");
					if (channel.receiveData(responseData->rfftMagData.get(),
							rfftMagArryPayloadSize) != SerialChannel::OK) {
						log_e("Timeout Reading RFFT Mag Payload");
						responseData->statusCode = Vld1ReplyStatus_t::READ_TIMEOUT;
						flushBuffer = true;
						break;
					}
					if (channel.receiveData(responseData->rfftThreshData.get(),
							rfftThreshArryPayloadSize) != SerialChannel::OK) {
						log_e("Timeout Reading RFFT Thresh Payload");
						responseData->statusCode = Vld1ReplyStatus_t::READ_TIMEOUT;
						flushBuffer = true;
					}
					break;

				case Vld1Reply_t::PDAT:
					if (channel.receiveData(&(responseData->pdataData),
							receivedHeader.payloadLength) != SerialChannel::OK) {
						log_e("Timeout Reading PDAT Payload");
						responseData->statusCode = Vld1ReplyStatus_t::READ_TIMEOUT;
						flushBuffer = true;
					}
					break;

				case Vld1Reply_t::DONE:
					if (channel.receiveData(&(responseData->frameCoun),
							receivedHeader.payloadLength) != SerialChannel::OK) {
						log_e("Timeout Reading DONE Payload");
						responseData->statusCode = Vld1ReplyStatus_t::READ_TIMEOUT;
						flushBuffer = true;
					}
					break;

				case Vld1Reply_t::RPST:
					responseData->rpstData.reset(new RadarDataStructure_t);
					assert((responseData->rpstData != nullptr) && "rpstData NULL");
					if (channel.receiveData(responseData->rpstData.get(),
							receivedHeader.payloadLength) != SerialChannel::OK) {
						log_e("Timeout Reading RPST Payload");
						responseData->statusCode = Vld1ReplyStatus_t::READ_TIMEOUT;
						flushBuffer = true;
					}
					break;

				default:
					log_e("Unknown opCode");
					flushBuffer = true;
					break;
			}

			if (flushBuffer) {
				break;
			}
		}
		if (flushBuffer) {
			log_e("Flushing Buffer");
			channel.flush();
		}

		if ((responseData->commandSent == Vld1Command_t::INIT)
				&& (responseData->statusCode == Vld1ReplyStatus_t::OK)) {
			Vld1Baud_t newBaud = static_cast<Vld1Baud_t>(*dataPtr);
			if (newBaud != currentBaud) {
				uint32_t baud = baudMap.at(newBaud);
				log_i("Changing Baud to: %d", baud);
				channel.updateBaud(baud);
				vTaskDelay(250);
			}
			responseData->baud = newBaud;
		}

		auto ptr = responseData.get();
		BaseType_t result = xQueueSendToBack(responceQueue, &ptr, 0);
		if (result == pdTRUE) {
			log_i("Results Enqueued");
			responseData.release();
		} else {
			log_e("Results Queue Full");
		}
	}
}

void RFbeamVld1::taskLauncher(void *param) {
	TaskFunction *ptr = reinterpret_cast<TaskFunction*>(param);
	TaskFunction actualTask = std::move(*ptr);
	delete ptr;
	actualTask();
}

