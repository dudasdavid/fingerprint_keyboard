/*
 * fingerprint_lib.h
 *
 *  Created on: Jun 27, 2025
 *      Author: david.dudas
 */

#ifndef INC_FINGERPRINT_LIB_H_
#define INC_FINGERPRINT_LIB_H_



#endif /* INC_FINGERPRINT_LIB_H_ */

#include "cmsis_os.h"
#include "stm32f1xx_hal.h"

#define TX_BUFFER_SIZE 128

#define FINGERPRINT_OK 0x00
#define FINGERPRINT_NOFINGER 0x02
#define FINGERPRINT_PACKETRECIEVEERR 0x01
#define FINGERPRINT_IMAGEFAIL 0x03
#define FINGERPRINT_IMAGEMESS 0x06
#define FINGERPRINT_FEATUREFAIL 0x07
#define FINGERPRINT_NOMATCH 0x09

#define FINGERPRINT_STARTCODE 0xEF01
#define FINGERPRINT_COMMANDPACKET 0x01
#define FINGERPRINT_ACKPACKET 0x07

#define FINGERPRINT_GETIMAGE 0x01
#define FINGERPRINT_IMAGE2TZ 0x02
#define FINGERPRINT_SEARCH 0x04

#define FINGERPRINT_LEDON 0x12


#define FINGERPRINT_STARTCODE 0xEF01
#define FINGERPRINT_COMMANDPACKET 0x01
#define FINGERPRINT_ACKPACKET 0x07

#define FINGERPRINT_GETIMAGE 0x01
#define FINGERPRINT_IMAGE2TZ 0x02
#define FINGERPRINT_SEARCH 0x04

#define FINGERPRINT_ADDR 0xFFFFFFFF

#define FINGERPRINT_STARTCODE 0xEF01
#define FINGERPRINT_COMMANDPACKET 0x01
#define FINGERPRINT_RESET 0x0D
#define FINGERPRINT_ADDR 0xFFFFFFFF

#define FINGERPRINT_TIMEOUT 0xFF



HAL_StatusTypeDef receivePacket(uint8_t *buf, uint16_t bufsize);
void sendPacket(uint8_t *data, uint16_t length);
uint8_t getImage(void);
uint8_t image2Tz(uint8_t slot);
uint8_t fingerFastSearch(uint16_t *fingerID);
void resetSensor(void);
