/*
 * fingerprint_lib.c
 *
 *  Created on: Jun 27, 2025
 *      Author: david.dudas
 */
#include "fingerprint_lib.h"
#include <stdio.h>
#include "stm32f1xx_hal.h"
#include <string.h>

extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart2;
extern uint8_t txBuffer[TX_BUFFER_SIZE];

uint16_t fingerAddress = 0xFFFFFFFF;
uint8_t fingerPacket[64];

HAL_StatusTypeDef receivePacket(uint8_t *buf, uint16_t bufsize)
{
    // Basic implementation (improve as needed)
    return HAL_UART_Receive(&huart3, buf, bufsize, 1000);
}

void sendPacket(uint8_t *data, uint16_t length)
{
    uint8_t header[] = {
        (FINGERPRINT_STARTCODE >> 8) & 0xFF,
        FINGERPRINT_STARTCODE & 0xFF,
        (FINGERPRINT_ADDR >> 24) & 0xFF,
        (FINGERPRINT_ADDR >> 16) & 0xFF,
        (FINGERPRINT_ADDR >> 8) & 0xFF,
        FINGERPRINT_ADDR & 0xFF,
        FINGERPRINT_COMMANDPACKET
    };

    uint16_t packetLen = length + 2;
    uint8_t lenBuf[] = {(packetLen >> 8) & 0xFF, packetLen & 0xFF};

    uint16_t sum = FINGERPRINT_COMMANDPACKET + lenBuf[0] + lenBuf[1];
    for (int i = 0; i < length; i++) sum += data[i];

    uint8_t checksum[] = {(sum >> 8) & 0xFF, sum & 0xFF};

    HAL_UART_Transmit(&huart3, header, sizeof(header), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart3, lenBuf, sizeof(lenBuf), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart3, data, length, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart3, checksum, sizeof(checksum), HAL_MAX_DELAY);
}

uint8_t getImage(void)
{
    uint8_t packet[] = {FINGERPRINT_GETIMAGE};
    sendPacket(packet, sizeof(packet));
    uint8_t reply[12];
    if (receivePacket(reply, sizeof(reply)) != HAL_OK) return FINGERPRINT_TIMEOUT;
    return reply[9];
}

uint8_t image2Tz(uint8_t slot)
{
    uint8_t packet[] = {FINGERPRINT_IMAGE2TZ, slot};
    sendPacket(packet, sizeof(packet));
    uint8_t reply[12];
    if (receivePacket(reply, sizeof(reply)) != HAL_OK) return FINGERPRINT_TIMEOUT;
    return reply[9];
}

uint8_t fingerFastSearch(uint16_t *fingerID)
{
    uint8_t packet[] = {FINGERPRINT_SEARCH, 0x01, 0x00, 0x00, 0x00, 0xA3}; // Search 163 templates
    sendPacket(packet, sizeof(packet));
    uint8_t reply[16];
    if (receivePacket(reply, sizeof(reply)) != HAL_OK) return FINGERPRINT_TIMEOUT;

    if (reply[9] == FINGERPRINT_OK)
    {
        *fingerID = (reply[10] << 8) | reply[11];
    }
    return reply[9];
}



void resetSensor(void)
{
    uint8_t packet[] = {FINGERPRINT_RESET};
    sendPacket(packet, sizeof(packet));

    uint8_t reply[12];
    if (receivePacket(reply, sizeof(reply)) == HAL_OK)
    {
        if (reply[9] == 0x00)
        {
            // Reset acknowledged
            snprintf(txBuffer, sizeof(txBuffer), "Sensor reset OK\n");
            HAL_UART_Transmit(&huart2, (uint8_t*)txBuffer, strlen(txBuffer), HAL_MAX_DELAY);
        }
        else
        {
            snprintf(txBuffer, sizeof(txBuffer), "Sensor reset error: 0x%02X\n", reply[9]);
            HAL_UART_Transmit(&huart2, (uint8_t*)txBuffer, strlen(txBuffer), HAL_MAX_DELAY);
        }
    }
    else
    {
        snprintf(txBuffer, sizeof(txBuffer), "Sensor reset: no response\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)txBuffer, strlen(txBuffer), HAL_MAX_DELAY);
    }

    // Wait for the sensor to reboot (approx 1s)
    osDelay(1000);
}

uint8_t createModel(void)
{
    uint8_t packet[] = {FINGERPRINT_CREATE_MODEL};
    sendPacket(packet, sizeof(packet));

    uint8_t reply[12];
    if (receivePacket(reply, sizeof(reply)) != HAL_OK) return FINGERPRINT_TIMEOUT;
    return reply[9];
}

uint8_t storeModel(uint16_t location)
{
    uint8_t packet[] = {
        FINGERPRINT_STORE,
        0x01,                          // Buffer ID (Tz1)
        (location >> 8) & 0xFF,
        location & 0xFF
    };

    sendPacket(packet, sizeof(packet));

    uint8_t reply[12];
    if (receivePacket(reply, sizeof(reply)) != HAL_OK) return FINGERPRINT_TIMEOUT;
    return reply[9];
}

uint8_t enrollFinger(uint16_t id)
{
    char txBuf[64];
    uint8_t res;

    HAL_UART_Transmit(&huart2, (uint8_t*)"Place finger...\n", 16, HAL_MAX_DELAY);

    // STEP 1: Get first image
    while ((res = getImage()) != FINGERPRINT_OK)
    {
        if (res != FINGERPRINT_NOFINGER)
        {
            snprintf(txBuf, sizeof(txBuf), "Error getting image 1: 0x%02X\n", res);
            HAL_UART_Transmit(&huart2, (uint8_t*)txBuf, strlen(txBuf), HAL_MAX_DELAY);
            return 1;
        }
        osDelay(100);
    }

    // STEP 2: Convert image to template 1
    res = image2Tz(1);
    if (res != FINGERPRINT_OK)
    {
        snprintf(txBuf, sizeof(txBuf), "Error converting image 1: 0x%02X\n", res);
        HAL_UART_Transmit(&huart2, (uint8_t*)txBuf, strlen(txBuf), HAL_MAX_DELAY);
        return 2;
    }

    HAL_UART_Transmit(&huart2, (uint8_t*)"Remove finger\n", 14, HAL_MAX_DELAY);

    // STEP 3: Wait for finger removal
    uint32_t timeout = 3000;
    while (getImage() != FINGERPRINT_NOFINGER && timeout > 0)
    {
        osDelay(100);
        timeout -= 100;
    }
    if (timeout == 0)
    {
        HAL_UART_Transmit(&huart2, (uint8_t*)"Finger not removed\n", 20, HAL_MAX_DELAY);
        return 3;
    }

    HAL_UART_Transmit(&huart2, (uint8_t*)"Place same finger again\n", 24, HAL_MAX_DELAY);

    // STEP 4: Get second image
    timeout = 5000;
    while ((res = getImage()) != FINGERPRINT_OK && timeout > 0)
    {
        if (res != FINGERPRINT_NOFINGER)
        {
            snprintf(txBuf, sizeof(txBuf), "Error getting image 2: 0x%02X\n", res);
            HAL_UART_Transmit(&huart2, (uint8_t*)txBuf, strlen(txBuf), HAL_MAX_DELAY);
            return 4;
        }
        osDelay(100);
        timeout -= 100;
    }

    if (timeout == 0)
    {
        HAL_UART_Transmit(&huart2, (uint8_t*)"Timeout waiting for finger\n", 27, HAL_MAX_DELAY);
        return 5;
    }

    // STEP 5: Convert image to template 2
    res = image2Tz(2);
    if (res != FINGERPRINT_OK)
    {
        snprintf(txBuf, sizeof(txBuf), "Error converting image 2: 0x%02X\n", res);
        HAL_UART_Transmit(&huart2, (uint8_t*)txBuf, strlen(txBuf), HAL_MAX_DELAY);
        return 6;
    }

    // STEP 6: Create model from templates
    res = createModel();
    if (res != FINGERPRINT_OK)
    {
        snprintf(txBuf, sizeof(txBuf), "Error creating model: 0x%02X\n", res);
        HAL_UART_Transmit(&huart2, (uint8_t*)txBuf, strlen(txBuf), HAL_MAX_DELAY);
        return 7;
    }

    // STEP 7: Store the model
    res = storeModel(id);
    if (res != FINGERPRINT_OK)
    {
        snprintf(txBuf, sizeof(txBuf), "Error storing model: 0x%02X\n", res);
        HAL_UART_Transmit(&huart2, (uint8_t*)txBuf, strlen(txBuf), HAL_MAX_DELAY);
        return 8;
    }

    HAL_UART_Transmit(&huart2, (uint8_t*)"Enrollment successful!\n", 23, HAL_MAX_DELAY);
    return 0;
}
