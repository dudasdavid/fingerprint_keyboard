/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f1xx_hal.h"
#include "usbd_hid.h"
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "fingerprint_lib.h"
#include "aes.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_SIZE 64
#define TX_BUFFER_SIZE 64

#define FLASH_USER_START_ADDR  ((uint32_t)0x0801FC00)

//#define MAX_PWD_SIZE 48
//#define BLOCK_SIZE 8
//#define NUM_ROUNDS 32


#define ADC_MAX         4095.0f
#define V_REF           3.3f
#define ZERO_G_VOLTAGE  1.65f
#define SENSITIVITY     0.30f      // V/g for ADXL335
#define G_TO_MS2        9.80665f

//#define FLASH_PAGE_SIZE        ((uint16_t)0x400)   // 1KB on STM32F103
#define MAX_PWD_SIZE           64                  // adjust as needed
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart2_rx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for commTask */
osThreadId_t commTaskHandle;
const osThreadAttr_t commTask_attributes = {
  .name = "commTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for readerTask */
osThreadId_t readerTaskHandle;
const osThreadAttr_t readerTask_attributes = {
  .name = "readerTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for sensorTask */
osThreadId_t sensorTaskHandle;
const osThreadAttr_t sensorTask_attributes = {
  .name = "sensorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
uint8_t rxBuffer[RX_BUFFER_SIZE];
uint8_t txBuffer[TX_BUFFER_SIZE];

volatile uint16_t rxLen = 0;

char storedPassword[MAX_PWD_SIZE];

volatile bool keyboardFlag = false;
volatile bool enrollFlag = false;
volatile bool saveFlag = false;

extern USBD_HandleTypeDef hUsbDeviceFS;





static volatile uint32_t ADC_Buf[3];
static volatile uint32_t ADC_Values[3];
static float v_x = 0.0;
static float v_y = 0.0;
static float v_z = 0.0;
static float a_x_g = 0.0;
static float a_y_g = 0.0;
static float a_z_g = 0.0;
static float a_x = 0.0;
static float a_y = 0.0;
static float a_z = 0.0;
static float a_mag = 0.0;

static volatile uint32_t acc_timestamp = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
void StartDefaultTask(void *argument);
void StartCommTask(void *argument);
void StartReaderTask(void *argument);
void StartSensorTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){

  if(hadc->Instance==ADC1){
    ADC_Values[0] = ADC_Buf[0];
    ADC_Values[1] = ADC_Buf[1];
    ADC_Values[2] = ADC_Buf[2];
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_DMA(&huart2, rxBuffer, RX_BUFFER_SIZE);
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of commTask */
  commTaskHandle = osThreadNew(StartCommTask, NULL, &commTask_attributes);

  /* creation of readerTask */
  readerTaskHandle = osThreadNew(StartReaderTask, NULL, &readerTask_attributes);

  /* creation of sensorTask */
  sensorTaskHandle = osThreadNew(StartSensorTask, NULL, &sensorTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 57600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void build_response(char *dst, const char *prefix, const char *payload, size_t max_payload_len) {
    int i = 0;

    // Copy prefix (e.g., "OK:" or "ERR:")
    while (prefix[i] != '\0') {
        dst[i] = prefix[i];
        i++;
    }

    // Copy up to max_payload_len characters from payload
    for (size_t j = 0; j < max_payload_len && payload[j] != '\0'; j++) {
        dst[i++] = payload[j];
    }

    // Add newline and null terminator
    dst[i++] = '\n';
    dst[i] = '\0';
}



void xtea_encrypt(uint32_t v[2], const uint32_t key[4], unsigned int rounds) {
    uint32_t v0 = v[0], v1 = v[1], sum = 0, delta = 0x9E3779B9;
    for (unsigned int i = 0; i < rounds; i++) {
        v0 += (((v1 << 4) ^ (v1 >> 5)) + v1) ^ (sum + key[sum & 3]);
        sum += delta;
        v1 += (((v0 << 4) ^ (v0 >> 5)) + v0) ^ (sum + key[(sum >> 11) & 3]);
    }
    v[0] = v0;
    v[1] = v1;
}

void xtea_decrypt(uint32_t v[2], const uint32_t key[4], unsigned int rounds) {
    uint32_t v0 = v[0], v1 = v[1];
    uint32_t sum = 0x9E3779B9 * rounds, delta = 0x9E3779B9;
    for (unsigned int i = 0; i < rounds; i++) {
        v1 -= (((v0 << 4) ^ (v0 >> 5)) + v0) ^ (sum + key[(sum >> 11) & 3]);
        sum -= delta;
        v0 -= (((v1 << 4) ^ (v1 >> 5)) + v1) ^ (sum + key[sum & 3]);
    }
    v[0] = v0;
    v[1] = v1;
}

void deriveKeyFromUID(uint8_t *key)
{
    uint8_t *uid = (uint8_t *)0x1FFFF7E8; // STM32F103 UID base address
    for (int i = 0; i < 12; i++)
    {
        key[i] = uid[i] ^ 0xA5; // basic XOR obfuscation
    }
    key[12] = 0xDE;
    key[13] = 0xAD;
    key[14] = 0xBE;
    key[15] = 0xEF;
}

void generateIV(uint8_t *iv)
{
    uint32_t seed = HAL_GetTick(); // not cryptographically secure, but okay for basic protection
    srand(seed);
    for (int i = 0; i < 16; i++)
    {
        iv[i] = rand() & 0xFF;
    }
}

/*
void writePasswordToFlash(const char *password)
{
    HAL_FLASH_Unlock();

    // Erase the page first
    FLASH_EraseInitTypeDef eraseInit;
    uint32_t pageError = 0;

    eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseInit.PageAddress = FLASH_USER_START_ADDR;
    eraseInit.NbPages = 1;

    if (HAL_FLASHEx_Erase(&eraseInit, &pageError) != HAL_OK)
    {
        // Handle erase error here
        HAL_FLASH_Lock();
        return;
    }

    // Write the password including null terminator
    uint32_t addr = FLASH_USER_START_ADDR;
    const uint8_t *data = (const uint8_t *)password;
    size_t totalBytes = strlen(password) + 1; // +1 for '\0'

    for (size_t i = 0; i < totalBytes; i += 2)
    {
        uint16_t halfWord = 0xFFFF; // default padding

        halfWord = data[i];
        if ((i + 1) < totalBytes)
        {
            halfWord |= (data[i + 1] << 8);
        }

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, halfWord) != HAL_OK)
        {
            // Handle error
            break;
        }

        addr += 2;
    }

    HAL_FLASH_Lock();
}
*/
void writePasswordToFlash(const char *password)
{
    HAL_FLASH_Unlock();

    // Erase the flash page
    FLASH_EraseInitTypeDef eraseInit;
    uint32_t pageError = 0;
    eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseInit.PageAddress = FLASH_USER_START_ADDR;
    eraseInit.NbPages = 1;
    if (HAL_FLASHEx_Erase(&eraseInit, &pageError) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return;
    }

    // Prepare AES encryption
    struct AES_ctx ctx;
    uint8_t key[16];
    deriveKeyFromUID(key);

    uint8_t iv[16];
    generateIV(iv);

    // Pad the password to 16-byte block (PKCS#7)
    uint8_t buffer[64] = {0}; // MAX password size + padding
    size_t len = strlen(password);
    size_t padded_len = ((len / 16) + 1) * 16;
    uint8_t pad = padded_len - len;

    memcpy(buffer, password, len);
    for (size_t i = len; i < padded_len; i++) buffer[i] = pad;

    AES_init_ctx_iv(&ctx, key, iv);
    AES_CBC_encrypt_buffer(&ctx, buffer, padded_len);

    // Write to flash: [ IV (16B) ][ LEN (2B) ][ Encrypted Data ]
    uint32_t addr = FLASH_USER_START_ADDR;

    // Write IV
    for (int i = 0; i < 16; i += 2)
    {
        uint16_t halfWord = iv[i] | (iv[i + 1] << 8);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, halfWord);
        addr += 2;
    }

    // Write length (uint16_t)
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, (uint16_t)padded_len);
    addr += 2;

    // Write encrypted buffer
    for (int i = 0; i < padded_len; i += 2)
    {
        uint16_t halfWord = buffer[i] | (buffer[i + 1] << 8);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, halfWord);
        addr += 2;
    }

    HAL_FLASH_Lock();
}


/*
void readPasswordFromFlash(char *password)
{
    const uint8_t *data = (const uint8_t *)FLASH_USER_START_ADDR;
    for (uint32_t i = 0; i < MAX_PWD_SIZE; i++)
    {
        password[i] = data[i];
        if (data[i] == '\0')
        {
            break;
        }
    }
    password[MAX_PWD_SIZE - 1] = '\0'; // Safety null-termination
}
*/
void readPasswordFromFlash(char *outPassword)
{
    uint8_t key[16], iv[16], encrypted[64] = {0};
    deriveKeyFromUID(key);

    const uint8_t *flash_ptr = (uint8_t *)FLASH_USER_START_ADDR;

    // Read IV
    memcpy(iv, flash_ptr, 16);
    flash_ptr += 16;

    // Read length (2 bytes)
    uint16_t padded_len = flash_ptr[0] | (flash_ptr[1] << 8);
    flash_ptr += 2;

    if (padded_len > sizeof(encrypted)) padded_len = sizeof(encrypted);  // safety

    // Read encrypted data
    memcpy(encrypted, flash_ptr, padded_len);

    // Decrypt
    struct AES_ctx ctx;
    AES_init_ctx_iv(&ctx, key, iv);
    AES_CBC_decrypt_buffer(&ctx, encrypted, padded_len);

    // Remove PKCS#7 padding
    uint8_t pad = encrypted[padded_len - 1];
    if (pad <= 16)
    {
        encrypted[padded_len - pad] = '\0';
    }
    else
    {
        encrypted[padded_len - 1] = '\0';
    }

    strncpy(outPassword, (char *)encrypted, MAX_PWD_SIZE - 1);
    outPassword[MAX_PWD_SIZE - 1] = '\0';
}



void calculate_acceleration(void) {

    // Convert ADC to voltage
    v_x = (ADC_Values[0] / ADC_MAX) * V_REF;
    v_y = (ADC_Values[1] / ADC_MAX) * V_REF;
    v_z = (ADC_Values[2] / ADC_MAX) * V_REF;

    // Convert voltage to acceleration in g
    a_x_g = (v_x - ZERO_G_VOLTAGE) / SENSITIVITY;
    a_y_g = (v_y - ZERO_G_VOLTAGE) / SENSITIVITY;
    a_z_g = (v_z - ZERO_G_VOLTAGE) / SENSITIVITY;

    // Convert g to m/s²
    a_x = a_x_g * G_TO_MS2;
    a_y = a_y_g * G_TO_MS2;
    a_z = a_z_g * G_TO_MS2;

    // Vector magnitude
    a_mag = sqrtf(a_x * a_x + a_y * a_y + a_z * a_z);

    // Print or use the values as needed
    //printf("Accel (m/s²): X=%.2f Y=%.2f Z=%.2f | |A|=%.2f\n", a_x, a_y, a_z, a_mag);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */

  // source: https://notes.iopush.net/blog/2016/stm32-custom-usb-hid-step-by-step-2/

  // HID Keyboard
  struct keyboardHID_t {
      uint8_t id;
      uint8_t modifiers;
      uint8_t key1;
      uint8_t key2;
      uint8_t key3;
  };
  struct keyboardHID_t keyboardHID;
  keyboardHID.id = 1;
  keyboardHID.modifiers = 0;
  keyboardHID.key1 = 0;
  keyboardHID.key2 = 0;
  keyboardHID.key3 = 0;


  /* Infinite loop */
  for(;;)
  {
		osDelay(500);
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

        if (keyboardFlag)
        {
            // Type the stored password
            size_t pwdLen = strlen(storedPassword);

            for (size_t i = 0; i < pwdLen; i++)
            {
                char c = storedPassword[i];
                uint8_t keycode = 0;
                uint8_t modifier = 0;

                // Translate ASCII to HID keycode
                if (c >= 'a' && c <= 'z')
                {
                    keycode = (c - 'a') + 0x04; // HID codes for 'a' to 'z'
                }
                else if (c >= 'A' && c <= 'Z')
                {
                    keycode = (c - 'A') + 0x04;
                    modifier = 0x02; // Left Shift
                }
                else if (c >= '0' && c <= '9')
                {
                    keycode = (c == '0') ? 0x27 : (c - '1') + 0x1E;
                }
                else if (c == '-')
                {
                    keycode = 0x38;
                }
                else if (c == '\0')
                {
                    break;
                }
                else
                {
                    // Handle other characters as needed
                    continue;
                }

                keyboardHID.modifiers = modifier;
                keyboardHID.key1 = keycode;
                USBD_HID_SendReport(&hUsbDeviceFS, &keyboardHID, sizeof(struct keyboardHID_t));
                HAL_Delay(30);

                // Release key
                keyboardHID.modifiers = 0;
                keyboardHID.key1 = 0;
                USBD_HID_SendReport(&hUsbDeviceFS, &keyboardHID, sizeof(struct keyboardHID_t));
                HAL_Delay(30);
            }

            // After typing the password
            keyboardHID.modifiers = 0;
            keyboardHID.key1 = 0x28;  // HID keycode for Enter
            USBD_HID_SendReport(&hUsbDeviceFS, &keyboardHID, sizeof(struct keyboardHID_t));
            HAL_Delay(30);

            // Release Enter
            keyboardHID.key1 = 0;
            USBD_HID_SendReport(&hUsbDeviceFS, &keyboardHID, sizeof(struct keyboardHID_t));
            HAL_Delay(30);

            keyboardFlag = false;
        }

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartCommTask */
/**
* @brief Function implementing the commTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCommTask */
void StartCommTask(void *argument)
{
  /* USER CODE BEGIN StartCommTask */
  readPasswordFromFlash(storedPassword);

  /* Infinite loop */
  for(;;)
  {
      // Wait for notification from ISR
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

      // Process received data
      if(rxLen > 0)
      {
          // Look for '\n'
          char* newlinePos = memchr(rxBuffer, '\n', rxLen);
          if(newlinePos)
          {
              size_t msgLen = newlinePos - (char*)rxBuffer;
              if(msgLen >= RX_BUFFER_SIZE) msgLen = RX_BUFFER_SIZE - 1;

              // Null-terminate
              rxBuffer[msgLen] = '\0';

              // Build response
              // Check for prefix "PWD:"
              if (strncmp((char*)rxBuffer, "PWD:", 4) == 0)
              {
                  char *pwdStart = (char*)rxBuffer + 4;
                  snprintf((char*)txBuffer, TX_BUFFER_SIZE, "OK:%.50s\n", rxBuffer+4);
                  //build_response(txBuffer, "OK:", pwdStart, 50);
                  saveFlag = true;
                  osDelay(500);
                  writePasswordToFlash(pwdStart);
                  strncpy(storedPassword, pwdStart, MAX_PWD_SIZE - 1);
                  storedPassword[MAX_PWD_SIZE - 1] = '\0';
                  osDelay(500);
                  saveFlag = false;
              }
              /*
              else if (strncmp((char*)rxBuffer, "READ", 4) == 0)
              {
                  snprintf((char*)txBuffer, TX_BUFFER_SIZE, "READ:%.50s\n", storedPassword);
            	  //build_response(txBuffer, "READ:", storedPassword, 50);
              }
              else if (strncmp((char*)rxBuffer, "SEND", 4) == 0)
              {
            	  keyboardFlag = true;
                  snprintf((char*)txBuffer, TX_BUFFER_SIZE, "SEND:%.50s\n", storedPassword);
                  //build_response(txBuffer, "SEND:", storedPassword, 50);
              }
              */
              else if (strncmp((char*)rxBuffer, "ENROLL:", 7) == 0)
              {
                  char ch = rxBuffer[7];

                  // Check if it's a digit between '1' and '9'
                  if (ch >= '1' && ch <= '9')
                  {
                      uint8_t id = ch - '0';

                      snprintf(txBuffer, TX_BUFFER_SIZE, "ENROLL FINGER:%d\n", id);
                      HAL_UART_Transmit(&huart2, txBuffer, strlen((char*)txBuffer), HAL_MAX_DELAY);
                      enrollFlag = true;
                      osDelay(500);
                      enrollFinger(id);  // start enrollment
                      snprintf(txBuffer, TX_BUFFER_SIZE, "ENROLL FINISHED:%d\n", id);
                      osDelay(500);
                      enrollFlag = false;
                  }
                  else
                  {
                      snprintf(txBuffer, TX_BUFFER_SIZE, "ENROLL FINGER: ERROR\n");
                  }
              }
              else
              {
                  snprintf((char*)txBuffer, TX_BUFFER_SIZE, "ERR:%.50s\n", rxBuffer);
                  //build_response(txBuffer, "ERR:", rxBuffer, 50);
              }

              // Send response
              HAL_UART_Transmit(&huart2, txBuffer, strlen((char*)txBuffer), HAL_MAX_DELAY);
          }
      }

      // Reset buffer length
      rxLen = 0;
  }
  /* USER CODE END StartCommTask */
}

/* USER CODE BEGIN Header_StartReaderTask */
/**
* @brief Function implementing the readerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartReaderTask */
void StartReaderTask(void *argument)
{
  /* USER CODE BEGIN StartReaderTask */
	//char idStr[10];
	uint8_t error03Count = 0;
	/* Infinite loop */
    for (;;)
    {
    	if (HAL_GetTick() - acc_timestamp > 30000) {
    		osDelay(200);
    		continue;
    	}
    	if (enrollFlag || saveFlag)
    	{
    		osDelay(200);
    		continue;
    	}
    	if (keyboardFlag)
    	{
    		osDelay(200);
    		continue;
    	}
        uint8_t p = getImage();
        if (p == FINGERPRINT_NOFINGER)
        {
            osDelay(200);
            continue;
        }
        if (p != FINGERPRINT_OK)
        {
            snprintf(txBuffer, sizeof(txBuffer), "Image error: 0x%02X\n", p);
            //build_response(txBuffer, "Image error", ".", 50);
            HAL_UART_Transmit(&huart2, (uint8_t*)txBuffer, strlen(txBuffer), HAL_MAX_DELAY);
            osDelay(1000);

            error03Count++;
            if (error03Count >= 3)
            {
                snprintf(txBuffer, sizeof(txBuffer), "Too many image errors, resetting sensor...\n");
                //build_response(txBuffer, "Too many image errors, resetting", ".", 50);
                HAL_UART_Transmit(&huart2, (uint8_t*)txBuffer, strlen(txBuffer), HAL_MAX_DELAY);

                //resetSensor();
                error03Count = 0;
            }

            continue;
        }
        else {
        	error03Count = 0;
        }

        p = image2Tz(1);
        if (p != FINGERPRINT_OK)
        {
            snprintf(txBuffer, sizeof(txBuffer), "Image conversion error: 0x%02X\n", p);
            //build_response(txBuffer, "Image conversion error", ".", 50);
            HAL_UART_Transmit(&huart2, (uint8_t*)txBuffer, strlen(txBuffer), HAL_MAX_DELAY);
            osDelay(1000);
            continue;
        }

        uint16_t fingerID = 0;
        p = fingerFastSearch(&fingerID);
        if (p == FINGERPRINT_OK)
        {
            snprintf(txBuffer, sizeof(txBuffer), "Finger %d recognized\n", fingerID);
        	//itoa(fingerID, idStr, 10);
        	//build_response(txBuffer, "Finger recognized: ", idStr, 10);
            HAL_UART_Transmit(&huart2, (uint8_t*)txBuffer, strlen(txBuffer), HAL_MAX_DELAY);

            // Set flag to type password
            keyboardFlag = true;
            // Set accelerometer magnitude timestamp to 0 so reading turns off immediately after successful finger scanning
            acc_timestamp = 0;
        }
        else if (p == FINGERPRINT_NOMATCH)
        {
            snprintf(txBuffer, sizeof(txBuffer), "Finger not recognized\n");
            //build_response(txBuffer, "Finger not recognized", ".", 50);
            HAL_UART_Transmit(&huart2, (uint8_t*)txBuffer, strlen(txBuffer), HAL_MAX_DELAY);
            acc_timestamp = HAL_GetTick(); // misuse this timestamp to avoid going back sleeping while trying to log in
        }
        else
        {
            snprintf(txBuffer, sizeof(txBuffer), "Search error: 0x%02X\n", p);
            //build_response(txBuffer, "Search error", ".", 50);
            HAL_UART_Transmit(&huart2, (uint8_t*)txBuffer, strlen(txBuffer), HAL_MAX_DELAY);
        }

        osDelay(1000);  // Wait before next scan
    }

  /* USER CODE END StartReaderTask */
}

/* USER CODE BEGIN Header_StartSensorTask */
/**
* @brief Function implementing the sensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void *argument)
{
  /* USER CODE BEGIN StartSensorTask */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Buf,3);
  /* Infinite loop */
  for(;;)
  {
	calculate_acceleration();
	if (a_mag > 20.0){
		acc_timestamp = HAL_GetTick();
	}
    osDelay(10);
  }
  /* USER CODE END StartSensorTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
