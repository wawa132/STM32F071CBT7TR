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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// 120.9ohm x 4mA = 0.4836V -> 0.4836V/3.3V x 4095 = 600
#define RATE_A 4167
#define ZERO_A 600
#define ADC_NUM 4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Read_Sensor_ID(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char txBuff[350], txNewBuff[100], rxBuff[30], reqBuff[30], rxData, usrBuff[30],
    dataBuff[30], usrData, gwData, gwBuff[30];
uint8_t senID[4], adcFlag, sendFlag, reqFlag, usrFlag,
    reqCnt = 20, reqNum, com_err, sensorNum, I2CBuff[16], ReadI2C[16];
uint16_t adcRaw[40], adcData[4], adcAvg[4], txCnt = 1000, adcCnt = 300;
uint32_t adcSum[4], adcRawSum[4];
int32_t senValue[100], adcValue[4];
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
  MX_ADC_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART4_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
    Read_Sensor_ID();
    HAL_ADCEx_Calibration_Start(&hadc);
    HAL_ADC_Start_DMA(&hadc, (uint32_t *)&adcRaw, 40);
    HAL_UART_Receive_IT(&huart4, (uint8_t *)&rxData, 1);
    HAL_UART_Receive_IT(&huart2, (uint8_t *)&gwData, 1);
    HAL_UART_Receive_IT(&huart1, (uint8_t *)&usrData, 1);
    HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

        if (adcFlag) // 4-20mA ADC 연산
        {
            adcRawSum[0] = adcRaw[0] + adcRaw[4] + adcRaw[8] + adcRaw[12] + adcRaw[16] + adcRaw[20] + adcRaw[24] + adcRaw[28] + adcRaw[32] + adcRaw[36];
            adcRawSum[1] = adcRaw[1] + adcRaw[5] + adcRaw[9] + adcRaw[13] + adcRaw[17] + adcRaw[21] + adcRaw[25] + adcRaw[29] + adcRaw[33] + adcRaw[37];
            adcRawSum[2] = adcRaw[2] + adcRaw[6] + adcRaw[10] + adcRaw[14] + adcRaw[18] + adcRaw[22] + adcRaw[26] + adcRaw[30] + adcRaw[34] + adcRaw[38];
            adcRawSum[3] = adcRaw[3] + adcRaw[7] + adcRaw[11] + adcRaw[15] + adcRaw[19] + adcRaw[23] + adcRaw[27] + adcRaw[31] + adcRaw[35] + adcRaw[39];

            adcData[0] = adcRawSum[0] / 10;
            adcData[1] = adcRawSum[1] / 10;
            adcData[2] = adcRawSum[2] / 10;
            adcData[3] = adcRawSum[3] / 10;

            adcRawSum[0] = 0;
            adcRawSum[1] = 0;
            adcRawSum[2] = 0;
            adcRawSum[3] = 0;

            adcSum[0] += adcData[0];
            adcSum[1] += adcData[1];
            adcSum[2] += adcData[2];
            adcSum[3] += adcData[3];

            if (adcCnt > 0)
                adcCnt--;

            if (adcCnt == 0)
            {
                adcAvg[0] = adcSum[0] / 300;
                adcAvg[1] = adcSum[1] / 300;
                adcAvg[2] = adcSum[2] / 300;
                adcAvg[3] = adcSum[3] / 300;

                adcSum[0] = 0;
                adcSum[1] = 0;
                adcSum[2] = 0;
                adcSum[3] = 0;

                adcValue[0] = ((adcAvg[0] - ZERO_A) * RATE_A) / 100000;
                adcValue[1] = ((adcAvg[1] - ZERO_A) * RATE_A) / 100000;
                adcValue[2] = ((adcAvg[2] - ZERO_A) * RATE_A) / 100000;
                adcValue[3] = ((adcAvg[3] - ZERO_A) * RATE_A) / 100000;

                senValue[senID[0] - 1] = adcValue[0];
                senValue[senID[1] - 1] = adcValue[1];
                senValue[senID[2] - 1] = adcValue[2];
                senValue[senID[3] - 1] = adcValue[3];

                // 환경공단 테스트 설정 ADC1: 배출시설(E0101), ADC2: 송풍시설(F0001), ADC3: 배출시설(E8001), ADC4: 송풍시설(F8001)
                // ADC3, ADC4 0으로 설정
                senValue[senID[2] - 1] = 0;
                senValue[senID[3] - 1] = 0;

                adcCnt = 300;
            }

            adcFlag = 0;
        }

        if (reqFlag) // 센서보드로 데이터 요청: R01\r\n(R: 요청, 01: ID)
        {
            if (com_err)
            {
                senValue[reqNum - 1] = 666;
                com_err = 0; // 무응답 통신불량 처리 후, 응답에러 플래그 해제
            }

            if (reqNum >= sensorNum)
                reqNum = 0;

            reqNum++;

            if (reqNum != senID[0] && reqNum != senID[1] && reqNum != senID[2] && reqNum != senID[3])
            {
                // ex.R01\r\n
                snprintf(reqBuff, sizeof(reqBuff), "R%02d%c%c%c", reqNum, 0x0D, 0x0A, 0x00);
                HAL_UART_Transmit_DMA(&huart4, (uint8_t *)&reqBuff, strlen(reqBuff));
            }

            reqFlag = 0;
        }

        if (sendFlag) // 게이트웨이로 센서 데이터 전송
        {
            snprintf(txBuff, sizeof(txBuff), "%c", 0x02);

            for (int i = 0; i < 100; i++)
            {
                if (i >= sensorNum)
                {
                    snprintf(txNewBuff, sizeof(txNewBuff), "000");
                }
                else
                {
                    if (senValue[i] < 0)
                    {
                        uint32_t minus_data = abs(senValue[i]);

                        if (minus_data > 100)
                            minus_data = 99;

                        snprintf(txNewBuff, sizeof(txNewBuff), "-%02ld", minus_data);
                    }
                    else
                    {
                        if (senValue[i] > 999)
                            senValue[i] = 999;

                        snprintf(txNewBuff, sizeof(txNewBuff), "%03ld", senValue[i]);
                    }
                }
                strcat(txBuff, txNewBuff);
            }

            snprintf(txNewBuff, sizeof(txNewBuff), "%c%c%c%c", 0x03, 0x0D, 0x0A, 0x00);
            strcat(txBuff, txNewBuff);

            HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&txBuff, strlen(txBuff));

            sendFlag = 0;
        }

        if (usrFlag) // 전체 센서 갯수, 계측 센서 ID
        {
            // ex.Total num and PortA ID: 01, PortB ID: 02... T07PA01PB06PC03PD07\r\n
            snprintf(dataBuff, sizeof(dataBuff), "T%02dA1%02dA2%02dA3%02dA4%02d%c%c%c",
                     sensorNum, senID[0], senID[1], senID[2], senID[3], 0x0D, 0x0A, 0x00);
            HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&dataBuff, strlen(dataBuff));

            usrFlag = 0;
        }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        if (txCnt > 0)
            txCnt--;
        if (reqCnt)
            reqCnt--;

        // 1msec 간격 ADC
        adcFlag = 1;

        if (txCnt == 0)
        { // 1sec 간격 센서 데이터 전송
            txCnt = 1000;
            sendFlag = 1;
            usrFlag = 1;
        }

        if (reqCnt == 0)
        { // 20msec 간격 센서 데이터 요청
            reqCnt = 20;
            reqFlag = 1;
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART4)
    {
        // 센서 데이터 수신: A0100000\r\n(A: 응답, 01: ID, 000: 데이터, 0: checksum >> 8, 0: checkSum & 0xFF)
        // 체크섬의 경우, A01000 모두 더한 uint16_t 값

        static uint8_t rxNum;

        rxBuff[rxNum] = rxData;
        rxNum++;

        if (rxNum > 10)
            rxNum = 0;

        if (rxNum >= 2)
        {
            if (rxBuff[rxNum - 10] == 'A' && rxBuff[rxNum - 2] == 0x0D && rxBuff[rxNum - 1] == 0x0A)
            {
                uint8_t id;
                uint16_t checkSum, rxCheckSum;

                id = (rxBuff[rxNum - 9] - '0') * 10;
                id += (rxBuff[rxNum - 8] - '0');

                if (id == reqNum)
                {
                    checkSum = (rxBuff[rxNum - 10] + rxBuff[rxNum - 9] + rxBuff[rxNum - 8] + rxBuff[rxNum - 7] + rxBuff[rxNum - 6] + rxBuff[rxNum - 5]);
                    rxCheckSum = (rxBuff[rxNum - 4] << 8) | rxBuff[rxNum - 3];

                    if (checkSum == rxCheckSum)
                    {
                        int16_t data;

                        if (rxBuff[rxNum - 7] == '-')
                        {
                            data = (rxBuff[rxNum - 6] - '0') * 10;
                            data += (rxBuff[rxNum - 5] - '0');
                            data = (0 - 1) * data;
                        }
                        else
                        {
                            data = (rxBuff[rxNum - 7] - '0') * 100;
                            data += (rxBuff[rxNum - 6] - '0') * 10;
                            data += (rxBuff[rxNum - 5] - '0');
                        }

                        senValue[id - 1] = data;
                    }
                    else
                    {
                        senValue[id - 1] = 666; // 통신불량: 체크섬 불일치
                    }
                }
                else
                {
                    senValue[id - 1] = 666; // 통신불량: 응답아이디 불일치
                }

                reqCnt = 1;  // 응답 수신 시, 1msec 딜레이 후 다음 데이터 요청
                com_err = 0; // 응답 수신 시, 응답에러 플래그 해제
                rxNum = 0;
            }
        }
        HAL_UART_Receive_IT(&huart4, (uint8_t *)&rxData, 1);
    }

    if (huart->Instance == USART2)
    { // 센서 갯수 설정 ex. AS07\r\n(센서 갯수 7개)

        static uint8_t gwNum;

        gwBuff[gwNum] = gwData;
        gwNum++;

        if (gwNum > 6)
            gwNum = 0;

        if (gwBuff[gwNum - 6] == 'A' && gwBuff[gwNum - 2] == 0x0D && gwBuff[gwNum - 1] == 0x0A)
        {
            I2CBuff[0] = (gwBuff[gwNum - 4] - '0') * 10;
            I2CBuff[0] += (gwBuff[gwNum - 3] - '0');

            if (gwBuff[gwNum - 5] == 'S')
            {
                sensorNum = I2CBuff[0];
                HAL_I2C_Mem_Write(&hi2c1, 0xA0, 0x48, I2C_MEMADD_SIZE_8BIT, &I2CBuff[0], 1, 1);

                gwNum = 0;
            }
        }
        HAL_UART_Receive_IT(&huart2, (uint8_t *)&gwData, 1);
    }

    if (huart->Instance == USART1)
    { // 센서 갯수 및 ID 설정 ex. AS07\r\n(센서 갯수 7개) or A103(ADC1 ID: 3번)
        char tmpBuff[50];

        usrBuff[0] = usrBuff[1];
        usrBuff[1] = usrBuff[2];
        usrBuff[2] = usrBuff[3];
        usrBuff[3] = usrBuff[4];
        usrBuff[4] = usrBuff[5];
        usrBuff[5] = usrData;

        if (usrBuff[0] == 'A' && usrBuff[4] == 0x0D && usrBuff[5] == 0x0A)
        {
            I2CBuff[0] = (usrBuff[2] - '0') * 10;
            I2CBuff[0] += (usrBuff[3] - '0');

            if (usrBuff[1] == 'S') // 총 센서 갯수
            {
                sensorNum = I2CBuff[0];
                HAL_I2C_Mem_Write(&hi2c1, 0xA0, 0x48, I2C_MEMADD_SIZE_8BIT, &I2CBuff[0], 1, 1);
                snprintf(tmpBuff, sizeof(tmpBuff), "Total Sensor Num: %2d\r\n", I2CBuff[0]);
            }
            else if (usrBuff[1] == '1') // 1번 ADC ID
            {
                senID[0] = I2CBuff[0];
                HAL_I2C_Mem_Write(&hi2c1, 0xA0, 0x40, I2C_MEMADD_SIZE_8BIT, &I2CBuff[0], 1, 1);
                snprintf(tmpBuff, sizeof(tmpBuff), "ADC1 ID: %2d\r\n", I2CBuff[0]);
            }
            else if (usrBuff[1] == '2')
            {
                senID[1] = I2CBuff[0];
                HAL_I2C_Mem_Write(&hi2c1, 0xA0, 0x42, I2C_MEMADD_SIZE_8BIT, &I2CBuff[0], 1, 1);
                snprintf(tmpBuff, sizeof(tmpBuff), "ADC2 ID: %2d\r\n", I2CBuff[0]);
            }
            else if (usrBuff[1] == '3')
            {
                senID[2] = I2CBuff[0];
                HAL_I2C_Mem_Write(&hi2c1, 0xA0, 0x44, I2C_MEMADD_SIZE_8BIT, &I2CBuff[0], 1, 1);
                snprintf(tmpBuff, sizeof(tmpBuff), "ADC3 ID: %2d\r\n", I2CBuff[0]);
            }
            else if (usrBuff[1] == '4')
            {
                senID[3] = I2CBuff[0];
                HAL_I2C_Mem_Write(&hi2c1, 0xA0, 0x46, I2C_MEMADD_SIZE_8BIT, &I2CBuff[0], 1, 1);
                snprintf(tmpBuff, sizeof(tmpBuff), "ADC4 ID: %2d\r\n", I2CBuff[0]);
            }
            else
            {
                snprintf(tmpBuff, sizeof(tmpBuff), "Wrong Protocol: %s", usrBuff);
            }

            HAL_UART_Transmit_DMA(&huart1, (uint8_t *)tmpBuff, strlen(tmpBuff));
        }
        HAL_UART_Receive_IT(&huart1, (uint8_t *)&usrData, 1);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART4)
    {
        reqCnt += 10; // 전송 완료 후, 응답 수신 10msec 딜레이
        com_err = 1;  // 전송 완료 후, 응답에러 플래그 설정
    }
}

void Read_Sensor_ID(void)
{
    HAL_I2C_Mem_Read(&hi2c1, 0xA0, 0x40, I2C_MEMADD_SIZE_8BIT, &ReadI2C[0], 16, 1);

    for (uint8_t i = 0; i < ADC_NUM; i++)
    {
        senID[i] = ReadI2C[i * 2];
    }

    sensorNum = ReadI2C[8]; // Main 보드에만 해당
}
/* USER CODE END 4 */

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
    printf("Wrong parameters value: file %s on line %ld\r\n", file, line);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
