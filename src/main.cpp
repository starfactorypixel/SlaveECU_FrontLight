/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

#include "main.h"
#include "fatfs.h"

#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "sd.h"
#include <LoggerLibrary.h>
#include <About.h>
#include <Leds.h>
#include <MatrixLogic.h>
#include <OutputLogic.h>
#include <CANLogic.h>

// Peripheral variables
ADC_HandleTypeDef hadc1;
CAN_HandleTypeDef hcan;
SPI_HandleTypeDef hspi2;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim2_ch1;
UART_HandleTypeDef hDebugUart;

volatile uint16_t Timer1 = 0;

// Flash related variables
FATFS SDFatFs;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_CAN_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);

/// @brief Callback function of CAN receiver.
/// @param hcan Pointer to the structure that contains CAN configuration.
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8] = {0};

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
        CANLib::can_manager.IncomingCANFrame(RxHeader.StdId, RxData, RxHeader.DLC);
        //// LOG("RX: CAN 0x%04lX", RxHeader.StdId);
    }
}

/// @brief Callback function for CAN error handler
/// @param hcan Pointer to the structure that contains CAN configuration.
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    uint32_t er = HAL_CAN_GetError(hcan);
	DEBUG_LOG("CAN ERROR: %lu %08lX", (unsigned long)er, (unsigned long)er);
}

/// @brief Sends data via CAN bus
/// @param id CANObject ID
/// @param data Pointer to the CAN frame data buffer (8 bytes max)
/// @param length Length of the CAN frame data buffer
void HAL_CAN_Send(can_object_id_t id, uint8_t *data, uint8_t length)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8] = {0};
    uint32_t TxMailbox = 0;

    TxHeader.StdId = id;                   // Standard frame ID (sets to 0 if extended one used)
    TxHeader.ExtId = 0;                    // Extended frame ID (sets to 0 if standard one used)
    TxHeader.RTR = CAN_RTR_DATA;           // CAN_RTR_DATA: CAN frame with data will be sent
                                           // CAN_RTR_REMOTE: remote CAN frame will be sent
    TxHeader.IDE = CAN_ID_STD;             // CAN_ID_STD: CAN frame with standard ID
                                           // CAN_ID_EXT: CAN frame with extended ID
    TxHeader.DLC = length;                 // Data length of the CAN frame
    TxHeader.TransmitGlobalTime = DISABLE; // Time Triggered Communication Mode

    memcpy(TxData, data, length);

    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
    {
        Leds::obj.SetOn(Leds::LED_YELLOW);
    }
    Leds::obj.SetOff(Leds::LED_YELLOW);

    if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
    {
        //LOG("CAN TX ERROR: 0x%04lX", TxHeader.StdId);
    }
}

/// @brief Flash card initialization
void InitFlash()
{
    disk_initialize(SDFatFs.drv);

    // f_mount(&SDFatFs, "", 0);
    if (f_mount(&SDFatFs, "", 1) != FR_OK)
    {
        Leds::obj.SetOn(Leds::LED_RED, 250, 250);
    }
    /*
    else
    {
        FILINFO fileInfo;
        DIR dir;
        uint8_t sect[512] = {0};
        uint8_t result = 0;
        char *fn = nullptr;

        fileInfo.lfname = (char *)sect;
        fileInfo.lfsize = sizeof(sect);
        result = f_opendir(&dir, "/");
        if (result == FR_OK)
        {
            while (1)
            {
                result = f_readdir(&dir, &fileInfo);
                if (result == FR_OK && fileInfo.fname[0])
                {
                    fn = fileInfo.lfname;
                    if (strlen(fn))
                    {
                        LOG("file name: %s%s", fn, (fileInfo.fattrib & AM_DIR) ? "   [DIR]" : "");
                    }
                    else
                    {
                        LOG("file name: %s%s", fileInfo.fname, (fileInfo.fattrib & AM_DIR) ? "   [DIR]" : "");
                    }
                }
                else
                {
                    break;
                }
            }
            f_closedir(&dir);
        }
    }
    */
}

/// @brief Peripherals initialization: GPIO, DMA, CAN, SPI, USART, ADC, Timers
void InitPeripherals()
{
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_TIM2_Init();
    MX_CAN_Init();
    MX_SPI2_Init();
    MX_FATFS_Init();
    MX_USART1_UART_Init();
    MX_TIM1_Init();
    MX_ADC1_Init();
    HAL_TIM_Base_Start_IT(&htim1);
};

/// @brief  The application entry point.
/// @retval int
int main(void)
{
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    InitPeripherals();

    // Сразу после инициализации периферии, иначе программа упадёт, если попробовать включить диод.
    // Green LED lights up, when programm falls in the Error_Handler()
    // Blue LED is unused yet
    // Red LED lights up, if flash-card initialization failed
    // Yellow LED is on while free CAN mailboxes are not available.
    // When at least one mailbox is free, LED will go off.
	About::Setup();
	Leds::Setup();
	
	/* активируем события которые будут вызывать прерывания  */
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR | CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE);

    HAL_CAN_Start(&hcan);

    InitFlash();

    CANLib::Setup();
    Matrix::Setup();
    Outputs::Setup();

    uint32_t current_time = HAL_GetTick();
    while (1)
    {
        // don't need to update current_time because it is always updated by Loop() functions
        // current_time = HAL_GetTick();
        About::Loop(current_time);
		Leds::Loop(current_time);
        CANLib::Loop(current_time);
        Matrix::Loop(current_time);
        Outputs::Loop(current_time);
    }
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
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
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
    ADC_ChannelConfTypeDef sConfig = {0};

    /* Common config */
    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }
    /* Configure Regular Channel */
    sConfig.Channel = ADC_CHANNEL_1;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief CAN Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN_Init(void)
{
    // https://istarik.ru/blog/stm32/159.html

    CAN_FilterTypeDef sFilterConfig;

    // CAN interface initialization
    hcan.Instance = CAN1;
    hcan.Init.Prescaler = 4;
    hcan.Init.Mode = CAN_MODE_NORMAL; // CAN_MODE_NORMAL
    hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
    hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan.Init.TimeTriggeredMode = DISABLE;   // DISABLE
    hcan.Init.AutoBusOff = ENABLE;           // DISABLE
    hcan.Init.AutoWakeUp = ENABLE;           // DISABLE
    hcan.Init.AutoRetransmission = DISABLE;  // DISABLE
    hcan.Init.ReceiveFifoLocked = ENABLE;    // DISABLE
    hcan.Init.TransmitFifoPriority = ENABLE; // DISABLE
    if (HAL_CAN_Init(&hcan) != HAL_OK)
    {
        Error_Handler();
    }

    // CAN filtering initialization
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    // sFilterConfig.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void)
{
    hspi2.Instance = SPI2;
    hspi2.Init.Mode = SPI_MODE_MASTER;
    hspi2.Init.Direction = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi2) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 63999;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 10;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
    {
        Error_Handler();
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 89;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_TIM_MspPostInit(&htim2);
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{
    hDebugUart.Instance = USART1;
    hDebugUart.Init.BaudRate = 500000;
    hDebugUart.Init.WordLength = UART_WORDLENGTH_8B;
    hDebugUart.Init.StopBits = UART_STOPBITS_1;
    hDebugUart.Init.Parity = UART_PARITY_NONE;
    hDebugUart.Init.Mode = UART_MODE_TX_RX;
    hDebugUart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    hDebugUart.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&hDebugUart) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{
    // DMA controller clock enable
    __HAL_RCC_DMA1_CLK_ENABLE();

    // DMA interrupt init
    // DMA1_Channel5_IRQn interrupt configuration
    HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_15, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6, GPIO_PIN_RESET);

    /*Configure GPIO pins : PC13 PC14 PC15 */
    GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pins : PA7 PA15 */
    GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : PB0 PB1 PB2 PB10
                             PB11 PB12 PB3 PB4
                             PB5 PB6 */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pin : PA8 */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : PB7 */
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pins : PB8 PB9 */
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim1)
    {
        Timer1++;
    }
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    Leds::obj.SetOff(Leds::LED_GREEN);
    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
       tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */
