/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
    PMIC_STATUS1 = 0x27,
    PMIC_STATUS2 = 0x28
} PMIC_StatusReg_t;

typedef union
{
	uint8_t value;
	struct
	{
	    uint8_t PGBUCK1   : 1;   // D0
	    uint8_t PGBUCK2   : 1;   // D1
	    uint8_t PGBUCK3   : 1;   // D2
	    uint8_t PGBUCK4   : 1;   // D3
	    uint8_t reserved1 : 1;   // D4
	    uint8_t PGLDO2    : 1;   // D5
	    uint8_t reserved2 : 1;   // D6
	    uint8_t PGLDO4    : 1;   // D7
	}bit;
} PMIC_Status1_t;

typedef union
{
	uint8_t value;
	struct
	{
		uint8_t reserved1 : 1;
		uint8_t reserved2 : 1;
		uint8_t reserved3 : 1;
		uint8_t reserved4 : 1;
		uint8_t checksumFlag : 1;
		uint8_t reserved5 : 1;
		uint8_t otempp : 1;
		uint8_t otwarning : 1;
	} bit;
} PMIC_Status2_t;

typedef union
{
    uint8_t value;

    struct
    {
        uint8_t testFailed                         : 1;  // bit0
        uint8_t testFailedThisOperationCycle      : 1;  // bit1
        uint8_t pendingDTC                        : 1;  // bit2
        uint8_t confirmedDTC                      : 1;  // bit3
        uint8_t testNotCompletedSinceLastClear    : 1;  // bit4
        uint8_t testFailedSinceLastClear          : 1;  // bit5
        uint8_t testNotCompletedThisOperationCycle: 1;  // bit6
        uint8_t warningIndicatorRequested         : 1;  // bit7
    } bit;

} DTC_Status_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define SLAVE_ADDR 0x69

#define PMIC_BUCK4_VREF_REG     0x0F
#define PMIC_BUCK4_1V2_CODE     0x40

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c2_rx;
DMA_HandleTypeDef hdma_i2c2_tx;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

UART_HandleTypeDef huart4;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for I2CTask */
osThreadId_t I2CTaskHandle;
const osThreadAttr_t I2CTask_attributes = {
  .name = "I2CTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SPITask */
osThreadId_t SPITaskHandle;
const osThreadAttr_t SPITask_attributes = {
  .name = "SPITask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CANTask */
osThreadId_t CANTaskHandle;
const osThreadAttr_t CANTask_attributes = {
  .name = "CANTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UARTTask */
osThreadId_t UARTTaskHandle;
const osThreadAttr_t UARTTask_attributes = {
  .name = "UARTTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CanQueue */
osMessageQueueId_t CanQueueHandle;
const osMessageQueueAttr_t CanQueue_attributes = {
  .name = "CanQueue"
};
/* Definitions for CommMutexHandle */
osMutexId_t CommMutexHandleHandle;
const osMutexAttr_t CommMutexHandle_attributes = {
  .name = "CommMutexHandle"
};
/* USER CODE BEGIN PV */
//TEST num5
volatile uint8_t pmic_voltage_change_request = 1;
volatile uint8_t pmic_voltage_write_busy = 0;

uint8_t buck4_vref_data = PMIC_BUCK4_1V2_CODE;


//PMIC
PMIC_Status1_t status1;
PMIC_Status2_t status2;

volatile uint8_t i2c_step = 0;

//DTC
typedef struct
{
    uint8_t dtc[3];
    DTC_Status_t  status;
} DTC_Record_t;

DTC_Record_t brake_dtc;

volatile uint8_t dtc_save_request = 0;


//spi
uint8_t spiTxData[4];
uint8_t eeprom_mock[4];
uint8_t eeprom_read_data[4];

volatile uint8_t spi_busy = 0;


volatile uint8_t dtc_read_ready = 0;
volatile uint8_t eeprom_error = 0;

//CAN RX
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];

volatile uint8_t can_rx_flag = 0;

//CAN TX
CAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];
uint32_t TxMailbox;

//UART
uint8_t eeprom_mock[4];
uint8_t eeprom_read_data[4];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_UART4_Init(void);
void StartDefaultTask(void *argument);
void StartI2CTask(void *argument);
void StartSPITask(void *argument);
void StartCANTask(void *argument);
void StartUARTTask(void *argument);

/* USER CODE BEGIN PFP */
void WorkI2C(void);
void WorkSPI(void);
void WorkCAN(void);
void WorkUART(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
//  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of CommMutexHandle */
  CommMutexHandleHandle = osMutexNew(&CommMutexHandle_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of CanQueue */
  CanQueueHandle = osMessageQueueNew (8, 8, &CanQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of I2CTask */
  I2CTaskHandle = osThreadNew(StartI2CTask, NULL, &I2CTask_attributes);

  /* creation of SPITask */
  SPITaskHandle = osThreadNew(StartSPITask, NULL, &SPITask_attributes);

  /* creation of CANTask */
  CANTaskHandle = osThreadNew(StartCANTask, NULL, &CANTask_attributes);

  /* creation of UARTTask */
  UARTTaskHandle = osThreadNew(StartUARTTask, NULL, &UARTTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
//  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  WorkI2C();
	  WorkSPI();
	  WorkCAN();
	  WorkUART();
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  HAL_CAN_Start(&hcan1);

  HAL_CAN_ActivateNotification(
      &hcan1,
      CAN_IT_RX_FIFO0_MSG_PENDING
  );
  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_SET);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        // 0x27 읽기가 끝남
        if (i2c_step == 1)
        {
            if (HAL_I2C_Mem_Read_DMA(&hi2c1, SLAVE_ADDR << 1, PMIC_STATUS2, I2C_MEMADD_SIZE_8BIT, &status2.value, 1) == HAL_OK)
                i2c_step = 2;
        }

        // 0x28 읽기까지 끝남
        else if (i2c_step == 2)
        {
        	if (status1.bit.PGBUCK2 == 0 || status2.bit.otwarning == 1 || status2.bit.otempp == 1)
			{
        		brake_dtc.dtc[0] = 0x12;
				brake_dtc.dtc[1] = 0x34;
				brake_dtc.dtc[2] = 0x56;

        		    brake_dtc.status.value = 0x26;

				dtc_save_request = 1;
			}

        	i2c_step = 0;
        }
    }
}


/* 테스트용 콜백 */
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        pmic_voltage_write_busy = 0;
        pmic_voltage_change_request = 0;

        uint8_t msg[] = "PMIC Buck4 Voltage Set Complete\r\n";

		HAL_UART_Transmit(
			&huart4,
			msg,
			sizeof(msg) - 1,
			HAL_MAX_DELAY
		);
    }

}

void WorkI2C(void)
{
	/* PMIC 전압 변경 요청 */
	    if (pmic_voltage_change_request == 1 &&
	        pmic_voltage_write_busy == 0 &&
	        i2c_step == 0)
	    {
	        if (HAL_I2C_Mem_Write_DMA(
	                &hi2c1,
	                SLAVE_ADDR << 1,
	                PMIC_BUCK4_VREF_REG,
	                I2C_MEMADD_SIZE_8BIT,
	                &buck4_vref_data,
	                1) == HAL_OK)
	        {
	            pmic_voltage_write_busy = 1;
	        }

	        return;
	    }


	/* 기존 */
    if (i2c_step != 0)
        return;

    if (HAL_I2C_Mem_Read_DMA(&hi2c1, SLAVE_ADDR << 1, PMIC_STATUS1, I2C_MEMADD_SIZE_8BIT, &status1.value, 1) == HAL_OK) i2c_step = 1;


}



void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	 if (hspi->Instance == SPI1)
	{
		 /* EEPROM 저장 Mock */
		eeprom_mock[0] = spiTxData[0];
		eeprom_mock[1] = spiTxData[1];
		eeprom_mock[2] = spiTxData[2];
		eeprom_mock[3] = spiTxData[3];

		/* EEPROM Read-back Mock */
		eeprom_read_data[0] = eeprom_mock[0];
		eeprom_read_data[1] = eeprom_mock[1];
		eeprom_read_data[2] = eeprom_mock[2];
		eeprom_read_data[3] = eeprom_mock[3];

		  /* 저장값 검증 */
		if ((eeprom_read_data[0] == spiTxData[0]) && (eeprom_read_data[1] == spiTxData[1])
				&& (eeprom_read_data[2] == spiTxData[2]) && (eeprom_read_data[3] == spiTxData[3]))
		{
			dtc_read_ready = 1;
			eeprom_error = 0;
		}

		 else
		{
			dtc_read_ready = 0;
			eeprom_error = 1;
		}

		spi_busy = 0;
		dtc_save_request = 0;
	}
}

void WorkSPI(void)
{
	if (dtc_save_request == 0)	return;

	if (spi_busy == 1)	return;

	spiTxData[0] = brake_dtc.dtc[0];
	spiTxData[1] = brake_dtc.dtc[1];
	spiTxData[2] = brake_dtc.dtc[2];
	spiTxData[3] = brake_dtc.status.value;

	if (HAL_SPI_Transmit_DMA(&hspi1, spiTxData, 4) == HAL_OK) spi_busy = 1;

}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1)
    {
        HAL_CAN_GetRxMessage(
            hcan,
            CAN_RX_FIFO0,
            &RxHeader,
            RxData
        );

        can_rx_flag = 1;
    }
}

void WorkCAN(void){
	if (can_rx_flag == 0)	return;

	can_rx_flag = 0;

	 /* UDS ReadDTCInformation */
	if (RxData[0] == 0x19 && RxData[1] == 0x08 && dtc_read_ready == 1)
	{
		TxHeader.StdId = 0x7E8;
		TxHeader.IDE = CAN_ID_STD;
		TxHeader.RTR = CAN_RTR_DATA;
		TxHeader.DLC = 7;

		TxData[0] = 0x59;                   // Positive Response
		TxData[1] = 0x0B;                   // reportFirstTestFailedDTC
		TxData[2] = 0xFF;                   // DTCStatusAvailabilityMask

		TxData[3] = eeprom_read_data[0];
		TxData[4] = eeprom_read_data[1];
		TxData[5] = eeprom_read_data[2];
		TxData[6] = eeprom_read_data[3];

		HAL_CAN_AddTxMessage(
			&hcan1,
			&TxHeader,
			TxData,
			&TxMailbox
		);
	}

	 /* UDS ClearDiagnosticInformation */
	else if (RxData[0] == 0x14)
	{
		/* DTC Clear */
		brake_dtc.dtc[0] = 0x00;
		brake_dtc.dtc[1] = 0x00;
		brake_dtc.dtc[2] = 0x00;
		brake_dtc.status.value = 0x00;

		/* EEPROM Mock Clear */
		eeprom_mock[0] = 0x00;
		eeprom_mock[1] = 0x00;
		eeprom_mock[2] = 0x00;
		eeprom_mock[3] = 0x00;

		eeprom_read_data[0] = 0x00;
		eeprom_read_data[1] = 0x00;
		eeprom_read_data[2] = 0x00;
		eeprom_read_data[3] = 0x00;

		dtc_read_ready = 0;
		eeprom_error = 0;

		/* Positive Response */
		TxHeader.StdId = 0x7E8;
		TxHeader.IDE = CAN_ID_STD;
		TxHeader.RTR = CAN_RTR_DATA;
		TxHeader.DLC = 1;

		TxData[0] = 0x54;

		HAL_CAN_AddTxMessage(
			&hcan1,
			&TxHeader,
			TxData,
			&TxMailbox
		);
	}
}

void WorkUART(void){
	static uint32_t last_tick = 0;

	if (HAL_GetTick() - last_tick >= 1000)
	{
		if (eeprom_error == 1)
		{
			uint8_t msg[] = "EEPROM Error\r\n";

			HAL_UART_Transmit(
				&huart4,
				msg,
				sizeof(msg) - 1,
				HAL_MAX_DELAY
			);
		}

		else if (brake_dtc.status.value != 0x00)
		{
			uint8_t msg[] = "System Fault\r\n";

			HAL_UART_Transmit(
				&huart4,
				msg,
				sizeof(msg) - 1,
				HAL_MAX_DELAY
			);
		}

		else
		{
			uint8_t msg[] = "System Normal\r\n";

			HAL_UART_Transmit(
				&huart4,
				msg,
				sizeof(msg) - 1,
				HAL_MAX_DELAY
			);
		}

		last_tick = HAL_GetTick();
	}

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
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartI2CTask */
/**
* @brief Function implementing the I2CTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartI2CTask */
void StartI2CTask(void *argument)
{
  /* USER CODE BEGIN StartI2CTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartI2CTask */
}

/* USER CODE BEGIN Header_StartSPITask */
/**
* @brief Function implementing the SPITask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSPITask */
void StartSPITask(void *argument)
{
  /* USER CODE BEGIN StartSPITask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartSPITask */
}

/* USER CODE BEGIN Header_StartCANTask */
/**
* @brief Function implementing the CANTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCANTask */
void StartCANTask(void *argument)
{
  /* USER CODE BEGIN StartCANTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartCANTask */
}

/* USER CODE BEGIN Header_StartUARTTask */
/**
* @brief Function implementing the UARTTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUARTTask */
void StartUARTTask(void *argument)
{
  /* USER CODE BEGIN StartUARTTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartUARTTask */
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
