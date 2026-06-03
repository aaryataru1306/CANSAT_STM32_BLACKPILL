/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LORA_NSS_PORT GPIOA
#define LORA_NSS_PIN  GPIO_PIN_4

#define LORA_RST_PORT GPIOB
#define LORA_RST_PIN  GPIO_PIN_0

#define LORA_DIO0_PORT GPIOB
#define LORA_DIO0_PIN  GPIO_PIN_1
extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
uint8_t rxBuffer[128];
uint32_t packets = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void usb_print(char *msg)
{
    while(CDC_Transmit_FS((uint8_t*)msg, strlen(msg)) == USBD_BUSY)
        HAL_Delay(1);
}

void lora_select()
{
    HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_RESET);
}

void lora_unselect()
{
    HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_SET);
}

void lora_write(uint8_t reg, uint8_t data)
{
    uint8_t buf[2];
    buf[0] = reg | 0x80;
    buf[1] = data;

    lora_select();
    HAL_SPI_Transmit(&hspi1, buf, 2, 100);
    lora_unselect();
}

uint8_t lora_read(uint8_t reg)
{
    uint8_t tx[2], rx[2];

    tx[0] = reg & 0x7F;
    tx[1] = 0x00;

    lora_select();
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, 100);
    lora_unselect();

    return rx[1];
}

void lora_reset()
{
    HAL_GPIO_WritePin(LORA_RST_PORT, LORA_RST_PIN, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(LORA_RST_PORT, LORA_RST_PIN, GPIO_PIN_SET);
    HAL_Delay(10);
}

void lora_init()
{
    lora_reset();

    lora_write(0x01, 0x80);   // sleep LoRa
    HAL_Delay(10);

    lora_write(0x01, 0x81);   // standby

    /* 433 MHz */
    lora_write(0x06, 0x6C);
    lora_write(0x07, 0x80);
    lora_write(0x08, 0x00);

    lora_write(0x0C, 0x23);   // LNA boost

    lora_write(0x0E, 0x00);
    lora_write(0x0F, 0x00);

    lora_write(0x1D, 0x72);   // BW125 CR4/5
    lora_write(0x1E, 0x74);   // SF7 CRC ON
    lora_write(0x26, 0x04);   // AGC ON

    lora_write(0x39, 0x34);   // sync word

    lora_write(0x40, 0x00);   // DIO0 = RxDone

    lora_write(0x12, 0xFF);   // clear IRQ

    lora_write(0x01, 0x85);   // continuous RX mode
}

void lora_receive()
{
    uint8_t irq = lora_read(0x12);

    if((irq & 0x40) && !(irq & 0x20))   // RxDone & no CRC error
    {
        lora_write(0x12, 0xFF);

        uint8_t len  = lora_read(0x13);
        uint8_t addr = lora_read(0x10);

        if(len > sizeof(rxBuffer))
        {
            usb_print("PACKET TOO LARGE\r\n");
            return;
        }

        lora_write(0x0D, addr);

        for(uint8_t i = 0; i < len; i++)
        {
            rxBuffer[i] = lora_read(0x00);
        }

        int rssi = lora_read(0x1A) - 157;

        uint8_t p = 0;

        char team[16]  = {0};
        char state[16] = {0};

        /* ================= TEAM ================= */
        uint8_t team_len = rxBuffer[p++];

        if(team_len > 15)
            return;

        memcpy(team, &rxBuffer[p], team_len);
        team[team_len] = '\0';
        p += team_len;

        /* ================= TIME ================= */
        int32_t time;
        memcpy(&time, &rxBuffer[p], 4);
        p += 4;

        /* ================= PACKET COUNT ================= */
        int32_t packetCount;
        memcpy(&packetCount, &rxBuffer[p], 4);
        p += 4;

        /* ================= ALTITUDE ================= */
        float altitude;
        memcpy(&altitude, &rxBuffer[p], 4);
        p += 4;

        /* ================= PRESSURE ================= */
        float pressure;
        memcpy(&pressure, &rxBuffer[p], 4);
        p += 4;

        /* ================= TEMPERATURE ================= */
        float temperature;
        memcpy(&temperature, &rxBuffer[p], 4);
        p += 4;

        /* ================= VOLTAGE ================= */
        float voltage;
        memcpy(&voltage, &rxBuffer[p], 4);
        p += 4;

        /* ================= GPS ================= */
        float latitude;
        memcpy(&latitude, &rxBuffer[p], 4);
        p += 4;

        float longitude;
        memcpy(&longitude, &rxBuffer[p], 4);
        p += 4;

        float gps_altitude;
        memcpy(&gps_altitude, &rxBuffer[p], 4);
        p += 4;

        /* ================= SATELLITES ================= */
        int32_t satellites;
        memcpy(&satellites, &rxBuffer[p], 4);
        p += 4;

        /* ================= ACCEL ================= */
        float ax, ay, az;

        memcpy(&ax, &rxBuffer[p], 4);
        p += 4;

        memcpy(&ay, &rxBuffer[p], 4);
        p += 4;

        memcpy(&az, &rxBuffer[p], 4);
        p += 4;

        /* ================= GYRO ================= */
        float gyro;
        memcpy(&gyro, &rxBuffer[p], 4);
        p += 4;

        /* ================= STATE ================= */
        uint8_t state_len = rxBuffer[p++];

        if(state_len > 15)
            return;

        memcpy(state, &rxBuffer[p], state_len);
        state[state_len] = '\0';
        p += state_len;

        /* ================= PRINT ================= */
        char msg[400];

        sprintf(msg,
        "TEAM:%s | "
        "TIME:%ld | "
        "PKT:%ld | "
        "ALT:%.2fm | "
        "P:%.2fhPa | "
        "T:%.2fC | "
        "V:%.2fV | "
        "LAT:%.5f | "
        "LON:%.5f | "
        "GPS_ALT:%.2fm | "
        "SAT:%ld | "
        "AX:%.2f AY:%.2f AZ:%.2f | "
        "GYRO:%.2f | "
        "STATE:%s | "
        "RSSI:%d dBm\r\n",

        team,
        time,
        packetCount,
        altitude,
        pressure,
        temperature,
        voltage,
        latitude,
        longitude,
        gps_altitude,
        satellites,
        ax, ay, az,
        gyro,
        state,
        rssi);

        usb_print(msg);

        packets++;
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
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(1000);

  usb_print("LoRa Receiver Start\r\n");

  lora_init();

  uint8_t ver = lora_read(0x42);
  char msg[50];
  sprintf(msg,"SX1278 Version: 0x%02X\r\n",ver);
  usb_print(msg);

  usb_print("Waiting Packets...\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  lora_receive();
	  HAL_Delay(10);
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_1 */
  // Set initial states BEFORE config
  HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_SET);   // NSS HIGH (inactive)
  HAL_GPIO_WritePin(LORA_RST_PORT, LORA_RST_PIN, GPIO_PIN_SET);   // RST HIGH
  /* USER CODE END MX_GPIO_Init_1 */

  /*Configure GPIO pin : NSS_Pin → PA4 */
  GPIO_InitStruct.Pin = LORA_NSS_PIN;  // PA4
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LORA_NSS_PORT, &GPIO_InitStruct);  // GPIOA

  /*Configure GPIO pin : RST_Pin → PB0 */
  GPIO_InitStruct.Pin = LORA_RST_PIN;  // PB0
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LORA_RST_PORT, &GPIO_InitStruct);  // GPIOB

  /*Configure GPIO pin : DIO0_Pin → PB1 */
  GPIO_InitStruct.Pin = LORA_DIO0_PIN;  // PB1
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);  // GPIOB

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_SET);  // Ensure NSS idle HIGH
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
