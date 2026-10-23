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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
    uint8_t stable;       // debounced state: 1 = pressed
    uint8_t last_raw;     // last raw reading
    uint32_t last_change; // tick of the last raw change
} Button_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAN_ID_BCM_CMD        0x100
#define CAN_ID_DOOR_STATUS    0x200
#define TX_PERIOD_MS          100
#define NODEB_TIMEOUT_MS      500
#define BLINK_HALF_PERIOD_MS  333

#define MCP_MODE_NORMAL       0x00
#define MCP_MODE_LOOPBACK     0x40
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MCP_Select()   HAL_GPIO_WritePin(MCP_CS_GPIO_Port, MCP_CS_Pin, GPIO_PIN_RESET)
#define MCP_Deselect() HAL_GPIO_WritePin(MCP_CS_GPIO_Port, MCP_CS_Pin, GPIO_PIN_SET)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
Button_t btn1 = { BTN1_GPIO_Port, BTN1_Pin, 0, 0, 0 };   // ignition (blue button)
Button_t btn2 = { BTN2_GPIO_Port, BTN2_Pin, 0, 0, 0 };   // headlight
Button_t btn3 = { BTN3_GPIO_Port, BTN3_Pin, 0, 0, 0 };   // left turn
Button_t btn4 = { BTN4_GPIO_Port, BTN4_Pin, 0, 0, 0 };   // right turn

uint8_t ignition = 0, headlight = 0, left_active = 0, right_active = 0;
uint8_t buzzer_on = 0, lock_cmd = 0, alive_counter = 0, bus_ok = 1;

uint8_t door_open = 0, nodeb_lock = 0, nodeb_window = 0, nodeb_alive = 0;

uint32_t last_blink_tick = 0, last_buzzer_tick = 0, last_bcm_tx = 0;
uint32_t last_nodeb_rx = 0, last_fault_tick = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

static uint8_t Button_Pressed(Button_t *b)
{
    uint8_t raw = (HAL_GPIO_ReadPin(b->port, b->pin) == GPIO_PIN_RESET);
    if (raw != b->last_raw) {
        b->last_raw = raw;
        b->last_change = HAL_GetTick();
    }
    if ((HAL_GetTick() - b->last_change) >= 30 && raw != b->stable) {
        b->stable = raw;
        if (raw) return 1;
    }
    return 0;
}

static void Print_Status(void)
{
    printf("[STATUS] Ign=%s Head=%s L=%s R=%s | NodeB=%s Door=%s Lock=%s Win=%u\r\n",
           ignition ? "ON" : "OFF", headlight ? "ON" : "OFF",
           left_active ? "ON" : "OFF", right_active ? "ON" : "OFF",
           nodeb_alive ? "OK" : "LOST",
           door_open ? "OPEN" : "CLOSED",
           nodeb_lock ? "LOCKED" : "UNLOCKED",
           nodeb_window);
}

static void MCP_Reset(void)
{
    uint8_t cmd = 0xC0;
    MCP_Select();
    HAL_SPI_Transmit(&hspi2, &cmd, 1, 10);
    MCP_Deselect();
    HAL_Delay(10);
}

static uint8_t MCP_ReadReg(uint8_t reg)
{
    uint8_t cmd[2] = { 0x03, reg };
    uint8_t val = 0;
    MCP_Select();
    HAL_SPI_Transmit(&hspi2, cmd, 2, 10);
    HAL_SPI_Receive(&hspi2, &val, 1, 10);
    MCP_Deselect();
    return val;
}

static void MCP_WriteReg(uint8_t reg, uint8_t val)
{
    uint8_t tx[3] = { 0x02, reg, val };
    MCP_Select();
    HAL_SPI_Transmit(&hspi2, tx, 3, 10);
    MCP_Deselect();
}

// 500 kbit/s with an 8 MHz module crystal. Returns 1 on success.
static uint8_t MCP_Init(uint8_t mode)
{
    MCP_Reset();
    if ((MCP_ReadReg(0x0E) & 0xE0) != 0x80) return 0;   // must be in config mode

    MCP_WriteReg(0x2A, 0x00);   // CNF1
    MCP_WriteReg(0x29, 0x90);   // CNF2
    MCP_WriteReg(0x28, 0x02);   // CNF3
    MCP_WriteReg(0x60, 0x60);   // RXB0CTRL: filters off, accept all
    MCP_WriteReg(0x2B, 0x01);   // CANINTE: INT low when a frame arrives
    MCP_WriteReg(0x0F, mode);   // request the mode
    HAL_Delay(5);

    return ((MCP_ReadReg(0x0E) & 0xE0) == (mode & 0xE0));
}

static uint8_t MCP_Send(uint16_t id, const uint8_t *data, uint8_t len)
{
    if (MCP_ReadReg(0x30) & 0x08) return 0;   // TXB0 still pending

    uint8_t buf[15];
    buf[0] = 0x02;                       // WRITE
    buf[1] = 0x31;                       // TXB0SIDH
    buf[2] = (uint8_t)(id >> 3);
    buf[3] = (uint8_t)((id & 0x07) << 5);
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = len;
    for (uint8_t i = 0; i < len; i++) buf[7 + i] = data[i];

    MCP_Select();
    HAL_SPI_Transmit(&hspi2, buf, 7 + len, 10);
    MCP_Deselect();

    uint8_t rts = 0x81;                  // request to send, buffer 0
    MCP_Select();
    HAL_SPI_Transmit(&hspi2, &rts, 1, 10);
    MCP_Deselect();
    return 1;
}

static void MCP_Receive(uint16_t *id, uint8_t *dlc, uint8_t *data)
{
    uint8_t cmd = 0x90;                  // READ RX BUFFER 0 (clears RX0IF)
    uint8_t rx[14] = { 0 };

    MCP_Select();
    HAL_SPI_Transmit(&hspi2, &cmd, 1, 10);
    HAL_SPI_Receive(&hspi2, rx, 14, 10);
    MCP_Deselect();

    *id  = ((uint16_t)rx[0] << 3) | (rx[1] >> 5);
    *dlc = rx[4] & 0x0F;
    for (uint8_t i = 0; i < 8; i++) data[i] = rx[5 + i];
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
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  printf("\r\n==== Node A (BCM) started ====\r\n");
  if (MCP_Init(MCP_MODE_NORMAL))
      printf("[INIT] MCP2515 OK (normal mode)\r\n");
  else
      printf("[INIT] MCP2515 FAILED, CANSTAT=0x%02X\r\n", MCP_ReadReg(0x0E));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	    uint32_t now = HAL_GetTick();

	    /* 1. Buttons */
	    if (Button_Pressed(&btn1)) {
	        ignition = !ignition;
	        if (!ignition) {
	            left_active = 0;
	            right_active = 0;
	            HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
	            HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
	        }
	        printf("Ignition: %s\r\n", ignition ? "ON" : "OFF");
	        Print_Status();
	    }
	    if (Button_Pressed(&btn2)) {
	        headlight = !headlight;
	        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, headlight ? GPIO_PIN_SET : GPIO_PIN_RESET);
	        printf("Headlight: %s\r\n", headlight ? "ON" : "OFF");
	        Print_Status();
	    }
	    if (Button_Pressed(&btn3)) {
	        if (!ignition) {
	            printf("Left indicator ignored: ignition OFF\r\n");
	        } else {
	            left_active = !left_active;
	            if (!left_active) HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
	            printf("Left indicator: %s\r\n", left_active ? "ON" : "OFF");
	            Print_Status();
	        }
	    }
	    if (Button_Pressed(&btn4)) {
	        if (!ignition) {
	            printf("Right indicator ignored: ignition OFF\r\n");
	        } else {
	            right_active = !right_active;
	            if (!right_active) HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
	            printf("Right indicator: %s\r\n", right_active ? "ON" : "OFF");
	            Print_Status();
	        }
	    }

	    /* 2. Auto-lock decision */
	    if (!ignition)                         lock_cmd = 0;   // ignition off: unlock
	    else if (nodeb_alive && !door_open)    lock_cmd = 1;   // ignition on + door closed: lock

	    /* 3. Buzzer: ignition ON + door OPEN (passive buzzer, 500 Hz tone) */
	    buzzer_on = (ignition && nodeb_alive && door_open);
	    if (buzzer_on) {
	        if (now != last_buzzer_tick) {
	            last_buzzer_tick = now;
	            HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);
	        }
	    } else {
	        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
	    }

	    /* 4. Send BCM_Cmd (0x100) every 100 ms */
	    if ((now - last_bcm_tx) >= TX_PERIOD_MS) {
	        last_bcm_tx = now;
	        uint8_t d[3] = { ignition, lock_cmd, alive_counter++ };
	        if (MCP_Send(CAN_ID_BCM_CMD, d, 3)) {
	            if (!bus_ok) { bus_ok = 1; printf("[CAN] TX OK\r\n"); }
	        } else if (bus_ok) {
	            bus_ok = 0;
	            printf("[CAN] TX not acknowledged - check Node B and bus wiring\r\n");
	        }
	    }

	    /* 5. Receive Door_Status (0x200) when INT goes low */
	    if (HAL_GPIO_ReadPin(MCP_INT_GPIO_Port, MCP_INT_Pin) == GPIO_PIN_RESET) {
	        uint16_t rid; uint8_t rdlc; uint8_t rd[8];
	        MCP_Receive(&rid, &rdlc, rd);
	        if (rid == CAN_ID_DOOR_STATUS && rdlc >= 4) {
	            last_nodeb_rx = now;
	            uint8_t changed = (!nodeb_alive || rd[0] != door_open ||
	                               rd[1] != nodeb_lock || rd[2] != nodeb_window);
	            if (!nodeb_alive) { nodeb_alive = 1; printf("[CAN] Node B connected\r\n"); }
	            door_open    = rd[0];
	            nodeb_lock   = rd[1];
	            nodeb_window = rd[2];
	            if (changed) Print_Status();
	        }
	    }

	    /* 6. Node B timeout (500 ms) */
	    if (nodeb_alive && ((now - last_nodeb_rx) > NODEB_TIMEOUT_MS)) {
	        nodeb_alive = 0;
	        printf("[WARN] Node B lost (no status for 500 ms)\r\n");
	        Print_Status();
	    }

	    /* 7. Fault LED: LD2 (HIGH = on) blinks fast while Node B is lost */
	    if (!nodeb_alive) {
	        if ((now - last_fault_tick) >= 100) {
	            last_fault_tick = now;
	            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	        }
	    } else {
	        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);   // off
	    }

	    /* 8. Turn indicators blink at about 1.5 Hz (non-blocking) */
	    if ((now - last_blink_tick) >= BLINK_HALF_PERIOD_MS) {
	        last_blink_tick = now;
	        if (left_active)  HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
	        if (right_active) HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  huart2.Init.BaudRate = 115200;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|LED1_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MCP_CS_GPIO_Port, MCP_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BTN1_Pin */
  GPIO_InitStruct.Pin = BTN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTN1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN2_Pin BTN3_Pin */
  GPIO_InitStruct.Pin = BTN2_Pin|BTN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MCP_INT_Pin BTN4_Pin */
  GPIO_InitStruct.Pin = MCP_INT_Pin|BTN4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : MCP_CS_Pin LED3_Pin */
  GPIO_InitStruct.Pin = MCP_CS_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BUZZER_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

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
