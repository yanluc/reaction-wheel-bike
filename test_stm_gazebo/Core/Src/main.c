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
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* Drive modes, cycled with the blue user button B1. */
typedef enum
{
  MODE_STOP = 0,
  MODE_FORWARD,
  MODE_CIRCLE,
  MODE_RETURN_HOME, /* closed loop: drives back to (0,0) using odometry from Gazebo */
  MODE_COUNT
} DriveMode;

typedef struct
{
  int32_t x_mm;
  int32_t y_mm;
  int16_t yaw_mrad;
  int16_t v_mm_s;
} Odom;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*
 * Frame (both directions): 0xAA 0x55 | type | len | payload[len] | crc8
 * crc8: poly 0x07, init 0x00, over type, len and payload. Little-endian payload.
 *   MSG_CMD  (MCU -> host, len 6):  int16 vx [mm/s], int16 wz [mrad/s], uint8 mode, uint8 seq
 *   MSG_ODOM (host -> MCU, len 12): int32 x [mm], int32 y [mm], int16 yaw [mrad], int16 v [mm/s]
 */
#define SYNC1           0xAAu
#define SYNC2           0x55u
#define MSG_CMD         0x01u
#define MSG_ODOM        0x02u
#define MAX_PAYLOAD     16u

#define RX_BUF_SIZE     256u
#define CONTROL_PERIOD_MS 20u   /* 50 Hz */
#define LINK_TIMEOUT_MS 500u
#define MOVING_MM_S     50
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef hlpuart1;
DMA_HandleTypeDef hdma_lpuart_rx;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
static uint8_t rx_buf[RX_BUF_SIZE];   /* DMA circular RX buffer */
static uint16_t rx_tail;

static DriveMode mode = MODE_STOP;
static Odom odom;
static uint32_t last_odom_tick;
static uint8_t has_odom;
static uint8_t tx_seq;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
/* USER CODE BEGIN PFP */
static uint8_t crc8(const uint8_t *data, uint16_t len);
static void send_cmd(int16_t vx_mm_s, int16_t wz_mrad_s);
static void poll_rx(void);
static void compute_cmd(int16_t *vx_mm_s, int16_t *wz_mrad_s);
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
  MX_LPUART1_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */
  /* RX runs forever in circular DMA; the loop polls the DMA counter, so no UART IRQ is needed.
     Overrun detection is disabled in the .ioc, so a burst can never stall reception. */
  if (HAL_UART_Receive_DMA(&hlpuart1, rx_buf, RX_BUF_SIZE) != HAL_OK)
  {
    Error_Handler();
  }

  uint32_t next_tick = HAL_GetTick();
  GPIO_PinState btn_prev = GPIO_PIN_RESET;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    poll_rx();

    uint32_t now = HAL_GetTick();
    if ((int32_t)(now - next_tick) < 0)
    {
      continue;
    }
    next_tick += CONTROL_PERIOD_MS;

    /* B1 is active high on Nucleo-144; sampling at 50 Hz debounces it. */
    GPIO_PinState btn = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);
    if (btn == GPIO_PIN_SET && btn_prev == GPIO_PIN_RESET)
    {
      mode = (DriveMode)((mode + 1) % MODE_COUNT);
    }
    btn_prev = btn;

    int16_t vx, wz;
    compute_cmd(&vx, &wz);
    send_cmd(vx, wz);

    uint8_t link_ok = has_odom && (now - last_odom_tick) < LINK_TIMEOUT_MS;
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, link_ok ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,
                      (link_ok && abs(odom.v_mm_s) > MOVING_MM_S) ? GPIO_PIN_SET : GPIO_PIN_RESET);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 2000000;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
  hlpuart1.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel7_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static uint8_t crc8(const uint8_t *data, uint16_t len)
{
  uint8_t crc = 0;
  while (len--)
  {
    crc ^= *data++;
    for (uint8_t i = 0; i < 8; i++)
    {
      crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x07u) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

static void send_cmd(int16_t vx_mm_s, int16_t wz_mrad_s)
{
  uint8_t f[2 + 2 + 6 + 1];
  f[0] = SYNC1;
  f[1] = SYNC2;
  f[2] = MSG_CMD;
  f[3] = 6;
  memcpy(&f[4], &vx_mm_s, 2);   /* Cortex-M4 is little-endian */
  memcpy(&f[6], &wz_mrad_s, 2);
  f[8] = (uint8_t)mode;
  f[9] = tx_seq++;
  f[10] = crc8(&f[2], 2 + 6);
  HAL_UART_Transmit(&hlpuart1, f, sizeof(f), 5);
}

static void handle_frame(uint8_t type, const uint8_t *p, uint8_t len)
{
  if (type == MSG_ODOM && len == 12)
  {
    memcpy(&odom.x_mm, &p[0], 4);
    memcpy(&odom.y_mm, &p[4], 4);
    memcpy(&odom.yaw_mrad, &p[8], 2);
    memcpy(&odom.v_mm_s, &p[10], 2);
    last_odom_tick = HAL_GetTick();
    has_odom = 1;
  }
}

/* Byte-wise parser fed from the DMA ring buffer. */
static void poll_rx(void)
{
  static enum { S_SYNC1, S_SYNC2, S_TYPE, S_LEN, S_DATA, S_CRC } st = S_SYNC1;
  static uint8_t hdr[2 + MAX_PAYLOAD]; /* type, len, payload */
  static uint8_t idx;

  uint16_t head = RX_BUF_SIZE - (uint16_t)__HAL_DMA_GET_COUNTER(hlpuart1.hdmarx);
  if (head >= RX_BUF_SIZE)
  {
    head = 0;
  }

  while (rx_tail != head)
  {
    uint8_t b = rx_buf[rx_tail];
    rx_tail = (rx_tail + 1) % RX_BUF_SIZE;

    switch (st)
    {
      case S_SYNC1: st = (b == SYNC1) ? S_SYNC2 : S_SYNC1; break;
      case S_SYNC2: st = (b == SYNC2) ? S_TYPE : ((b == SYNC1) ? S_SYNC2 : S_SYNC1); break;
      case S_TYPE:  hdr[0] = b; st = S_LEN; break;
      case S_LEN:
        hdr[1] = b;
        idx = 0;
        st = (b > MAX_PAYLOAD) ? S_SYNC1 : ((b == 0) ? S_CRC : S_DATA);
        break;
      case S_DATA:
        hdr[2 + idx++] = b;
        if (idx >= hdr[1])
        {
          st = S_CRC;
        }
        break;
      case S_CRC:
        if (crc8(hdr, (uint16_t)(2 + hdr[1])) == b)
        {
          handle_frame(hdr[0], &hdr[2], hdr[1]);
        }
        st = S_SYNC1;
        break;
    }
  }
}

static int16_t clamp16(float v, float lim)
{
  if (v > lim) v = lim;
  if (v < -lim) v = -lim;
  return (int16_t)v;
}

static void compute_cmd(int16_t *vx_mm_s, int16_t *wz_mrad_s)
{
  *vx_mm_s = 0;
  *wz_mrad_s = 0;

  switch (mode)
  {
    case MODE_FORWARD:
      *vx_mm_s = 800;
      break;
    case MODE_CIRCLE:
      *vx_mm_s = 600;
      *wz_mrad_s = 500;
      break;
    case MODE_RETURN_HOME:
    {
      /* P-controller to the odometry origin (robot spawn pose). Needs live odometry. */
      if (!has_odom || (HAL_GetTick() - last_odom_tick) >= LINK_TIMEOUT_MS)
      {
        break;
      }
      float dx = -(float)odom.x_mm;
      float dy = -(float)odom.y_mm;
      float dist = sqrtf(dx * dx + dy * dy);
      if (dist < 150.0f)
      {
        break;
      }
      float err = atan2f(dy, dx) - (float)odom.yaw_mrad / 1000.0f;
      err = atan2f(sinf(err), cosf(err)); /* wrap to [-pi, pi] */
      *wz_mrad_s = clamp16(2000.0f * err, 1500.0f);
      /* Turn in place first, then drive. */
      *vx_mm_s = (fabsf(err) > 0.5f) ? 0 : clamp16(0.8f * dist, 800.0f);
      break;
    }
    default:
      break;
  }
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
