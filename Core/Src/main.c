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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <string.h>

#include "board_config.h"
#include "mcp_diag.h"
#include "mcp_transport.h"
#include "mcp_protocol.h"
#include "mcp_tools.h"
#include "mcp_registration.h"
#include "retarget_io.h"
#include "led.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* Receive buffer. One byte larger than the protocol's capacity so the
 * NUL-terminator MCP_Rx_Start() appends always has a home. */
static uint8_t  mcp_rx_buf[MCP_RX_BUF_LEN + 1U];
static mcp_rx_t mcp_rx;

static uint32_t mcp_last_poll_tick     = 0U;
static uint32_t mcp_last_register_tick = 0U;
static uint32_t mcp_last_status_tick   = 0U;

/* One transaction, no chunking -- see mcp_transport.h for why that is a
 * correctness requirement and not a shortcut. */
static const char MCP_ACK_JSON[] = "{\"command\":\"action_finish\",\"params\":\"true\"}";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  MCP_RetargetInit(&huart1);
  MCP_DiagReset();
  MCP_Rx_Init(&mcp_rx);
  mcp_i2c_speed_hz = MCP_I2C_SPEED_READ;   /* matches MX_I2C1_Init: 100 kHz */

  /* Back-date the registration timer so the retry condition is already
   * satisfied on the very first poll iteration. The module only accepts
   * registration during its own startup window, so we must start writing as
   * early as possible rather than waiting out a full retry period first. */
  mcp_last_register_tick = HAL_GetTick() - MCP_REG_RETRY_MS;

  MCP_LedInit();

  printf("\r\n\r\n=== WonderLLM MCP bring-up (STM32 as I2C master) ===\r\n");
  printf("MCU STM32F103RCT6 @ %lu Hz\r\n", (unsigned long)SystemCoreClock);
  printf("I2C slave 0x%02X (HAL write 0x%02X / read 0x%02X)\r\n",
         (unsigned)MCP_ADDR_7BIT,
         (unsigned)MCP_ADDR_HAL_W,
         (unsigned)MCP_ADDR_HAL_R);
  printf("\r\nSTARTUP ORDER THAT MATTERS:\r\n");
  printf("  The module only accepts MCP tool registration during its own\r\n");
  printf("  startup window. This firmware retries registration every\r\n");
  printf("  %u ms until frames start arriving.\r\n", (unsigned)MCP_REG_RETRY_MS);
  printf("  => POWER-CYCLE THE ESP32-S3 MODULE NOW, while this runs.\r\n\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now = HAL_GetTick();
    uint16_t got = 0U;

    /* Drain the log buffer. Bounded per pass so a backlog is spread across
     * loop iterations instead of monopolising one -- the console must never
     * become the reason the bus is late. */
    MCP_RetargetService(32U);

    if ((now - mcp_last_poll_tick) >= MCP_POLL_INTERVAL_MS)
    {
      mcp_last_poll_tick = now;

      /* ---- Registration retry -------------------------------------------
       * Re-send the tool registration periodically until a frame actually
       * carries JSON, i.e. until the model has been given its tools and has
       * responded.
       *
       * This deliberately does NOT stop at the first frame received. Stopping
       * at any frame meant the module's first 1-byte power-on notice
       * permanently disabled registration, and since the module only honours
       * registration during its startup window, a later module reboot could
       * then never be registered at all -- which is why talking to the model
       * produced nothing on the bus.
       * Re-sending costs one short write when already registered. -------- */
      if ((mcp_json_frames == 0U) && ((now - mcp_last_register_tick) >= MCP_REG_RETRY_MS))
      {
        mcp_last_register_tick = now;
        mcp_register_attempts++;

        if (MCP_Probe(&hi2c1) == HAL_OK)
        {
          mcp_probe_ok_count++;
          MCP_LedNoteProbeOk();

          if (MCP_ToolsRegister(&hi2c1) == HAL_OK)
          {
            mcp_register_ok++;
            mcp_state = (uint32_t)MCP_STATE_SILENT;
            printf("[%lu ms] registration #%lu sent (0x%02X ACK, %lu bytes total)\r\n",
                   (unsigned long)now, (unsigned long)mcp_register_ok,
                   (unsigned)MCP_ADDR_7BIT, (unsigned long)mcp_tx_total_bytes);
          }
          else
          {
            mcp_last_error = HAL_I2C_GetError(&hi2c1);
            printf("[%lu ms] registration FAILED  status=%lu err=0x%03lX\r\n",
                   (unsigned long)now, (unsigned long)mcp_last_status,
                   (unsigned long)mcp_last_error);
          }
        }
        else
        {
          mcp_probe_count++;
          mcp_last_error = HAL_I2C_GetError(&hi2c1);
          printf("[%lu ms] 0x%02X not responding  err=0x%03lX\r\n",
                 (unsigned long)now, (unsigned)MCP_ADDR_7BIT,
                 (unsigned long)mcp_last_error);
        }
      }

      /* ---- Receive one frame -------------------------------------------- */
      {
        mcp_rx_result_t res = MCP_Rx_Start(&mcp_rx, &hi2c1, mcp_rx_buf,
                                           (uint16_t)(MCP_RX_BUF_LEN - 1U), &got);

        mcp_last_status = MCP_Rx_LastHalStatus();

        if (res == MCP_RX_OK)
        {
          mcp_frames_ok++;
          mcp_state = (uint32_t)MCP_STATE_ACTIVE;
          mcp_last_len = got;
          MCP_LedNoteFrame(now);
          mcp_read_fail = 0U;   /* any success clears the consecutive-fault run */

          memset((void *)mcp_last_payload, 0, sizeof(mcp_last_payload));
          if (got < MCP_RX_BUF_LEN)
          {
            memcpy((void *)mcp_last_payload, mcp_rx_buf, got);
          }

          if (mcp_rx_buf[0] == (uint8_t)MCP_JSON_PREFIX)
          {
            mcp_json_frames++;
          }

          printf("\r\n=== MCP FRAME #%lu  len=%u  header=%02X %02X %02X %02X %02X %02X %02X %02X ===\r\n",
                 (unsigned long)mcp_frames_ok, (unsigned)got,
                 (unsigned)mcp_last_header[0], (unsigned)mcp_last_header[1],
                 (unsigned)mcp_last_header[2], (unsigned)mcp_last_header[3],
                 (unsigned)mcp_last_header[4], (unsigned)mcp_last_header[5],
                 (unsigned)mcp_last_header[6], (unsigned)mcp_last_header[7]);
          MCP_Dump(mcp_rx_buf, got);

          if ((got == 1U) && (mcp_rx_buf[0] < MCP_JSON_PREFIX))
          {
            /* The module's own status broadcast, not a model tool call.
             * State 5 (listening) means it is up and ready for speech. */
            printf("    ^ module status: %u = %s\r\n",
                   (unsigned)mcp_rx_buf[0], MCP_StateName(mcp_rx_buf[0]));
          }

          /* Any JSON object is treated as a tool call and acknowledged.
           *
           * This deliberately does NOT require a "tool_name" field. Measured
           * against the real module, the payload arrives as bare parameters:
           *     {"heading_deg":0,"speed_pct":30,"duration_ms":1000}
           * -- no tool_name wrapper. Demanding tool_name meant the call was
           * received, reassembled and checksummed correctly, yet never
           * acknowledged, so the model blocked forever waiting: both tools are
           * registered with "block":"true", and the vendor host states plainly
           * that without this response the model waits indefinitely.
           *
           * Identified by the leading '{' rather than by content, so it stays
           * correct whichever wrapper this firmware version uses. A 1-byte
           * status frame never starts with '{'. */
          if ((got > 1U) && (mcp_rx_buf[0] == (uint8_t)MCP_JSON_PREFIX))
          {
            mcp_tool_calls++;

            /* Name the tool from its parameter set: these are the only two
             * tools registered, and their argument names are disjoint. */
            if (strstr((char *)mcp_rx_buf, "heading_deg") != NULL)
            {
              printf(">>> TOOL CALL: move_linear -- replying action_finish\r\n");
            }
            else if (strstr((char *)mcp_rx_buf, "turn") != NULL)
            {
              printf(">>> TOOL CALL: drive_arc -- replying action_finish\r\n");
            }
            else
            {
              printf(">>> TOOL CALL (unrecognised parameter set) -- replying action_finish\r\n");
            }

            if (MCP_SendBytes(&hi2c1, (const uint8_t *)MCP_ACK_JSON,
                              (uint16_t)(sizeof(MCP_ACK_JSON) - 1U)) == HAL_OK)
            {
              mcp_finish_sent++;
              printf("    action_finish sent\r\n");
            }
            else
            {
              printf("    action_finish FAILED err=0x%03lX\r\n",
                     (unsigned long)HAL_I2C_GetError(&hi2c1));
            }
          }
        }
        else
        {
          mcp_last_error = MCP_Rx_LastHalError();

          /* An idle slave simply does not answer a read request, so a NACK is
           * the NORMAL "no frame waiting" result -- the vendor host treats it
           * the same way and just retries. Counting it as a fault used to
           * trigger a bus recovery every 2 seconds while idle, and that reset
           * tears the peripheral down and bit-bangs SCL/SDA, which can cut
           * across a transfer the module is starting.
           *
           * Only a genuine bus fault justifies recovery. */
          if (res == MCP_RX_NOTHING)
          {
            mcp_nack_count++;      /* expected while nothing is being sent */
          }
          else if (res == MCP_RX_FAULT)
          {
            mcp_read_fail++;

            if (mcp_read_fail >= MCP_BUS_FAULT_LIMIT)
            {
              mcp_read_fail = 0U;
              mcp_bus_recover_count++;
              printf("[%lu ms] bus fault err=0x%03lX -- recovering (#%lu)\r\n",
                     (unsigned long)now, (unsigned long)mcp_last_error,
                     (unsigned long)mcp_bus_recover_count);
              MCP_BusRecover(&hi2c1);
            }
          }
          /* MCP_RX_BAD_HEADER / MCP_RX_BAD_CHECKSUM were already counted where
           * they were detected, and are not bus faults. */
        }
      }

      /* ---- Periodic status summary ---------------------------------------
       * Without this, "no output" is ambiguous: it could mean the module is
       * quiet (idle, normal) or that every read is failing (broken). ----- */
      if ((now - mcp_last_status_tick) >= MCP_STATUS_INTERVAL_MS)
      {
        mcp_last_status_tick = now;
        MCP_StatusReport(now);
      }
    }

    MCP_LedService();

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
