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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* Where the bring-up state machine currently is. Purely for observation. */
typedef enum
{
  MCP_STATE_PROBING = 0U,   /* nothing ACKed at 0x55 yet            */
  MCP_STATE_SILENT  = 1U,   /* 0x55 ACKs, registration sent, no frames yet */
  MCP_STATE_ACTIVE  = 2U    /* frames are arriving                  */
} MCP_State_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ==========================================================================
 * WonderLLM MCP over I2C  --  STM32 is the I2C MASTER, ESP32-S3 the SLAVE.
 *
 * The ESP32-S3 answers at 7-bit 0x55 and speaks "MCP" framing:
 *     [8-byte header][payload ...][XOR checksum]
 *
 * !! READS ARE PURE READS. THERE IS NO REGISTER ADDRESS.
 *    Do NOT send 0x64 (or any other register byte) before reading --
 *    that was the CI1302 register protocol and it does not exist here.
 *    Verified against the vendor master source (WonderLLM.cpp):
 *        Wire.requestFrom(WonderLLM_SLAVE_ADDRESS, size, stop_flag);
 *    whose STM32 equivalent is HAL_I2C_Master_Receive().
 *
 * Header layout (byte order verified character-by-character against the
 * vendor's receive_frame_head()):
 *     [0]     0xAA               magic
 *     [1]     0x55               magic
 *     [2..3]  data_len           BIG endian
 *     [4..5]  part_id            LITTLE endian
 *     [6..7]  part_num           LITTLE endian
 * ======================================================================== */
#define MCP_ADDR_7BIT         0x55U
#define MCP_ADDR_HAL_W        ((uint16_t)(MCP_ADDR_7BIT << 1))         /* 0xAA */
#define MCP_ADDR_HAL_R        ((uint16_t)((MCP_ADDR_7BIT << 1) | 1U))  /* 0xAB */

#define MCP_HDR_LEN           8U
#define MCP_DATA_LEN_MAX      31U      /* vendor cap: 1..31 */
#define MCP_MAGIC_0           0xAAU
#define MCP_MAGIC_1           0x55U

#define MCP_TX_TIMEOUT_MS     100U
#define MCP_RX_TIMEOUT_MS     20U

#define MCP_POLL_INTERVAL_MS  100U
#define MCP_REG_RETRY_MS      2000U    /* re-send registration while no JSON yet */
#define MCP_STATUS_INTERVAL_MS 5000U   /* periodic counter summary */

/* A frame is treated as "the model answered" iff its payload looks like the
 * start of a JSON object. Registration retries until that happens. */
#define MCP_JSON_PREFIX       '{'

/* The LED shows live activity, not history: it stays solid only while frames
 * are actually arriving. */
#define MCP_ACTIVITY_WINDOW_MS 2000U

/* !! The vendor requires 400 kHz WHILE TRANSMITTING MCP JSON.
 *    Vendor comment (WonderLLM_porting.cpp), translated:
 *      "For longer strings (especially Chinese) you MUST raise the I2C rate
 *       to 400,000, otherwise WonderLLM cannot receive the data completely."
 *    Reads are a handful of bytes and are fine at 100 kHz. */
#define MCP_I2C_SPEED_WRITE   400000U
#define MCP_I2C_SPEED_READ    100000U

/* Raw receive buffer. Static, not on the stack -- fragmented frames can run
 * past 64 bytes, and the main loop must keep its stack small. */
#define MCP_RX_BUF_LEN        256U

/* Heartbeat LED: board LED0 (red) on PA8.
 * NOTE: PC13 is NOT an LED on this board (it is the touch panel CS). */
#define MCP_USE_HEARTBEAT_LED 1U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* ---- Live observation state -------------------------------------------
 * All volatile: the project builds at -O3, so a non-volatile static can be
 * kept in a register and become invisible to the debugger. ---------------- */

volatile uint32_t mcp_state            = MCP_STATE_PROBING;
volatile uint32_t mcp_probe_count      = 0U;  /* failed probes            */
volatile uint32_t mcp_probe_ok_count   = 0U;  /* successful ACKs at 0x55  */
volatile uint32_t mcp_register_attempts = 0U; /* registration bursts sent */
volatile uint32_t mcp_register_ok      = 0U;
volatile uint32_t mcp_frames_ok        = 0U;  /* fully valid frames       */
volatile uint32_t mcp_json_frames      = 0U;  /* frames whose payload is JSON */
volatile uint32_t mcp_bad_header       = 0U;
volatile uint32_t mcp_bad_checksum     = 0U;
volatile uint32_t mcp_read_fail        = 0U;  /* genuine I2C fault (BUSY/TIMEOUT) */
volatile uint32_t mcp_nack_count       = 0U;  /* idle NACK -- NOT a fault */
volatile uint32_t mcp_tool_calls       = 0U;  /* frames containing tool_name */
volatile uint32_t mcp_finish_sent      = 0U;  /* action_finish replies    */
volatile uint32_t mcp_bus_recover_count = 0U; /* bus resets performed     */
volatile uint32_t mcp_tx_total_bytes   = 0U;  /* bytes clocked out, for sizing checks */
volatile uint32_t mcp_last_status      = 0U;  /* last HAL status          */
volatile uint32_t mcp_last_error       = 0U;  /* last HAL_I2C error code  */
volatile uint32_t mcp_i2c_speed_hz     = 0U;  /* speed currently programmed */

/* Last frame, for the debugger and for printing. */
volatile uint8_t  mcp_last_header[MCP_HDR_LEN];
volatile uint16_t mcp_last_len    = 0U;
volatile uint8_t  mcp_last_payload[MCP_RX_BUF_LEN];

static uint8_t  mcp_rx_buf[MCP_RX_BUF_LEN];
static uint32_t mcp_last_poll_tick     = 0U;
static uint32_t mcp_last_register_tick = 0U;
static uint32_t mcp_last_status_tick   = 0U;
static uint32_t mcp_last_frame_tick    = 0U;  /* drives LED activity, see MCP_ACTIVITY_WINDOW_MS */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

static HAL_StatusTypeDef MCP_SetSpeed(uint32_t speed);
static HAL_StatusTypeDef MCP_Probe(void);
static HAL_StatusTypeDef MCP_SendBytes(const uint8_t *data, uint16_t len);
static HAL_StatusTypeDef MCP_RegisterTools(void);
static HAL_StatusTypeDef MCP_ReadFrame(uint8_t *out, uint16_t out_cap, uint16_t *out_len);
static HAL_StatusTypeDef MCP_BusRecover(void);
static void MCP_Dump(const uint8_t *data, uint16_t len);
static const char *MCP_StateName(uint8_t code);
#if MCP_USE_HEARTBEAT_LED
static void MCP_LedInit(void);
static void MCP_LedService(void);
#endif

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* --------------------------------------------------------------------------
 * printf retarget onto USART1 (PA9/PA10 -> onboard CH340 -> the same USB
 * cable that powers/programs the board, so no extra hardware is needed).
 *
 * MicroLIB is OFF in this project, so the full C library is linked. Without
 * the semihosting opt-out below, the linker pulls in the semihosting stubs
 * and the first printf() executes a BKPT that only a debugger can service --
 * the board would silently stop with no output at all.
 * ------------------------------------------------------------------------ */
#pragma import(__use_no_semihosting)

struct __FILE
{
  int handle;
};

FILE __stdout;

void _sys_exit(int x)
{
  (void)x;
}

void _ttywrch(int ch)
{
  (void)ch;
}

int fputc(int ch, FILE *f)
{
  uint8_t byte = (uint8_t)ch;
  (void)f;
  HAL_UART_Transmit(&huart1, &byte, 1U, 100U);
  return ch;
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  mcp_i2c_speed_hz = MCP_I2C_SPEED_READ;   /* matches MX_I2C1_Init: 100 kHz */
  mcp_last_register_tick = HAL_GetTick();

#if MCP_USE_HEARTBEAT_LED
  MCP_LedInit();
#endif

  printf("\r\n\r\n=== WonderLLM MCP bring-up (STM32 as I2C master) ===\r\n");
  printf("MCU STM32F103RCT6 @ %lu Hz\r\n", (unsigned long)SystemCoreClock);
  printf("I2C1 PB6=SCL PB7=SDA, slave 0x%02X (HAL write 0x%02X / read 0x%02X)\r\n",
         (unsigned)MCP_ADDR_7BIT,
         (unsigned)MCP_ADDR_HAL_W,
         (unsigned)MCP_ADDR_HAL_R);
  printf("\r\nSTARTUP ORDER THAT MATTERS:\r\n");
  printf("  The module only accepts MCP tool registration during its own\r\n");
  printf("  startup window. This firmware now retries registration every\r\n");
  printf("  %u ms until frames start arriving.\r\n", (unsigned)MCP_REG_RETRY_MS);
  printf("  => POWER-CYCLE THE ESP32-S3 MODULE NOW, while this runs.\r\n\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now = HAL_GetTick();
    uint16_t got = 0U;

    if ((now - mcp_last_poll_tick) >= MCP_POLL_INTERVAL_MS)
    {
      mcp_last_poll_tick = now;

      /* ---- Registration retry -------------------------------------------
       * Re-send the tool registration periodically until a frame actually
       * carries JSON, i.e. until the model has been given its tools and has
       * responded.
       *
       * This deliberately does NOT stop at the first frame received. An
       * earlier version keyed on mcp_frames_ok, so the very first 1-byte
       * event frame (a CI1302 power-on/wake notice) permanently disabled
       * registration -- and because the module only honours registration
       * during its startup window, a later module reboot could then never be
       * registered at all. That is why chatting with the model produced
       * nothing on the bus.
       * Re-sending costs one short write when already registered. -------- */
      if ((mcp_json_frames == 0U) && ((now - mcp_last_register_tick) >= MCP_REG_RETRY_MS))
      {
        mcp_last_register_tick = now;
        mcp_register_attempts++;

        if (MCP_Probe() == HAL_OK)
        {
          mcp_probe_ok_count++;

          if (MCP_RegisterTools() == HAL_OK)
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
      mcp_last_status = (uint32_t)MCP_ReadFrame(mcp_rx_buf,
                                                (uint16_t)(MCP_RX_BUF_LEN - 1U),
                                                &got);

      if (mcp_last_status == (uint32_t)HAL_OK)
      {
        mcp_frames_ok++;
        mcp_state = (uint32_t)MCP_STATE_ACTIVE;
        mcp_last_len = got;
        mcp_last_frame_tick = now;
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

        /* A tool call carries a tool_name field. Acknowledge it, or the
         * module waits forever: both vendor tools are registered with
         * "block":"true". */
        if (strstr((char *)mcp_rx_buf, "tool_name") != NULL)
        {
          mcp_tool_calls++;
          printf(">>> TOOL CALL -- replying action_finish\r\n");

          if (MCP_SendBytes((const uint8_t *)"{\"command\":\"action_finish\",\"params\":\"true\"}",
                            (uint16_t)(sizeof("{\"command\":\"action_finish\",\"params\":\"true\"}") - 1U)) == HAL_OK)
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
        mcp_last_error = HAL_I2C_GetError(&hi2c1);

        /* An idle slave simply does not answer a read request, so NACK
         * (HAL_I2C_ERROR_AF) is the NORMAL "no frame waiting" result -- the
         * vendor host treats it the same way and just retries. Counting it as
         * a fault used to trigger MCP_BusRecover() every 2 seconds while
         * idle, and that reset tears the peripheral down and bit-bangs
         * SCL/SDA, which can cut across a transfer the module is starting.
         *
         * Only a genuine bus fault (BUSY latched, or the bus never clocked)
         * justifies recovery. */
        if ((mcp_last_error & (uint32_t)HAL_I2C_ERROR_AF) != 0U)
        {
          mcp_nack_count++;      /* expected while nothing is being sent */
        }
        else
        {
          mcp_read_fail++;

          if (mcp_read_fail >= 20U)
          {
            mcp_read_fail = 0U;
            mcp_bus_recover_count++;
            printf("[%lu ms] bus fault err=0x%03lX -- recovering (#%lu)\r\n",
                   (unsigned long)now, (unsigned long)mcp_last_error,
                   (unsigned long)mcp_bus_recover_count);
            MCP_BusRecover();
          }
        }
      }

      /* ---- Periodic status summary ---------------------------------------
       * Without this, "no output" is ambiguous: it could mean the module is
       * quiet (idle, normal) or that every read is failing (broken). ----- */
      if ((now - mcp_last_status_tick) >= MCP_STATUS_INTERVAL_MS)
      {
        mcp_last_status_tick = now;
        printf("[%lu ms] state=%lu ack=%lu reg=%lu frames=%lu json=%lu tool=%lu "
               "nack=%lu rdfail=%lu badhdr=%lu badchk=%lu\r\n",
               (unsigned long)now, (unsigned long)mcp_state,
               (unsigned long)mcp_probe_ok_count,
               (unsigned long)mcp_register_ok,
               (unsigned long)mcp_frames_ok,
               (unsigned long)mcp_json_frames,
               (unsigned long)mcp_tool_calls,
               (unsigned long)mcp_nack_count,
               (unsigned long)mcp_read_fail,
               (unsigned long)mcp_bad_header,
               (unsigned long)mcp_bad_checksum);
      }
    }

#if MCP_USE_HEARTBEAT_LED
    MCP_LedService();
#endif

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

/* --------------------------------------------------------------------------
 * Reprogram the I2C bus speed at runtime.
 *
 * The vendor requires 400 kHz while sending MCP JSON and we use 100 kHz for
 * the short reads, so the speed has to change at runtime.
 *
 * Note on the F1 HAL: HAL_I2C_Init() only calls HAL_I2C_MspInit() when
 * State == HAL_I2C_STATE_RESET, so calling it here skips MspInit. That is
 * harmless -- the pins are already AF_OD from the first init and the HAL
 * reprograms CCR/FREQ either way. Do NOT "fix" this by calling DeInit first:
 * that would tear the pins down and they would stay plain GPIO inputs.
 * ------------------------------------------------------------------------ */
static HAL_StatusTypeDef MCP_SetSpeed(uint32_t speed)
{
  if (mcp_i2c_speed_hz == speed)
  {
    return HAL_OK;
  }

  hi2c1.Init.ClockSpeed = speed;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    return HAL_ERROR;
  }

  mcp_i2c_speed_hz = speed;
  return HAL_OK;
}

/* --------------------------------------------------------------------------
 * Address-only ACK test. This is what the scan already proved works.
 * ------------------------------------------------------------------------ */
static HAL_StatusTypeDef MCP_Probe(void)
{
  MCP_SetSpeed(MCP_I2C_SPEED_READ);
  return HAL_I2C_IsDeviceReady(&hi2c1, MCP_ADDR_HAL_W, 2U, 20U);
}

/* --------------------------------------------------------------------------
 * Send a byte stream to the module: ONE transaction, no chunking.
 *
 * An earlier version split the payload into 32-byte chunks and put a STOP
 * after each one. That was wrong on both counts:
 *   - the vendor's Python host sends the whole JSON in a single transaction
 *   - the vendor's Arduino host chunks with Wire.endTransmission(false), i.e.
 *     a repeated START and NO stop between chunks
 * Neither releases the bus mid-message. A STOP after every 32 bytes makes the
 * slave see ~14 separate short messages instead of one 422-byte JSON, so the
 * registration is fragmented and discarded -- which is exactly why tool
 * registration appeared to succeed but the model never called anything.
 *
 * Chunking is also unnecessary here: HAL_I2C_Master_Transmit takes a uint16_t
 * size and clocks it all out in one blocking transfer. At 400 kHz, 422 bytes
 * take about 9.5 ms -- well inside MCP_TX_TIMEOUT_MS.
 * ------------------------------------------------------------------------ */
static HAL_StatusTypeDef MCP_SendBytes(const uint8_t *data, uint16_t len)
{
  HAL_StatusTypeDef st;

  /* 400 kHz is mandatory for MCP JSON (see MCP_I2C_SPEED_WRITE). */
  if (MCP_SetSpeed(MCP_I2C_SPEED_WRITE) != HAL_OK)
  {
    return HAL_ERROR;
  }

  st = HAL_I2C_Master_Transmit(&hi2c1, MCP_ADDR_HAL_W,
                               (uint8_t *)data, len, MCP_TX_TIMEOUT_MS);

  if (st == HAL_OK)
  {
    mcp_tx_total_bytes += (uint32_t)len;
  }

  return st;
}

/* --------------------------------------------------------------------------
 * Register the two robot tools, then switch the module into MCP mode.
 *
 * The wording is copied verbatim from the project's authoritative tool
 * definitions (robot-planner-skill/i2c/act_tool_strings.h), which are kept
 * aligned with acts.json. Earlier this file used hand-written paraphrase --
 * including a different command sentence and ASCII colons where the original
 * has the full-width colon U+FF1A -- on the theory that any equivalent text
 * would do. Since the model only ever sees this text, any drift changes what
 * it believes the tool does, so the reference strings are reproduced exactly.
 * ------------------------------------------------------------------------ */
/* ARM Compiler 5 reads source in the system codepage (GBK on a Chinese
 * Windows install), NOT UTF-8, so literal Chinese here is misparsed and can
 * even swallow the closing quote via a stray 0x5C byte. The text below is
 * therefore stored as explicit UTF-8 byte escapes, split into separate
 * literals at every ASCII boundary so no \x escape can absorb a following
 * hex digit ("\x82d" would otherwise be read as one escape). The bytes sent
 * to the module are identical UTF-8, so the JSON stays readable to the model.
 *
 * NOTE: "\xE3\x80\x82" is U+3002 (。), "\xEF\xBC\x8C" is U+FF0C (，) and
 * "\xEF\xBC\x9A" is U+FF1A (：) -- the full-width forms, as in the original. */
static const char MCP_TOOL_MOVE[] =
  "{\"tool_name\":\"self.robot.move_linear\","
  "\"command\":\""
  "\xE4\xBB\x85\xE5\x9C\xA8\xE7\x94\xA8\xE6\x88\xB7\xE8\xA6\x81\xE6\xB1\x82\xE7\x9B\xB4\xE7\xBA\xBF\xE7\xA7\xBB\xE5\x8A\xA8\xEF\xBC\x8C\xE4\xB8\x94\xE8\xB7\xAF\xE5\xBE\x84\xE5\xAE\x89\xE5\x85\xA8\xE6\x98\x8E\xE7\xA1\xAE\xE6\x97\xB6\xE8\xB0\x83\xE7\x94\xA8\xE3\x80\x82"
  "heading_deg"
  "\xE6\x98\xAF\xE7\x9B\xB8\xE5\xAF\xB9\xE5\xBD\x93\xE5\x89\x8D\xE6\x9C\x9D\xE5\x90\x91\xE7\x9A\x84\xE8\xA7\x92\xE5\xBA\xA6\xEF\xBC\x8C\xE5\x8F\xB3\xE8\xBD\xAC\xE4\xB8\xBA\xE6\xAD\xA3\xEF\xBC\x8C\xE5\xB7\xA6\xE8\xBD\xAC\xE4\xB8\xBA\xE8\xB4\x9F\xE3\x80\x82"
  "speed_pct"
  "\xE6\x98\xAF\xE9\x80\x9F\xE5\xBA\xA6\xE7\x99\xBE\xE5\x88\x86\xE6\xAF\x94\xE3\x80\x82"
  "duration_ms"
  "\xE6\x98\xAF\xE6\x9C\x89\xE9\x99\x90\xE8\xBF\x90\xE5\x8A\xA8\xE6\x97\xB6\xE9\x97\xB4\xE3\x80\x82\","
  "\"params\":[[\"heading_deg\",\"int\",-180,180],"
  "[\"speed_pct\",\"int\",0,50],[\"duration_ms\",\"int\",100,3000]],"
  "\"block\":\"true\",\"return\":\"false\"}";

static const char MCP_TOOL_ARC[] =
  "{\"tool_name\":\"self.robot.drive_arc\","
  "\"command\":\""
  "\xE4\xBB\x85\xE5\x9C\xA8\xE7\x94\xA8\xE6\x88\xB7\xE8\xA6\x81\xE6\xB1\x82\xE8\xBF\x9E\xE7\xBB\xAD\xE6\x9B\xB2\xE7\xBA\xBF\xE8\xA1\x8C\xE9\xA9\xB6\xEF\xBC\x8C\xE4\xB8\x94\xE8\xB7\xAF\xE5\xBE\x84\xE5\xAE\x89\xE5\x85\xA8\xE6\x98\x8E\xE7\xA1\xAE\xE6\x97\xB6\xE8\xB0\x83\xE7\x94\xA8\xE3\x80\x82"
  "turn"
  "\xE5\x8F\xAA\xE8\x83\xBD\xE6\x98\xAF"
  "left"
  "\xE6\x88\x96"
  "right"
  "\xE3\x80\x82"
  "radius_mm"
  "\xE6\x98\xAF\xE6\x9B\xB2\xE7\xBA\xBF\xE5\x8D\x8A\xE5\xBE\x84\xE3\x80\x82"
  "speed_pct"
  "\xE6\x98\xAF\xE9\x80\x9F\xE5\xBA\xA6\xE7\x99\xBE\xE5\x88\x86\xE6\xAF\x94\xE3\x80\x82"
  "duration_ms"
  "\xE6\x98\xAF\xE6\x9C\x89\xE9\x99\x90\xE8\xBF\x90\xE5\x8A\xA8\xE6\x97\xB6\xE9\x97\xB4\xE3\x80\x82\","
  "\"params\":[[\"turn\",\"string\"],[\"radius_mm\",\"int\",200,2000],"
  "[\"speed_pct\",\"int\",0,35],[\"duration_ms\",\"int\",100,3000]],"
  "\"block\":\"true\",\"return\":\"false\"}";

static const char MCP_TOOL_FINISH[] = "{\"command\":\"mcp_setting\",\"params\":\"true\"}";

static HAL_StatusTypeDef MCP_RegisterTools(void)
{
  HAL_StatusTypeDef st;

  st = MCP_SendBytes((const uint8_t *)MCP_TOOL_MOVE, (uint16_t)(sizeof(MCP_TOOL_MOVE) - 1U));
  HAL_Delay(100U);
  if (st != HAL_OK) { return st; }

  st = MCP_SendBytes((const uint8_t *)MCP_TOOL_ARC, (uint16_t)(sizeof(MCP_TOOL_ARC) - 1U));
  HAL_Delay(100U);
  if (st != HAL_OK) { return st; }

  st = MCP_SendBytes((const uint8_t *)MCP_TOOL_FINISH, (uint16_t)(sizeof(MCP_TOOL_FINISH) - 1U));
  HAL_Delay(100U);

  /* Back to the read speed for the short frame reads. */
  MCP_SetSpeed(MCP_I2C_SPEED_READ);
  return st;
}

/* --------------------------------------------------------------------------
 * Receive one complete MCP frame.
 *
 * Mirrors the vendor's receive_frame() exactly:
 *   - read the 8-byte header, check the AA 55 magic
 *   - data_len is big endian, part_id and part_num are little endian
 *   - part_id must be 1, otherwise frames were dropped: abort rather than
 *     deliver a truncated message
 *   - per part: wait 10 ms, read data_len+1 bytes, last byte is the XOR
 *   - between parts: wait 100 ms, then read the NEXT header
 * ------------------------------------------------------------------------ */
static HAL_StatusTypeDef MCP_ReadFrame(uint8_t *out, uint16_t out_cap, uint16_t *out_len)
{
  uint8_t  hdr[MCP_HDR_LEN];
  uint8_t  part[MCP_DATA_LEN_MAX + 1U];
  uint16_t data_len = 0U;
  uint16_t part_id  = 0U;
  uint16_t part_num = 0U;
  uint16_t total    = 0U;
  uint16_t i;
  uint16_t k;
  uint8_t  sum;
  HAL_StatusTypeDef st;

  MCP_SetSpeed(MCP_I2C_SPEED_READ);

  /* ---- header 1 ---- */
  st = HAL_I2C_Master_Receive(&hi2c1, MCP_ADDR_HAL_R, hdr, MCP_HDR_LEN, MCP_RX_TIMEOUT_MS);
  if (st != HAL_OK)
  {
    return st;
  }

  memcpy((void *)mcp_last_header, hdr, MCP_HDR_LEN);

  if ((hdr[0] != MCP_MAGIC_0) || (hdr[1] != MCP_MAGIC_1))
  {
    mcp_bad_header++;
    return HAL_ERROR;
  }

  data_len = (uint16_t)(((uint16_t)hdr[2] << 8) | (uint16_t)hdr[3]);  /* big endian */
  part_id  = (uint16_t)(((uint16_t)hdr[5] << 8) | (uint16_t)hdr[4]);  /* little     */
  part_num = (uint16_t)(((uint16_t)hdr[7] << 8) | (uint16_t)hdr[6]);  /* little     */

  if ((data_len == 0U) || (data_len > MCP_DATA_LEN_MAX))
  {
    mcp_bad_header++;
    return HAL_ERROR;
  }

  /* Not the first fragment means data was lost before this point; the
   * remainder would be a truncated JSON document. */
  if ((part_id != 1U) || (part_num == 0U))
  {
    mcp_bad_header++;
    return HAL_ERROR;
  }

  for (i = 1U; i <= part_num; i++)
  {
    HAL_Delay(10U);

    st = HAL_I2C_Master_Receive(&hi2c1, MCP_ADDR_HAL_R, part,
                                (uint16_t)(data_len + 1U), MCP_RX_TIMEOUT_MS);
    if (st != HAL_OK)
    {
      return st;
    }

    sum = 0U;
    for (k = 0U; k < data_len; k++)
    {
      sum = (uint8_t)(sum ^ part[k]);
    }

    if (part[data_len] != sum)
    {
      mcp_bad_checksum++;
      return HAL_ERROR;
    }

    if ((uint16_t)(total + data_len) <= out_cap)
    {
      memcpy(out + total, part, data_len);
      total = (uint16_t)(total + data_len);
    }

    /* ---- next header, when the message is fragmented ---- */
    if (i < part_num)
    {
      HAL_Delay(100U);

      st = HAL_I2C_Master_Receive(&hi2c1, MCP_ADDR_HAL_R, hdr,
                                  MCP_HDR_LEN, MCP_RX_TIMEOUT_MS);
      if (st != HAL_OK)
      {
        return st;
      }

      if ((hdr[0] != MCP_MAGIC_0) || (hdr[1] != MCP_MAGIC_1))
      {
        mcp_bad_header++;
        return HAL_ERROR;
      }

      data_len = (uint16_t)(((uint16_t)hdr[2] << 8) | (uint16_t)hdr[3]);
      if ((data_len == 0U) || (data_len > MCP_DATA_LEN_MAX))
      {
        mcp_bad_header++;
        return HAL_ERROR;
      }
    }
  }

  out[total] = 0U;   /* NUL-terminate so strstr() is safe on a JSON payload */
  *out_len = total;
  return HAL_OK;
}

/* --------------------------------------------------------------------------
 * Bus recovery: if a slave is mid-transfer holding SDA low, the master must
 * clock it free. Nine SCL pulses then a manual STOP.
 *
 * DeInit is the correct teardown here: it sets State to RESET, so the
 * following MX_I2C1_Init() really does re-run MspInit and restore AF_OD.
 * (Using __HAL_I2C_DISABLE() instead leaves State at READY, MspInit is
 * skipped, and the pins silently stay plain GPIO inputs -- which cost us a
 * whole debugging session once already.)
 * ------------------------------------------------------------------------ */
static HAL_StatusTypeDef MCP_BusRecover(void)
{
  GPIO_InitTypeDef gpio = {0};
  uint8_t i;
  uint8_t released = 0U;

  HAL_I2C_DeInit(&hi2c1);

  /* SCL as open-drain output: we only ever pull it low; a pull-up makes the
   * high level. SDA as a plain input so we can watch for release. */
  gpio.Pin   = GPIO_PIN_6;
  gpio.Mode  = GPIO_MODE_OUTPUT_OD;
  gpio.Pull  = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &gpio);

  gpio.Pin  = GPIO_PIN_7;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &gpio);

  for (i = 0U; (i < 9U) && (released == 0U); i++)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);   /* SCL low  */
    HAL_Delay(1U);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);     /* SCL high */
    HAL_Delay(1U);

    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET)
    {
      released = 1U;
    }
  }

  /* Manual STOP. */
  gpio.Pin   = GPIO_PIN_7;
  gpio.Mode  = GPIO_MODE_OUTPUT_OD;
  gpio.Pull  = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &gpio);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);   /* SDA low  */
  HAL_Delay(1U);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);     /* SCL high */
  HAL_Delay(1U);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);     /* SDA high */
  HAL_Delay(1U);

  MX_I2C1_Init();                      /* restores AF_OD + 100 kHz */
  mcp_i2c_speed_hz = MCP_I2C_SPEED_READ;

  return (released != 0U) ? HAL_OK : HAL_ERROR;
}

/* --------------------------------------------------------------------------
 * Print a received frame three ways: hex, ASCII, and raw. The raw dump lets
 * a UTF-8 terminal render the Chinese JSON directly, which makes reading the
 * tool call easy.
 * ------------------------------------------------------------------------ */
static void MCP_Dump(const uint8_t *data, uint16_t len)
{
  uint16_t i;
  uint8_t  b;

  printf("HEX: ");
  for (i = 0U; i < len; i++)
  {
    printf("%02X ", (unsigned)data[i]);
  }
  printf("\r\n");

  /* Render non-printable bytes as '.' -- a 1-byte payload of 0x01/0x03 is a
   * control character and would otherwise print as nothing at all, which
   * looks exactly like "no data received". */
  printf("RAW: ");
  for (i = 0U; i < len; i++)
  {
    b = data[i];
    putchar(((b >= 0x20U) && (b <= 0x7EU)) ? (int)b : (int)'.');
  }
  printf("\r\n");
}

/* --------------------------------------------------------------------------
 * The module broadcasts a single status byte as a 1-byte frame. Naming it
 * makes the log readable and tells us where the module is in its boot
 * sequence -- in particular state 5 (listening), which means it is up,
 * network-connected, and ready to accept speech.
 * ------------------------------------------------------------------------ */
static const char *MCP_StateName(uint8_t code)
{
  static const char *const names[12] =
  {
    "unknown",         /* 0  */
    "starting",        /* 1  */
    "configuring",     /* 2  */
    "idle",            /* 3  */
    "connecting",      /* 4  */
    "listening",       /* 5  */
    "speaking",        /* 6  */
    "upgrading",       /* 7  */
    "activating",      /* 8  */
    "audio_testing",   /* 9  */
    "fatal_error",     /* 10 */
    "invalid_state"    /* 11 */
  };

  return (code < 12U) ? names[code] : "out_of_range";
}

#if MCP_USE_HEARTBEAT_LED
/* --------------------------------------------------------------------------
 * LED0 (red) on PA8, active low, non-blocking.
 *
 *   no ACK at 0x55        -> 2 Hz blink
 *   ACK, no frames yet    -> 1 Hz blink
 *   frames arriving       -> solid ON
 * This is a debugger-independent view of mcp_state.
 * ------------------------------------------------------------------------ */
static void MCP_LedInit(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  gpio.Pin   = GPIO_PIN_8;
  gpio.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio.Pull  = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &gpio);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);   /* off */
}

static void MCP_LedService(void)
{
  static uint32_t next_tick = 0U;
  static uint8_t  on = 0U;
  uint32_t now = HAL_GetTick();

  if (now < next_tick)
  {
    return;
  }

  /* Solid only while frames are genuinely arriving. Keying this off
   * mcp_state alone made the LED latch on after the very first frame and
   * stay on forever, which read as "still receiving" when nothing was --
   * a misleading signal exactly when it mattered most. */
  if ((mcp_last_frame_tick != 0U) &&
      ((now - mcp_last_frame_tick) < MCP_ACTIVITY_WINDOW_MS))
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);   /* solid on */
    next_tick = now + 100U;
    return;
  }

  on = (uint8_t)(on ^ 1U);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, on ? GPIO_PIN_RESET : GPIO_PIN_SET);

  /* 1 Hz once the module ACKs, 2 Hz before that. */
  next_tick = now + ((mcp_probe_ok_count > 0U) ? 500U : 250U);
}
#endif /* MCP_USE_HEARTBEAT_LED */

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
