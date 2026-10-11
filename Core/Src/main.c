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

/* Self-test verdict, reported on UART and (optionally) by LED blink code. */
typedef enum
{
  ASR_VERDICT_PENDING       = 0xFFU,  /* self-test not finished yet        */
  ASR_VERDICT_PASS          = 0x00U,  /* bus alive, module quiet (reads 0x00) */
  ASR_VERDICT_NO_DEVICE     = 0x01U,  /* bus clocks, but nobody ACKs        */
  ASR_VERDICT_DEVICE_SILENT = 0x02U,  /* ACKs, but never returns a valid code */
  ASR_VERDICT_BUS_STUCK     = 0x03U,  /* a line held low                    */
  ASR_VERDICT_NO_PULLUP     = 0x04U   /* no external pull-up fitted -- the
                                       * bus only looks high because of the
                                       * MCU's own weak internal pull-up      */
} ASR_Verdict_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ---- WonderLLM / CI1302 offline-voice slave ---------------------------- */
/* 7-bit address 0x34 -> HAL 8-bit form 0x68. Some units are reconfigured to
 * 7-bit 0x33 through their own register 0x03 (HAL 8-bit form 0x66).
 * Both are probed at startup; whichever ACKs is latched. */
#define ASR_ADDR_7BIT_PRIMARY    (0x34U)
#define ASR_ADDR_7BIT_ALTERNATE  (0x33U)
#define ASR_ADDR_PRIMARY         (ASR_ADDR_7BIT_PRIMARY   << 1)  /* 0x68 */
#define ASR_ADDR_ALTERNATE       (ASR_ADDR_7BIT_ALTERNATE << 1)  /* 0x66 */

#define ASR_RESULT_REGISTER      0x64U     /* one byte of result            */

/* Bus timing. 100 kHz standard mode: a 3-byte transaction is ~90 us, so
 * 10 ms is already a 100x margin. Startup probe uses a longer one because
 * the module may still be settling after its own power-up. */
#define ASR_I2C_TIMEOUT_MS       10U
#define ASR_PROBE_TIMEOUT_MS     500U
#define ASR_POLL_INTERVAL_MS     100U

/* Bus recovery: 9 SCL pulses (the I2C spec's unstick procedure), plus a
 * manual STOP. Only attempted after repeated failure, or immediately when
 * the peripheral is hard-latched in BUSY. */
#define ASR_RECOVER_THRESHOLD    5U        /* consecutive fails before retry */
#define ASR_RECOVER_COOLDOWN_MS  500U      /* min gap between recoveries     */

/* De-duplication window. The module's result hold time is NOT knowable from
 * source, so this is a starting guess; the raw trace below measures it.
 * A command counts as new if the value changed OR this window elapsed. */
#define ASR_DUP_WINDOW_MS        700U

/* Raw trace ring buffer: every successful read is recorded verbatim, with
 * no de-duplication. This is what tells you the module's hold time. */
#define ASR_TRACE_LEN            32U

/* Self-test bounds. */
#define ASR_SELFTEST_ATTEMPTS    20U
#define ASR_SELFTEST_MIN_OK      18U       /* of 20 -> PASS                  */
#define ASR_SELFTEST_PROBE_GAP   50U
#define ASR_SELFTEST_READ_GAP    100U

/* Optional second observation channel: the board's red LED0 on PA8.
 * NOTE: PC13 is NOT an LED on the MiniSTM32 V4 (it is touch CS). PA8 is
 * LED0 (red). Set to 0 to compile the blinker out entirely. */
#define ASR_USE_HEARTBEAT_LED    1U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* ---- Live state (all volatile: Optim=4 is -O3, a non-volatile static
 * would be kept in a register and be invisible to the debugger) ---------- */

volatile uint8_t  asr_device_online   = 0U;   /* hysteresis, see below      */
volatile uint16_t asr_i2c_address     = ASR_ADDR_PRIMARY;
volatile uint8_t  asr_result          = 0U;   /* latest raw byte, verbatim  */
volatile uint8_t  asr_last_command    = 0U;   /* last de-duplicated command */
volatile uint8_t  asr_new_command     = 0U;   /* consume-once flag, host clears */
volatile uint32_t asr_command_count   = 0U;
volatile uint32_t asr_dup_suppressed  = 0U;   /* reads swallowed by the window */
volatile uint32_t asr_read_ok_count   = 0U;
volatile uint32_t asr_read_fail_count = 0U;
volatile uint32_t asr_consecutive_fail = 0U;
volatile uint32_t asr_online_flaps    = 0U;   /* online<->offline transitions  */
volatile uint32_t asr_last_command_tick = 0U;
volatile uint8_t  asr_last_raw_nonzero  = 0U;
volatile uint32_t asr_last_poll_tick    = 0U;   /* non-blocking poll schedule */

/* HAL status/error of the most recent transaction. Keil's Watch window does
 * not resolve the HAL_I2C_ERROR_* names, so compare numerically:
 *   HAL_I2C_ERROR_AF      = 0x04  -> address not acknowledged (bus is fine)
 *   HAL_I2C_ERROR_TIMEOUT = 0x20  -> bus did not clock (pull-ups / stuck) */
volatile HAL_StatusTypeDef asr_i2c_status = HAL_ERROR;
volatile uint32_t          asr_i2c_error  = HAL_I2C_ERROR_NONE;

/* Raw trace: every successful read, in order, no filtering whatsoever.
 * Read this to learn the module's result hold time (Step 6b). */
volatile uint8_t  asr_raw_trace[ASR_TRACE_LEN];
volatile uint8_t  asr_raw_trace_head  = 0U;   /* next write index           */
volatile uint8_t  asr_raw_trace_full  = 0U;

/* Bus recovery counters. recover_count > 0 during a quiet period means the
 * bus is degrading, not that a single command was missed. */
volatile uint32_t asr_recover_count    = 0U;
volatile uint32_t asr_recover_ok_count = 0U;
volatile uint32_t asr_last_recover_tick = 0U;

/* Self-test. verdict is what you look at first. */
volatile uint8_t  asr_selftest_stage    = 0U;
volatile uint8_t  asr_selftest_verdict  = ASR_VERDICT_PENDING;
volatile uint32_t asr_selftest_read_attempts = 0U;
volatile uint32_t asr_selftest_read_ok  = 0U;
volatile uint8_t  asr_selftest_bus_low  = 0U;  /* 1 = a line held low       */
volatile uint8_t  asr_selftest_pullup_present = 0U;  /* 1 = external pull-up found */

/* Per-candidate address probe results, indexed [0]=0x68, [1]=0x66.
 * probe_result: 0 = ACK, 0xFF = no ACK. */
volatile uint8_t  asr_addr_probe_result[2] = { 0xFFU, 0xFFU };
volatile uint32_t asr_probe_status[2]      = { 0U, 0U };
volatile uint32_t asr_probe_error[2]       = { 0U, 0U };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* Bus bit-level recovery (9 clocks + STOP), then re-init via MX_I2C1_Init. */
static HAL_StatusTypeDef ASR_BusRecover(void);
/* Read result register 0x64 once. Wraps HAL_I2C_Mem_Read + error capture. */
static HAL_StatusTypeDef ASR_ReadResult(uint16_t addr8, uint8_t *out);
/* Probe candidates, latch whichever ACKs. Returns chosen address. */
static uint16_t ASR_ProbeAddress(void);
/* Sweep the 7-bit address space and report every address that ACKs. */
static void ASR_BusScan(void);
/* Bounded self-test with a verdict. Runs without any speech. */
static void ASR_SelfTest(void);
/* Push one sample into the raw trace ring. */
static void ASR_TracePush(uint8_t value);
/* Feed one successful sample through the de-duplication rule. */
static void ASR_HandleSample(uint8_t value);
/* Short busy-wait, ~us scale. Used only by bus recovery. */
static void ASR_ShortDelay(uint32_t us);
#if ASR_USE_HEARTBEAT_LED
static void ASR_LedInit(void);
static void ASR_LedService(void);   /* non-blocking, tick driven */
#endif
static const char *ASR_VerdictName(uint8_t verdict);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* --------------------------------------------------------------------------
 * printf retarget onto USART1 (PA9/PA10 -> onboard CH340 -> same USB cable).
 *
 * MicroLIB is OFF in this project (useUlib=0), so the full C library is
 * linked. Without the semihosting opt-out below, the linker pulls in the
 * semihosting stubs and the first printf() executes a BKPT that only a
 * debugger can service -- the board would silently stop with no output.
 * __use_no_semihosting plus these three stubs avoids that entirely.
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

  /* Let the module finish its own power-up before touching the bus. */
  HAL_Delay(1000U);

  memset((void *)asr_raw_trace, 0, sizeof(asr_raw_trace));

#if ASR_USE_HEARTBEAT_LED
  ASR_LedInit();
#endif

  printf("\r\n\r\n=== WonderLLM / CI1302 I2C bring-up ===\r\n");
  printf("MCU STM32F103RCT6 @ %lu Hz, I2C1 PB6=SCL PB7=SDA, %s\r\n",
         (unsigned long)SystemCoreClock,
         "polls reg 0x64 every 100 ms");

  /* One unconditional recovery before anything else: it clears whatever
   * state a previous power cycle left on the bus and doubles as a first
   * hardware sanity check. */
  ASR_BusRecover();

  ASR_SelfTest();

  /* Stage 2 of the self-test already probed both addresses and latched
   * whichever answered, so only re-probe when the address is genuinely in
   * doubt -- the module may simply have been slower to boot than the MCU.
   * Re-probing after a PASS or a stuck bus would just add startup delay. */
  if ((asr_selftest_verdict != ASR_VERDICT_PASS) &&
      (asr_selftest_verdict != ASR_VERDICT_BUS_STUCK) &&
      (asr_selftest_verdict != ASR_VERDICT_NO_PULLUP))
  {
    ASR_ProbeAddress();

    /* Neither documented address answered. Only a scan can distinguish
     * "the module is somewhere undocumented" from "the module is not
     * talking at all" -- and the bus must be healthy for the scan to mean
     * anything, hence NO_PULLUP/BUS_STUCK were excluded above. */
    if (asr_selftest_verdict == ASR_VERDICT_NO_DEVICE)
    {
      ASR_BusScan();
    }
  }

  printf("Address in use: 0x%02X (7-bit 0x%02X)\r\n",
         (unsigned)asr_i2c_address, (unsigned)(asr_i2c_address >> 1));
  printf("Verdict: %u (%s)\r\n",
         (unsigned)asr_selftest_verdict,
         ASR_VerdictName(asr_selftest_verdict));
  /* ASCII only: terminal/encoding varies, and Chinese UTF-8 in a serial
   * window is a common source of confusing mojibake during bring-up. */
  printf("Entering poll loop. Speak: qianjin / houtui / zuozhuan / youzhuan / tingzhi\r\n");
  printf("(forward / backward / left / right / stop)\r\n\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now = HAL_GetTick();
    uint8_t  received = 0U;

    if ((now - asr_last_poll_tick) >= ASR_POLL_INTERVAL_MS)
    {
      asr_last_poll_tick = now;

      asr_i2c_status = ASR_ReadResult(asr_i2c_address, &received);

      if (asr_i2c_status == HAL_OK)
      {
        asr_consecutive_fail = 0U;
        asr_i2c_error = HAL_I2C_ERROR_NONE;

        /* Recover the online flag only after hysteresis cleared it; this
         * keeps a single stray NACK from flapping the flag at 10 Hz. */
        if (asr_device_online == 0U)
        {
          asr_device_online = 1U;
        }

        asr_result = received;
        asr_read_ok_count++;

        ASR_TracePush(received);
        ASR_HandleSample(received);
      }
      else
      {
        asr_read_fail_count++;
        asr_consecutive_fail++;

        /* Capture the error code BEFORE classifying it. It must be read
         * right after the failed transaction; using the value left over
         * from the previous iteration would classify this failure with
         * stale information. */
        asr_i2c_error = HAL_I2C_GetError(&hi2c1);

        /* HAL_BUSY with no error code means the peripheral itself is
         * latched -- retrying cannot help, only recovery can. The internal
         * BUSY check in HAL_I2C_Mem_Read is a hard-coded 25 ms and is not
         * affected by our timeout argument. */
        if ((asr_i2c_status == HAL_BUSY) && (asr_i2c_error == HAL_I2C_ERROR_NONE))
        {
          if (ASR_BusRecover() == HAL_OK)
          {
            asr_recover_ok_count++;
          }
          asr_last_recover_tick = HAL_GetTick();
        }
        else if (asr_consecutive_fail >= ASR_RECOVER_THRESHOLD)
        {
          /* Cool down so a genuinely dead bus does not become a recovery
           * storm that also destroys our ability to observe what is
           * happening. */
          if ((HAL_GetTick() - asr_last_recover_tick) >= ASR_RECOVER_COOLDOWN_MS)
          {
            if (ASR_BusRecover() == HAL_OK)
            {
              asr_recover_ok_count++;
            }
            asr_last_recover_tick = HAL_GetTick();
          }
        }

        if ((asr_device_online != 0U) &&
            (asr_consecutive_fail >= ASR_RECOVER_THRESHOLD))
        {
          asr_device_online = 0U;
          asr_online_flaps++;
        }
      }

      asr_i2c_error = HAL_I2C_GetError(&hi2c1);
    }

#if ASR_USE_HEARTBEAT_LED
    /* Runs every iteration, independent of the poll schedule. */
    ASR_LedService();
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
 * Short busy-wait in the microsecond range. Used only by bus recovery, where
 * the exact clock rate is irrelevant (anything from ~50 kHz to ~400 kHz
 * successfully unsticks a slave). HAL_Delay(1) would be 1 ms per half clock.
 * ------------------------------------------------------------------------ */
static void ASR_ShortDelay(uint32_t us)
{
  /* ~72 MHz core, roughly 5-6 cycles per loop iteration at -O3. */
  volatile uint32_t n = us * 12U;
  while (n-- > 0U)
  {
  }
}

/* --------------------------------------------------------------------------
 * Bus recovery: the I2C spec's unstick procedure.
 *
 * If a slave is mid-transfer and holding SDA low, the master must clock it
 * free. Order matters: HAL_I2C_DeInit() routes through HAL_I2C_MspDeInit(),
 * which disables the I2C1 clock and resets PB6/PB7, so the pins are free
 * to bit-bang only after it runs.
 * ------------------------------------------------------------------------ */
static HAL_StatusTypeDef ASR_BusRecover(void)
{
  GPIO_InitTypeDef gpio = {0};
  uint8_t sda_seen_high = 0U;
  uint8_t i;
  uint8_t finished = 0U;

  asr_recover_count++;

  HAL_I2C_DeInit(&hi2c1);   /* releases PB6/PB7, disables I2C1 clock */

  /* SCL as open-drain output: we only ever pull it low, the external
   * pull-up provides the high level. */
  gpio.Pin   = GPIO_PIN_6;
  gpio.Mode  = GPIO_MODE_OUTPUT_OD;
  gpio.Pull  = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &gpio);

  /* SDA as input so we can observe whether the slave has let go. */
  gpio.Pin  = GPIO_PIN_7;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &gpio);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);   /* release SCL */
  ASR_ShortDelay(5U);

  for (i = 0U; (i < 9U) && (finished == 0U); i++)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);  /* SCL low  */
    ASR_ShortDelay(5U);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);    /* SCL high */
    ASR_ShortDelay(5U);

    /* Sample while SCL is high: a released SDA means we are done. */
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET)
    {
      sda_seen_high = 1U;
      finished = 1U;
    }
  }

  /* Manual STOP: SDA low while SCL high, then release SDA. */
  gpio.Pin   = GPIO_PIN_7;
  gpio.Mode  = GPIO_MODE_OUTPUT_OD;
  gpio.Pull  = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &gpio);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);   /* SDA low  */
  ASR_ShortDelay(5U);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);     /* SCL high */
  ASR_ShortDelay(5U);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);     /* SDA high */
  ASR_ShortDelay(5U);

  /* Hand the pins back to the peripheral by re-running the CubeMX init, so
   * the AF_OD configuration cannot drift from what CubeMX generates. */
  MX_I2C1_Init();

  return (sda_seen_high != 0U) ? HAL_OK : HAL_ERROR;
}

/* --------------------------------------------------------------------------
 * One register-0x64 read. Exactly the transaction the reference host uses:
 * START, addr+W, 0x64, repeated START, addr+R, 1 byte, NACK, STOP --
 * which is what HAL_I2C_Mem_Read produces.
 * ------------------------------------------------------------------------ */
static HAL_StatusTypeDef ASR_ReadResult(uint16_t addr8, uint8_t *out)
{
  return HAL_I2C_Mem_Read(&hi2c1,
                          addr8,
                          (uint16_t)ASR_RESULT_REGISTER,
                          I2C_MEMADD_SIZE_8BIT,
                          out,
                          1U,
                          ASR_I2C_TIMEOUT_MS);
}

/* --------------------------------------------------------------------------
 * Address probe. Uses the protocol's own access rather than
 * HAL_I2C_IsDeviceReady(): the latter only emits a bare address byte and
 * waits for ADDR, which proves an address ACKs but not that the register
 * read actually works -- and the register read is what we need.
 *
 * Each candidate is preceded by a recovery so a NACK on the first cannot
 * leave the peripheral BUSY for the second.
 * ------------------------------------------------------------------------ */
static uint16_t ASR_ProbeAddress(void)
{
  static const uint16_t candidates[2] = { ASR_ADDR_PRIMARY, ASR_ADDR_ALTERNATE };
  uint8_t probe = 0U;
  uint8_t i;

  for (i = 0U; i < 2U; i++)
  {
    ASR_BusRecover();

    asr_probe_status[i] = (uint32_t)HAL_I2C_Mem_Read(&hi2c1,
                                                     candidates[i],
                                                     (uint16_t)ASR_RESULT_REGISTER,
                                                     I2C_MEMADD_SIZE_8BIT,
                                                     &probe,
                                                     1U,
                                                     ASR_PROBE_TIMEOUT_MS);
    asr_probe_error[i] = HAL_I2C_GetError(&hi2c1);

    if (asr_probe_status[i] == (uint32_t)HAL_OK)
    {
      asr_addr_probe_result[i] = 0U;
      asr_i2c_address = candidates[i];
      return candidates[i];
    }

    asr_addr_probe_result[i] = 0xFFU;
  }

  /* Nothing ACKed. Keep polling anyway -- the module may simply be slower
   * to boot than the MCU, so do not latch this as a permanent verdict. */
  asr_i2c_address = ASR_ADDR_PRIMARY;
  return asr_i2c_address;
}

/* --------------------------------------------------------------------------
 * Sweep the 7-bit address space and print every address that ACKs.
 *
 * This is the tool for the situation the handoff cannot resolve: the module
 * answers at neither 0x34 nor 0x33. Either it is at an address nobody
 * documented, or it is not talking. A scan separates those two cases in one
 * run instead of in an afternoon of guessing.
 *
 * Uses HAL_I2C_IsDeviceReady, which only emits a bare address byte and waits
 * for the ADDR flag. That is all a scan needs -- it proves an address ACKs,
 * without assuming anything about the register protocol behind it.
 *
 * Reserved addresses are skipped: 0x00-0x07 (general call / CBUS / high-speed
 * / EEPROM) and 0x78-0x7F (10-bit addressing and reserved).
 * ------------------------------------------------------------------------ */
static void ASR_BusScan(void)
{
  uint8_t addr7;
  uint8_t found = 0U;

  printf("Bus scan (7-bit 0x08-0x77)...\r\n");

  for (addr7 = 0x08U; addr7 <= 0x77U; addr7++)
  {
    if (HAL_I2C_IsDeviceReady(&hi2c1,
                              (uint16_t)(addr7 << 1),
                              2U,          /* 2 trials per address */
                              20U) == HAL_OK)
    {
      printf("  found device at 7-bit 0x%02X (HAL 0x%02X)\r\n",
             (unsigned)addr7, (unsigned)(addr7 << 1));
      found = 1U;
    }
  }

  if (found == 0U)
  {
    printf("  no device ACKed anywhere on the bus.\r\n");
    printf("  Bus reads high but nothing answers: check module power and\r\n");
    printf("  that SDA/SCL are wired and not swapped.\r\n");
  }
}

/* --------------------------------------------------------------------------
 * Push one raw sample into the ring buffer. Deliberately unconditional:
 * this is the evidence that reveals the module's result hold time.
 * ------------------------------------------------------------------------ */
static void ASR_TracePush(uint8_t value)
{
  asr_raw_trace[asr_raw_trace_head] = value;
  asr_raw_trace_head++;
  if (asr_raw_trace_head >= ASR_TRACE_LEN)
  {
    asr_raw_trace_head = 0U;
    asr_raw_trace_full = 1U;
  }
}

/* --------------------------------------------------------------------------
 * De-duplication.
 *
 * Whether the module latches its result until read, holds it for a fixed
 * window, or never clears it is not knowable from source, so the rule must
 * be correct under all three: a sample counts as a NEW command if the value
 * changed OR the window elapsed.
 *
 *   - value-change alone would miss a legitimate repeat ("forward" twice)
 *   - window alone would double-count a value that persists past it
 *   - the OR gives one count per distinct command event
 * ------------------------------------------------------------------------ */
static void ASR_HandleSample(uint8_t value)
{
  uint32_t now;

  if (value == 0U)
  {
    /* A zero read is the natural end-of-command marker. */
    return;
  }

  now = HAL_GetTick();

  if ((value != asr_last_raw_nonzero) ||
      ((now - asr_last_command_tick) > ASR_DUP_WINDOW_MS))
  {
    asr_last_command      = value;
    asr_last_raw_nonzero  = value;
    asr_last_command_tick = now;
    asr_new_command       = 1U;
    asr_command_count++;

    printf("CMD 0x%02X  #%lu  (t=%lu ms)\r\n",
           (unsigned)value,
           (unsigned long)asr_command_count,
           (unsigned long)now);
  }
  else
  {
    /* Suppressed as a duplicate. Counted so the window can be tuned. */
    asr_dup_suppressed++;
  }
}

/* --------------------------------------------------------------------------
 * Self-test. Runs with no speech at all, and answers the question that
 * actually matters: is the bus alive, and is the module quiet?
 * ------------------------------------------------------------------------ */
static void ASR_SelfTest(void)
{
  GPIO_InitTypeDef gpio = {0};
  uint8_t x;
  uint8_t probe = 0U;
  uint8_t ok = 0U;
  uint32_t err_seen = 0U;
  uint8_t nonzero_seen = 0U;
  uint8_t invalid_code_seen = 0U;

  asr_selftest_verdict = ASR_VERDICT_PENDING;

  /* ---- Stage 1: is there a real external pull-up? ----------------------
   * The naive test -- configure the pins as pulled-up inputs and read --
   * is useless, because the STM32F1's HAL programs a GPIO_PULLUP for INPUT
   * mode by setting the ODR bit (stm32f1xx_hal_gpio.c, the GPIO_PULLUP
   * branch). That internal pull-up (~30-50k) holds the line high even on a
   * bare board with nothing attached, so the test passes when it should
   * fail. It measures the MCU's own pull-up, not the board's.
   *
   * Instead, use the resolution difference between a real pull-up and a
   * floating pin:
   *
   *   1. Drive the line LOW through an open-drain output. This discharges
   *      the bus capacitance. Open-drain means we can only pull down; the
   *      high level must come from a pull-up resistor, whoever provides it.
   *   2. Release it (switch to a plain input, NO internal pull-up) and wait.
   *      A real external pull-up (~5k) recharges the line in well under
   *      50 us: tau = 5k * ~200pF = 1 us. With NO external pull-up the pin
   *      is floating; the only current is leakage, so it stays low.
   *   3. Re-read with the internal pull-up enabled. This separates
   *      "no pull-up" (now reads high -- the MCU supplies it) from
   *      "something is holding the bus down" (still reads low).
   *
   * Outcomes:
   *   step 2 high                        -> external (or module) pull-up present
   *   step 2 low, step 3 high            -> NO external pull-up: fits one
   *   step 2 low, step 3 low             -> a device is holding the bus low
   */
  asr_selftest_stage = 1U;

  /* DeInit rather than __HAL_I2C_DISABLE(): DeInit sets State to
   * HAL_I2C_STATE_RESET, and HAL_I2C_Init() only re-runs HAL_I2C_MspInit()
   * when State is RESET. Using the bare disable left State at READY, so the
   * later MX_I2C1_Init() skipped MspInit entirely and PB6/PB7 stayed plain
   * GPIO inputs instead of AF_OD -- the peripheral could not drive the bus
   * at all. That produced HAL_I2C_WRONG_START (0x200) on every probe. */
  HAL_I2C_DeInit(&hi2c1);

  /* 1. Discharge: drive both lines low as open-drain outputs. */
  gpio.Pin   = GPIO_PIN_6 | GPIO_PIN_7;
  gpio.Mode  = GPIO_MODE_OUTPUT_OD;
  gpio.Pull  = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &gpio);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET);

  ASR_ShortDelay(200U);   /* hold low long enough to fully discharge */

  /* 2. Release into a floating input and see whether anything recharges. */
  gpio.Pin  = GPIO_PIN_6 | GPIO_PIN_7;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &gpio);

  ASR_ShortDelay(50U);

  /* Require a majority of samples high, not merely one. A floating line can
   * pick up noise and read high on a single sample; a real pull-up holds it
   * high on every sample. Accepting any single high would let noise fake a
   * pass, which is the failure mode this whole check exists to prevent. */
  {
    uint8_t high_count = 0U;

    for (x = 0U; x < 8U; x++)
    {
      if ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET) &&
          (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET))
      {
        high_count++;
      }
      ASR_ShortDelay(20U);
    }

    if (high_count >= 6U)   /* 6 of 8 */
    {
      asr_selftest_pullup_present = 1U;
    }
  }

  /* 3. Now with the internal pull-up: only a genuinely held-low bus stays low. */
  gpio.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &gpio);

  HAL_Delay(1U);

  for (x = 0U; x < 4U; x++)
  {
    if ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) != GPIO_PIN_SET) ||
        (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) != GPIO_PIN_SET))
    {
      asr_selftest_bus_low = 1U;
    }
    HAL_Delay(1U);
  }

  MX_I2C1_Init();   /* restore AF_OD */

  /* Report the raw evidence, so the conclusion is never guessed from prose. */
  printf("Post-test 1: pull-up %s, bus %s\r\n",
         (asr_selftest_pullup_present != 0U) ? "detected" : "NOT detected",
         (asr_selftest_bus_low != 0U) ? "held LOW" : "idles high");

  if (asr_selftest_bus_low != 0U)
  {
    /* Something is actively holding the line down. Stop here: the answer
     * and the reason are both already known, and exercising I2C could not
     * add anything. */
    asr_selftest_verdict = ASR_VERDICT_BUS_STUCK;
    asr_selftest_stage = 4U;
    return;
  }

  if (asr_selftest_pullup_present == 0U)
  {
    /* The bus idles high only thanks to the MCU's internal pull-up, which
     * is far too weak to run I2C at 100 kHz. Any reads would NACK, which
     * looks identical to an absent device -- so do not run them and do not
     * report NO_DEVICE. That confusion is exactly what this check exists
     * to prevent. */
    asr_selftest_verdict = ASR_VERDICT_NO_PULLUP;
    asr_selftest_stage = 4U;
    return;
  }

  /* ---- Stage 2: bounded address probe --------------------------------- */
  asr_selftest_stage = 2U;
  {
    static const uint16_t candidates[2] = { ASR_ADDR_PRIMARY, ASR_ADDR_ALTERNATE };
    uint8_t i;

    for (i = 0U; i < 2U; i++)
    {
      for (x = 0U; x < ASR_SELFTEST_ATTEMPTS; x++)
      {
        HAL_StatusTypeDef st = HAL_I2C_Mem_Read(&hi2c1, candidates[i],
                                                (uint16_t)ASR_RESULT_REGISTER,
                                                I2C_MEMADD_SIZE_8BIT,
                                                &probe, 1U, ASR_PROBE_TIMEOUT_MS);
        asr_probe_status[i] = (uint32_t)st;

        if (st == HAL_OK)
        {
          asr_addr_probe_result[i] = 0U;
          asr_i2c_address = candidates[i];
          /* Do NOT fold the error code in on success -- a stale code left
           * over from an earlier failure would be misread as evidence about
           * this probe. */
          break;
        }

        /* Only a failed transaction contributes to the error evidence. */
        asr_probe_error[i] = HAL_I2C_GetError(&hi2c1);
        err_seen |= asr_probe_error[i];

        asr_addr_probe_result[i] = 0xFFU;
        HAL_Delay(ASR_SELFTEST_PROBE_GAP);
      }

      if (asr_addr_probe_result[i] == 0U)
      {
        break;
      }
    }
  }

  /* ---- Stage 3: bounded idle read at whatever address answered -------- */
  asr_selftest_stage = 3U;
  for (x = 0U; x < ASR_SELFTEST_ATTEMPTS; x++)
  {
    asr_selftest_read_attempts++;

    if (ASR_ReadResult(asr_i2c_address, &probe) == HAL_OK)
    {
      ok++;
      asr_selftest_read_ok++;

      if (probe != 0U)
      {
        nonzero_seen = 1U;
        if (probe > 0x09U)
        {
          invalid_code_seen = 1U;   /* valid set is 0x00-0x09 */
        }
      }
    }
    else
    {
      err_seen |= HAL_I2C_GetError(&hi2c1);
    }

    HAL_Delay(ASR_SELFTEST_READ_GAP);
  }

  asr_selftest_stage = 4U;

  /* ---- Raw evidence, printed before any interpretation ---------------
   * The previous version of this code collapsed several distinct faults
   * into one verdict and then described it in prose ("bus clocks, nobody
   * ACKs") that was not actually verified. Print the measurements first so
   * the conclusion can be checked rather than trusted.
   *
   * Error code reference (Keil's Watch window will not resolve these):
   *   HAL_I2C_ERROR_AF      = 0x04 -> NACK: bytes clocked out, nobody answered
   *   HAL_I2C_ERROR_TIMEOUT = 0x20 -> the bus never clocked at all
   *   HAL_I2C_ERROR_BERR    = 0x01 -> misplaced start/stop
   *   HAL_I2C_ERROR_ARLO    = 0x02 -> arbitration lost
   * A code of 0x00 alongside a HAL_ERROR/HAL_BUSY status is its own case:
   * the HAL returned without setting anything, which is the signature of
   * the peripheral being latched rather than of a device-level NACK. */
  /* Status legend matches HAL_StatusTypeDef (stm32f1xx_hal_def.h):
   *   0 = HAL_OK, 1 = HAL_ERROR, 2 = HAL_BUSY, 3 = HAL_TIMEOUT
   * Error code legend:
   *   0x01 BERR, 0x02 ARLO, 0x04 AF (NACK), 0x08 OVR, 0x20 TIMEOUT,
   *   0x200 WRONG_START (START never generated -- bus not being driven) */
  printf("Probe 0x68: status=%u (0=OK 1=ERR 2=BUSY 3=TIMEOUT), err=0x%03lX\r\n",
         (unsigned)asr_probe_status[0], (unsigned long)asr_probe_error[0]);
  printf("Probe 0x66: status=%u (0=OK 1=ERR 2=BUSY 3=TIMEOUT), err=0x%03lX\r\n",
         (unsigned)asr_probe_status[1], (unsigned long)asr_probe_error[1]);
  printf("Self-test: %lu/%lu reads OK, errors seen 0x%02lX\r\n",
         (unsigned long)asr_selftest_read_ok,
         (unsigned long)ASR_SELFTEST_ATTEMPTS,
         (unsigned long)err_seen);

  /* ---- Verdict --------------------------------------------------------
   * Order matters: the bus-health conclusions must be reachable before the
   * device-level ones, because a bus fault makes every device-level
   * observation meaningless. */
  if ((ok >= ASR_SELFTEST_MIN_OK) && (nonzero_seen == 0U))
  {
    asr_selftest_verdict = ASR_VERDICT_PASS;
  }
  else if ((asr_addr_probe_result[0] == 0U) || (asr_addr_probe_result[1] == 0U))
  {
    /* An address answered, so the bus demonstrably works and the module is
     * the thing that needs attention. */
    asr_selftest_verdict = ASR_VERDICT_DEVICE_SILENT;
  }
  else if (err_seen == 0U)
  {
    /* Nothing ACKed AND the HAL never set an error code. Falling through to
     * NO_DEVICE here would repeat the original mistake: claiming the bus
     * clocked when there is no evidence that it did. Report a bus fault. */
    asr_selftest_verdict = ASR_VERDICT_BUS_STUCK;
  }
  else if ((err_seen & HAL_I2C_ERROR_TIMEOUT) != 0U)
  {
    /* The bus never clocked at all -- a held line or a clocking fault. */
    asr_selftest_verdict = ASR_VERDICT_BUS_STUCK;
  }
  else
  {
    /* NACKs with no timeout: the bus clocked bytes out, so it is
     * electrically sound, and nothing answered at either address. */
    asr_selftest_verdict = ASR_VERDICT_NO_DEVICE;
  }

  /* 0xFF is outside the valid code set, which means the slave was not
   * driving SDA during the read phase -- a different fault from a NACK. */
  if ((asr_selftest_verdict == ASR_VERDICT_DEVICE_SILENT) && (invalid_code_seen != 0U))
  {
    printf("  note: read values outside the valid set 0x00-0x09; slave is\r\n");
    printf("        probably not driving SDA (check module power/init).\r\n");
  }
}

static const char *ASR_VerdictName(uint8_t verdict)
{
  switch (verdict)
  {
    case ASR_VERDICT_PASS:          return "PASS - bus alive, module quiet";
    case ASR_VERDICT_NO_DEVICE:     return "NO_DEVICE - bus clocked, nobody ACKed";
    case ASR_VERDICT_DEVICE_SILENT: return "DEVICE_SILENT - ACKs but no valid code";
    case ASR_VERDICT_BUS_STUCK:     return "BUS_STUCK - line held low or no clock";
    case ASR_VERDICT_NO_PULLUP:     return "NO_PULLUP - fit an external pull-up";
    default:                        return "PENDING";
  }
}

#if ASR_USE_HEARTBEAT_LED
/* --------------------------------------------------------------------------
 * LED0 (red) on PA8 -- NOT PC13, which is touch CS on this board.
 * Non-blocking: driven from HAL_GetTick(), never HAL_Delay, so it cannot
 * spoil the poll schedule.
 *
 *   1 Hz slow  = no device on the bus
 *   double     = device present but every read is 0x00 (bus alive, module silent)
 *   single     = one command just received
 *   solid on   = bus dead / recovery failing
 * ------------------------------------------------------------------------ */
static void ASR_LedInit(void)
{
  GPIO_InitTypeDef gpio = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  gpio.Pin   = GPIO_PIN_8;
  gpio.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio.Pull  = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &gpio);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);   /* LED off (active low) */
}

static void ASR_LedService(void)
{
  /* Clean state machine: one static for the next timer deadline, one for
   * the LED phase (on/off), one for how many blinks remain in the current
   * group, and one for the group's trailing gap. Non-blocking throughout --
   * the caller runs this every loop iteration. */
  static uint32_t next_tick    = 0U;
  static uint32_t last_seen    = 0U;
  static uint8_t  led_on       = 0U;
  static uint8_t  blinks_left  = 0U;
  static uint8_t  in_gap       = 0U;
  uint32_t now = HAL_GetTick();

  if (now < next_tick)
  {
    return;
  }

  /* A received command briefly overrides the verdict pattern, so there is
   * instantaneous feedback while speaking. Watch asr_command_count rather
   * than consuming asr_new_command -- that flag is a consume-once handshake
   * reserved for the actuator layer, and the LED must not eat it. */
  if (asr_command_count != last_seen)
  {
    last_seen = asr_command_count;
    led_on = 1U;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);   /* on, active low */
    next_tick = now + 40U;
    /* Reset the pattern so it restarts cleanly afterwards. */
    blinks_left = 0U;
    in_gap = 1U;
    return;
  }

  /* The verdict drives the light, so LED and UART cannot disagree. The
   * earlier version lit "bus dead" from a read failure while the UART
   * claimed the bus was clocking fine.
   *
   *   verdict+1 slow blinks, then a 2 s gap, repeating:
   *     1 blink  = PASS           (bus alive, module quiet)
   *     2 blinks = NO_DEVICE      (bus clocked, nothing answered)
   *     3 blinks = DEVICE_SILENT  (answered, but no valid code)
   *     4 blinks = BUS_STUCK      (line held low / never clocked)
   *     5 blinks = NO_PULLUP      (fit an external pull-up)
   */
  if (in_gap != 0U)
  {
    led_on = 0U;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);      /* off */
    in_gap = 0U;
    next_tick = now + 2000U;    /* long gap: makes groups countable */
    return;
  }

  if (blinks_left == 0U)
  {
    if (asr_selftest_verdict == ASR_VERDICT_PENDING)
    {
      /* Self-test still running: stay dark rather than show a stale code. */
      led_on = 0U;
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
      next_tick = now + 200U;
      return;
    }

    blinks_left = (uint8_t)(asr_selftest_verdict + 1U);
    if (blinks_left > 5U)
    {
      blinks_left = 5U;
    }
  }

  if (led_on == 0U)
  {
    led_on = 1U;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);    /* on  */
    next_tick = now + 120U;
  }
  else
  {
    led_on = 0U;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);      /* off */
    next_tick = now + 180U;

    if (blinks_left > 0U)
    {
      blinks_left--;
    }
    if (blinks_left == 0U)
    {
      in_gap = 1U;   /* group finished; take the long gap next time */
    }
  }
}
#endif /* ASR_USE_HEARTBEAT_LED */

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
