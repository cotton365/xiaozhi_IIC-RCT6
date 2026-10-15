/* ---------------------------------------------------------------------------
 * board_config.h -- THE ONLY PLACE PIN ASSIGNMENTS ARE WRITTEN DOWN.
 *
 * Porting this firmware to another STM32F103RCT6 board should mean editing
 * this file and nothing else. Everything a board can differ in lives here:
 * I2C pins, UART pins, the LED, and the two feature switches.
 *
 * ---------------------------------------------------------------------------
 * WHY THE PIN GROUPS INCLUDE A REMAP MACRO
 *
 * I2C1 on the F1 cannot use an arbitrary pin pair. It is either
 *     PB6/PB7  (no remap)          or   PB8/PB9  (AFIO_MAPR I2C1_REMAP = 1)
 * and nothing else. The remap bit is not an optional extra that happens to
 * accompany a pin change -- it IS part of the pin change. Splitting the two
 * across separate defines is how you end up moving SCL/SDA, forgetting the
 * remap, and watching the bus sit there completely dead with no error to
 * explain why. So each group below carries its pin pair AND its remap call
 * together, and you select the whole group with one line.
 *
 * USART1 is the same story: PA9/PA10 with no remap, or PB6/PB7 remapped. Both
 * options are spelled out even though the alternate pins collide with the
 * default I2C1 pins -- that collision is real and worth seeing, not hiding.
 * -------------------------------------------------------------------------*/

#ifndef __BOARD_CONFIG_H__
#define __BOARD_CONFIG_H__

#include "stm32f1xx_hal.h"

/* ==========================================================================
 * 1. BOARD SELECTION -- uncomment exactly one of these two lines.
 * ======================================================================== */

/* Current board (ALIENTEK MiniSTM32 V4): PB6 = SCL, PB7 = SDA, no remap. */
#define BOARD_I2C1_PINS_STD       1
/* Alternate wiring: PB8 = SCL, PB9 = SDA, requires the AFIO remap bit. */
/* #define BOARD_I2C1_PINS_REMAP */

#if !defined(BOARD_I2C1_PINS_STD) && !defined(BOARD_I2C1_PINS_REMAP)
#error "board_config.h: select an I2C1 pin group (BOARD_I2C1_PINS_STD or BOARD_I2C1_PINS_REMAP)"
#endif
#if defined(BOARD_I2C1_PINS_STD) && defined(BOARD_I2C1_PINS_REMAP)
#error "board_config.h: both I2C1 pin groups selected -- uncomment only one"
#endif

/* ==========================================================================
 * 2. I2C1 -- the bus to the WonderLLM module.  Hardware peripheral, blocking.
 * ======================================================================== */

#if defined(BOARD_I2C1_PINS_STD)
  #define BOARD_I2C_SCL_PORT      GPIOB
  #define BOARD_I2C_SCL_PIN       GPIO_PIN_6
  #define BOARD_I2C_SDA_PORT      GPIOB
  #define BOARD_I2C_SDA_PIN       GPIO_PIN_7
  #define BOARD_I2C_GPIO_CLK()    __HAL_RCC_GPIOB_CLK_ENABLE()
  #define BOARD_I2C_REMAP_CALL()  do { } while (0)
#endif

#if defined(BOARD_I2C1_PINS_REMAP)
  #define BOARD_I2C_SCL_PORT      GPIOB
  #define BOARD_I2C_SCL_PIN       GPIO_PIN_8
  #define BOARD_I2C_SDA_PORT      GPIOB
  #define BOARD_I2C_SDA_PIN       GPIO_PIN_9
  #define BOARD_I2C_GPIO_CLK()    __HAL_RCC_GPIOB_CLK_ENABLE()
  #define BOARD_I2C_REMAP_CALL()  __HAL_AFIO_REMAP_I2C1_ENABLE()
#endif

/* Both groups above happen to sit on GPIOB. If a future board splits SCL and
 * SDA across two ports -- which the F1's pin tables do not allow for I2C1, but
 * which a software-I2C port might -- the two clock enables below are separate
 * so only one needs changing. The AFIO clock is enabled by HAL_MspInit(). */
#define BOARD_I2C_SCL_CLK()       BOARD_I2C_GPIO_CLK()
#define BOARD_I2C_SDA_CLK()       BOARD_I2C_GPIO_CLK()

/* ==========================================================================
 * 3. DEBUG UART (USART1) -- the only way to see inside a running board.
 * ======================================================================== */

/* USART1's two physical options. PA9/PA10 is the current board. */
#define BOARD_UART1_PINS_STD      1
/* #define BOARD_UART1_PINS_REMAP */   /* PB6 = TX, PB7 = RX */

#if defined(BOARD_UART1_PINS_STD)
  #define BOARD_UART_TX_PORT      GPIOA
  #define BOARD_UART_TX_PIN       GPIO_PIN_9
  #define BOARD_UART_RX_PORT      GPIOA
  #define BOARD_UART_RX_PIN       GPIO_PIN_10
  #define BOARD_UART_GPIO_CLK()   __HAL_RCC_GPIOA_CLK_ENABLE()
  #define BOARD_UART_REMAP_CALL() do { } while (0)
#endif

#if defined(BOARD_UART1_PINS_REMAP)
  #define BOARD_UART_TX_PORT      GPIOB
  #define BOARD_UART_TX_PIN       GPIO_PIN_6
  #define BOARD_UART_RX_PORT      GPIOB
  #define BOARD_UART_RX_PIN       GPIO_PIN_7
  #define BOARD_UART_GPIO_CLK()   __HAL_RCC_GPIOB_CLK_ENABLE()
  #define BOARD_UART_REMAP_CALL() __HAL_AFIO_REMAP_USART1_ENABLE()
#endif

/* Console baud. 115200 8N1 matches the onboard CH340 and needs no USB
 * re-enumeration; changing it needs no other edit. */
#define BOARD_UART_BAUD           115200U

/* ==========================================================================
 * 4. HEARTBEAT LED
 *
 * PA8 (red, ACTIVE LOW) on the current board. It is deliberately NOT PC13 --
 * on this board PC13 is the touch panel chip-select, and driving it lights
 * nothing while breaking the touch panel.
 * ======================================================================== */

#define BOARD_LED_PORT            GPIOA
#define BOARD_LED_PIN             GPIO_PIN_8
#define BOARD_LED_GPIO_CLK()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define BOARD_LED_ACTIVE_LOW      1     /* 1 = drive pin LOW to light the LED */

/* ==========================================================================
 * 5. FEATURE SWITCHES
 * ======================================================================== */

/* Debug logging over USART1. Set to 0 for the final build: the logging code
 * drops out of the build entirely, which is how the serial port is meant to
 * disappear once bring-up is over.
 *
 * Note this only removes the LOGGING. The UART itself is still initialised by
 * MX_USART1_UART_Init(), so leaving the peripheral configured costs nothing
 * and keeps `#define`-ing this back to 1 a one-line operation. */
#define BOARD_DEBUG_UART          1

/* Heartbeat LED service. */
#define BOARD_USE_HEARTBEAT_LED   1

/* Size of the non-blocking log ring buffer, in bytes. A full line is dropped
 * rather than allowed to block: losing log bytes is survivable, stalling the
 * I2C bus while the module is mid-frame is not. g_uart_dropped counts what was
 * lost, so a truncated log is visible instead of silent. */
#define BOARD_LOG_RING_SIZE       512

/* ==========================================================================
 * 6. WONDERLLM MCP PROTOCOL CONSTANTS
 *
 * These are protocol facts, not board facts -- they stay the same whatever
 * board this runs on. Collected here so the pin section above stays readable.
 * ======================================================================== */

/* The ESP32-S3 answers at 7-bit 0x55. NOTE: reads are PURE reads. There is no
 * register address and nothing may be written before a read -- sending 0x64
 * first was the CI1302 register protocol and it does not exist on this bus. */
#define MCP_ADDR_7BIT             0x55U
#define MCP_ADDR_HAL_W            ((uint16_t)(MCP_ADDR_7BIT << 1))         /* 0xAA */
#define MCP_ADDR_HAL_R            ((uint16_t)((MCP_ADDR_7BIT << 1) | 1U))  /* 0xAB */

#define MCP_HDR_LEN               8U
#define MCP_DATA_LEN_MAX          31U      /* vendor cap: 1..31 */
#define MCP_MAGIC_0               0xAAU
#define MCP_MAGIC_1               0x55U

#define MCP_TX_TIMEOUT_MS         100U
#define MCP_RX_TIMEOUT_MS         20U

#define MCP_POLL_INTERVAL_MS      100U
#define MCP_REG_RETRY_MS          200U     /* re-send registration while no JSON yet */
#define MCP_STATUS_INTERVAL_MS    5000U    /* periodic counter summary */

/* A frame counts as "the model answered" iff its payload starts like a JSON
 * object. Registration retries until that happens. */
#define MCP_JSON_PREFIX           '{'

/* The LED shows live activity, not history: solid only while frames arrive. */
#define MCP_ACTIVITY_WINDOW_MS    2000U

/* !! The vendor requires 400 kHz WHILE TRANSMITTING MCP JSON:
 *      "For longer strings (especially Chinese) you MUST raise the I2C rate
 *       to 400,000, otherwise WonderLLM cannot receive the data completely."
 *    Reads are a handful of bytes and are fine at 100 kHz. */
#define MCP_I2C_SPEED_WRITE       400000U
#define MCP_I2C_SPEED_READ        100000U

/* Raw receive buffer. Static, not on the stack -- fragmented frames can run
 * past 64 bytes. */
#define MCP_RX_BUF_LEN            256U

/* Vendor-mandated inter-step settling times in the receive path. These mirror
 * receive_frame() in the vendor host, which waits before each part. */
#define MCP_PART_SETTLE_MS        10U
#define MCP_FRAGMENT_GAP_MS       100U

/* Consecutive genuine bus faults before attempting recovery. NACK does not
 * count -- an idle slave simply does not answer, which is normal. */
#define MCP_BUS_FAULT_LIMIT       20U

#endif /* __BOARD_CONFIG_H__ */
