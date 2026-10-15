/* ---------------------------------------------------------------------------
 * led.c -- heartbeat LED service.
 *
 * The LED is on PA8 on the current board, red, ACTIVE LOW -- driven low to
 * light. Both the pin and the polarity come from board_config.h, so a board
 * with an active-high LED needs no change here.
 *
 * Note PC13 is NOT an LED on this board: it is the touch panel chip-select, and
 * driving it lights nothing while breaking the panel.
 * -------------------------------------------------------------------------*/

#include "led.h"
#include "mcp_diag.h"
#include "stm32f1xx_hal.h"

#if BOARD_USE_HEARTBEAT_LED

/* Blink periods. 2 Hz until the module answers, 1 Hz once it does -- a visible
 * difference between "not there" and "there but quiet". */
#define LED_BLINK_SLOW_MS   500U
#define LED_BLINK_FAST_MS   250U
#define LED_SOLID_HOLD_MS   100U

static uint32_t s_next_tick    = 0U;
static uint8_t  s_on           = 0U;
static uint8_t  s_probe_ok     = 0U;
static uint32_t s_last_frame   = 0U;

/* Write the pin for a logical on/off, honouring the board's polarity. */
static void led_write(uint8_t on)
{
#if BOARD_LED_ACTIVE_LOW
  HAL_GPIO_WritePin(BOARD_LED_PORT, BOARD_LED_PIN, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
#else
  HAL_GPIO_WritePin(BOARD_LED_PORT, BOARD_LED_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
}

void MCP_LedInit(void)
{
  GPIO_InitTypeDef gpio = {0};

  BOARD_LED_GPIO_CLK();

  gpio.Pin   = BOARD_LED_PIN;
  gpio.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio.Pull  = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BOARD_LED_PORT, &gpio);

  led_write(0U);   /* off */
}

void MCP_LedNoteFrame(uint32_t now_ms)
{
  s_last_frame = now_ms;
}

void MCP_LedNoteProbeOk(void)
{
  s_probe_ok = 1U;
}

void MCP_LedService(void)
{
  uint32_t now = HAL_GetTick();

  if (now < s_next_tick)
  {
    return;
  }

  /* Solid only while frames are genuinely arriving. Keying this off mcp_state
   * alone made the LED latch on after the very first frame and stay on
   * forever, which reads as "still receiving" when nothing is -- a misleading
   * signal at exactly the moment it matters most. */
  if ((s_last_frame != 0U) && ((now - s_last_frame) < MCP_ACTIVITY_WINDOW_MS))
  {
    led_write(1U);
    s_next_tick = now + LED_SOLID_HOLD_MS;
    return;
  }

  s_on = (uint8_t)(s_on ^ 1U);
  led_write(s_on);
  s_next_tick = now + (s_probe_ok ? LED_BLINK_SLOW_MS : LED_BLINK_FAST_MS);
}

#endif /* BOARD_USE_HEARTBEAT_LED */
