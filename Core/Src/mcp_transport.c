/* ---------------------------------------------------------------------------
 * mcp_transport.c -- I2C bus layer.
 *
 * Blocking transfers throughout, by design. The tradeoff is deliberate: at
 * 100 kHz an 8-byte header takes ~0.7 ms and a 32-byte fragment ~2.8 ms, which
 * is small enough not to matter, and blocking is what the F1 HAL's blocking
 * API is actually good at. Moving to the _IT API would not automatically fix
 * anything -- the I2C event interrupt preempts the CPU mid-transfer, so if the
 * application's other work ever masks interrupts for more than a few
 * microseconds it would corrupt bus timing in a new and harder-to-diagnose way.
 * -------------------------------------------------------------------------*/

#include "mcp_transport.h"
#include "mcp_diag.h"
#include "i2c.h"

/* --------------------------------------------------------------------------
 * Reprogram the bus speed at runtime.
 *
 * The vendor requires 400 kHz while sending MCP JSON and 100 kHz is fine for
 * the short reads, so this changes under program control rather than once at
 * init.
 * ------------------------------------------------------------------------ */
HAL_StatusTypeDef MCP_SetSpeed(I2C_HandleTypeDef *hi2c, uint32_t speed)
{
  if (hi2c == 0)
  {
    return HAL_ERROR;
  }

  if (mcp_i2c_speed_hz == speed)
  {
    return HAL_OK;
  }

  hi2c->Init.ClockSpeed = speed;
  if (HAL_I2C_Init(hi2c) != HAL_OK)
  {
    return HAL_ERROR;
  }

  mcp_i2c_speed_hz = speed;
  return HAL_OK;
}

/* --------------------------------------------------------------------------
 * Address-only ACK test. This is what a bus scan does, and it is the cheapest
 * way to answer "is the module there at all" without touching the frame
 * protocol.
 * ------------------------------------------------------------------------ */
HAL_StatusTypeDef MCP_Probe(I2C_HandleTypeDef *hi2c)
{
  if (MCP_SetSpeed(hi2c, MCP_I2C_SPEED_READ) != HAL_OK)
  {
    return HAL_ERROR;
  }
  return HAL_I2C_IsDeviceReady(hi2c, MCP_ADDR_HAL_W, 2U, 20U);
}

/* --------------------------------------------------------------------------
 * One transaction, no chunking. See the header for why the absence of
 * chunking is a correctness requirement rather than a simplification.
 * ------------------------------------------------------------------------ */
HAL_StatusTypeDef MCP_SendBytes(I2C_HandleTypeDef *hi2c, const uint8_t *data, uint16_t len)
{
  HAL_StatusTypeDef st;

  /* 400 kHz is mandatory for MCP JSON. */
  if (MCP_SetSpeed(hi2c, MCP_I2C_SPEED_WRITE) != HAL_OK)
  {
    return HAL_ERROR;
  }

  st = HAL_I2C_Master_Transmit(hi2c, MCP_ADDR_HAL_W,
                               (uint8_t *)data, len, MCP_TX_TIMEOUT_MS);

  if (st == HAL_OK)
  {
    mcp_tx_total_bytes += (uint32_t)len;
  }

  return st;
}

/* --------------------------------------------------------------------------
 * Pure read. Nothing is written before it -- there is no register address on
 * this bus, and sending one (0x64, the CI1302 register protocol) is exactly
 * the mistake that made this link appear dead for a whole session.
 * ------------------------------------------------------------------------ */
HAL_StatusTypeDef MCP_ReadBytes(I2C_HandleTypeDef *hi2c, uint8_t *out, uint16_t len)
{
  if (MCP_SetSpeed(hi2c, MCP_I2C_SPEED_READ) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_I2C_Master_Receive(hi2c, MCP_ADDR_HAL_R, out, len, MCP_RX_TIMEOUT_MS);
}

/* --------------------------------------------------------------------------
 * Bus recovery. Nine SCL pulses, then a manual STOP.
 *
 * WHY DeInit AND NOT __HAL_I2C_DISABLE():
 *   HAL_I2C_Init() only calls HAL_I2C_MspInit() when State == HAL_I2C_STATE_RESET.
 *   __HAL_I2C_DISABLE() clears the PE bit but leaves State at READY, so the
 *   re-init below would skip MspInit and the pins would stay in the plain-GPIO
 *   mode this function just put them in -- leaving the bus undrivable and
 *   producing HAL_I2C_WRONG_START (0x200) on every subsequent transfer.
 *   HAL_I2C_DeInit() sets State to RESET, so MX_I2C1_Init() genuinely re-runs
 *   MspInit and restores AF_OD. This specific mistake cost a full debugging
 *   session once; do not undo it.
 *
 * Pins come from board_config.h. The SCL/SDA roles are asymmetric on purpose:
 * SCL is an open-drain output (we only ever pull it low, the pull-up makes the
 * high level), while SDA is an input while we wait for the slave to release it.
 * ------------------------------------------------------------------------ */
HAL_StatusTypeDef MCP_BusRecover(I2C_HandleTypeDef *hi2c)
{
  GPIO_InitTypeDef gpio = {0};
  uint8_t i;
  uint8_t released = 0U;

  HAL_I2C_DeInit(hi2c);

  /* SCL as open-drain output so we can pulse it; SDA as an input with a pull-up
   * so we can watch for the slave letting go. */
  gpio.Pin   = BOARD_I2C_SCL_PIN;
  gpio.Mode  = GPIO_MODE_OUTPUT_OD;
  gpio.Pull  = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(BOARD_I2C_SCL_PORT, &gpio);

  gpio.Pin  = BOARD_I2C_SDA_PIN;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BOARD_I2C_SDA_PORT, &gpio);

  for (i = 0U; (i < 9U) && (released == 0U); i++)
  {
    HAL_GPIO_WritePin(BOARD_I2C_SCL_PORT, BOARD_I2C_SCL_PIN, GPIO_PIN_RESET);  /* SCL low  */
    HAL_Delay(1U);
    HAL_GPIO_WritePin(BOARD_I2C_SCL_PORT, BOARD_I2C_SCL_PIN, GPIO_PIN_SET);    /* SCL high */
    HAL_Delay(1U);

    if (HAL_GPIO_ReadPin(BOARD_I2C_SDA_PORT, BOARD_I2C_SDA_PIN) == GPIO_PIN_SET)
    {
      released = 1U;
    }
  }

  /* Manual STOP: SDA low while SCL is high, then release SDA. */
  gpio.Pin   = BOARD_I2C_SDA_PIN;
  gpio.Mode  = GPIO_MODE_OUTPUT_OD;
  gpio.Pull  = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(BOARD_I2C_SDA_PORT, &gpio);

  HAL_GPIO_WritePin(BOARD_I2C_SDA_PORT, BOARD_I2C_SDA_PIN, GPIO_PIN_RESET);  /* SDA low  */
  HAL_Delay(1U);
  HAL_GPIO_WritePin(BOARD_I2C_SCL_PORT, BOARD_I2C_SCL_PIN, GPIO_PIN_SET);    /* SCL high */
  HAL_Delay(1U);
  HAL_GPIO_WritePin(BOARD_I2C_SDA_PORT, BOARD_I2C_SDA_PIN, GPIO_PIN_SET);    /* SDA high */
  HAL_Delay(1U);

  MX_I2C1_Init();                        /* restores AF_OD + 100 kHz */
  mcp_i2c_speed_hz = MCP_I2C_SPEED_READ;

  return (released != 0U) ? HAL_OK : HAL_ERROR;
}
