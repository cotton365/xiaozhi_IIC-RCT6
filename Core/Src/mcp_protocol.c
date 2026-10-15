/* ---------------------------------------------------------------------------
 * mcp_protocol.c -- receive one MCP frame.
 *
 * MIRRORS THE VENDOR'S receive_frame() STEP FOR STEP:
 *   - read the 8-byte header, check the AA 55 magic
 *   - data_len is big endian; part_id and part_num are little endian
 *   - part_id must be 1, otherwise frames were dropped before this point and
 *     the remainder would be a truncated JSON document -- abandon rather than
 *     deliver something half-parsed
 *   - per part: wait, read data_len+1 bytes, last byte is the XOR
 *   - between parts: wait, then read the NEXT header
 *
 * The waits are vendor-mandated settling times, not incidental sleeps. They are
 * named in board_config.h so their purpose survives the next reader.
 * -------------------------------------------------------------------------*/

#include "mcp_protocol.h"
#include "mcp_transport.h"
#include "mcp_diag.h"
#include <string.h>

static uint32_t s_last_hal_error = 0U;
static uint32_t s_last_hal_status = (uint32_t)HAL_OK;

void MCP_Rx_Init(mcp_rx_t *rx)
{
  if (rx == 0)
  {
    return;
  }
  memset(rx, 0, sizeof(*rx));
  s_last_hal_error = 0U;
  s_last_hal_status = (uint32_t)HAL_OK;
}

uint32_t MCP_Rx_LastHalError(void)
{
  return s_last_hal_error;
}

/* The raw HAL status of the last transfer, mirroring what the pre-refactor
 * code stored in mcp_last_status. Kept because it is the difference between
 * "the transfer completed but the data was wrong" (HAL_OK) and "the transfer
 * itself failed" -- a distinction the result enum alone does not carry. */
uint32_t MCP_Rx_LastHalStatus(void)
{
  return s_last_hal_status;
}

/* --------------------------------------------------------------------------
 * Read and validate a header into rx. Returns the result; on success rx->data_len
 * is populated and the raw bytes are kept in rx->header for the log.
 *
 * Every rejection is counted, and the count is what tells a reader whether the
 * frames are malformed or simply absent -- two situations that look identical
 * on a console without them.
 * ------------------------------------------------------------------------ */
static mcp_rx_result_t read_header(mcp_rx_t *rx, I2C_HandleTypeDef *hi2c)
{
  HAL_StatusTypeDef st;

  st = MCP_ReadBytes(hi2c, rx->header, MCP_HDR_LEN);
  if (st != HAL_OK)
  {
    s_last_hal_status = (uint32_t)st;
    s_last_hal_error = HAL_I2C_GetError(hi2c);
    return ((s_last_hal_error & (uint32_t)HAL_I2C_ERROR_AF) != 0U)
             ? MCP_RX_NOTHING      /* idle slave: normal, not a fault */
             : MCP_RX_FAULT;
  }
  s_last_hal_status = (uint32_t)HAL_OK;

  memcpy((void *)mcp_last_header, rx->header, MCP_HDR_LEN);

  if ((rx->header[0] != MCP_MAGIC_0) || (rx->header[1] != MCP_MAGIC_1))
  {
    mcp_bad_header++;
    return MCP_RX_BAD_HEADER;
  }

  rx->data_len = (uint16_t)(((uint16_t)rx->header[2] << 8) | (uint16_t)rx->header[3]);

  if ((rx->data_len == 0U) || (rx->data_len > MCP_DATA_LEN_MAX))
  {
    mcp_bad_header++;
    return MCP_RX_BAD_HEADER;
  }

  return MCP_RX_OK;
}

/* --------------------------------------------------------------------------
 * Read one frame, blocking, start to finish.
 * ------------------------------------------------------------------------ */
mcp_rx_result_t MCP_Rx_Start(mcp_rx_t *rx, I2C_HandleTypeDef *hi2c,
                             uint8_t *out, uint16_t out_cap, uint16_t *out_len)
{
  uint8_t  part[MCP_DATA_LEN_MAX + 1U];
  uint16_t part_id;
  uint16_t part_num;
  uint16_t i;
  uint16_t k;
  uint8_t  sum;
  HAL_StatusTypeDef st;
  mcp_rx_result_t res;

  if ((rx == 0) || (hi2c == 0) || (out == 0) || (out_len == 0))
  {
    return MCP_RX_FAULT;
  }

  rx->total       = 0U;
  rx->part_index  = 0U;
  rx->part_total  = 0U;
  rx->started_tick = HAL_GetTick();

  /* ---- header 1 ---- */
  res = read_header(rx, hi2c);
  if (res != MCP_RX_OK)
  {
    return res;
  }

  part_id  = (uint16_t)(((uint16_t)rx->header[5] << 8) | (uint16_t)rx->header[4]);  /* little */
  part_num = (uint16_t)(((uint16_t)rx->header[7] << 8) | (uint16_t)rx->header[6]);  /* little */

  /* Not the first fragment means data was lost before this point. */
  if ((part_id != 1U) || (part_num == 0U))
  {
    mcp_bad_header++;
    return MCP_RX_BAD_HEADER;
  }

  rx->part_total = part_num;

  for (i = 1U; i <= part_num; i++)
  {
    HAL_Delay(MCP_PART_SETTLE_MS);

    st = MCP_ReadBytes(hi2c, part, (uint16_t)(rx->data_len + 1U));
    if (st != HAL_OK)
    {
      s_last_hal_status = (uint32_t)st;
      s_last_hal_error = HAL_I2C_GetError(hi2c);
      return MCP_RX_FAULT;
    }

    sum = 0U;
    for (k = 0U; k < rx->data_len; k++)
    {
      sum = (uint8_t)(sum ^ part[k]);
    }

    if (part[rx->data_len] != sum)
    {
      mcp_bad_checksum++;
      return MCP_RX_BAD_CHECKSUM;
    }

    /* A frame longer than the caller's buffer is truncated rather than
     * overflowed. The checksum above still has to pass, so a short buffer
     * cannot deliver unverified bytes. */
    if ((uint16_t)(rx->total + rx->data_len) <= out_cap)
    {
      memcpy(out + rx->total, part, rx->data_len);
      rx->total = (uint16_t)(rx->total + rx->data_len);
    }

    rx->part_index = i;

    /* ---- next header, when the message is fragmented ---- */
    if (i < part_num)
    {
      uint16_t next_len;

      HAL_Delay(MCP_FRAGMENT_GAP_MS);

      st = MCP_ReadBytes(hi2c, rx->header, MCP_HDR_LEN);
      if (st != HAL_OK)
      {
        s_last_hal_status = (uint32_t)st;
        s_last_hal_error = HAL_I2C_GetError(hi2c);
        return MCP_RX_FAULT;
      }

      if ((rx->header[0] != MCP_MAGIC_0) || (rx->header[1] != MCP_MAGIC_1))
      {
        mcp_bad_header++;
        return MCP_RX_BAD_HEADER;
      }

      next_len = (uint16_t)(((uint16_t)rx->header[2] << 8) | (uint16_t)rx->header[3]);
      if ((next_len == 0U) || (next_len > MCP_DATA_LEN_MAX))
      {
        mcp_bad_header++;
        return MCP_RX_BAD_HEADER;
      }

      /* the vendor keeps every part of a message the same length */
      rx->data_len = next_len;

      memcpy((void *)mcp_last_header, rx->header, MCP_HDR_LEN);
    }
  }

  out[rx->total] = 0U;   /* NUL-terminate so strstr() is safe on a JSON payload */
  *out_len = rx->total;
  return MCP_RX_OK;
}
