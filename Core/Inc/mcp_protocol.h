/* ---------------------------------------------------------------------------
 * mcp_protocol.h -- reading one complete MCP frame off the bus.
 *
 * ---------------------------------------------------------------------------
 * FRAME LAYOUT (byte order verified against the vendor's receive_frame_head())
 *
 *     [0]     0xAA              magic
 *     [1]     0x55              magic
 *     [2..3]  data_len          BIG endian, 1..31
 *     [4..5]  part_id           LITTLE endian
 *     [6..7]  part_num          LITTLE endian
 *
 * then, per part:  data_len payload bytes followed by 1 XOR checksum byte over
 * those bytes. A message longer than 31 bytes arrives as several parts, each
 * with its own header and checksum.
 *
 * Note the mixed byte order. That is not a typo in this file -- data_len is big
 * endian and the part fields are little endian, confirmed character-by-character
 * against the vendor implementation.
 *
 * ---------------------------------------------------------------------------
 * WHY THE STATE IS A STRUCT
 *
 * The receive path is inherently multi-step: read header, wait, read part,
 * check checksum, maybe wait longer and read the next header. Right now those
 * steps run back to back with HAL_Delay() between them, which holds the CPU for
 * up to ~250 ms per frame.
 *
 * Keeping the partial frame in a caller-owned struct means the steps can later
 * be driven one at a time from the main loop, spread across tick boundaries,
 * without this module having to own any global state or know about scheduling.
 * MCP_Rx_Start() still runs the whole sequence in one go for now.
 * -------------------------------------------------------------------------*/

#ifndef __MCP_PROTOCOL_H__
#define __MCP_PROTOCOL_H__

#include "board_config.h"
#include "stm32f1xx_hal.h"

/* Working state for one frame in flight. The caller owns it, so two readers
 * could in principle exist without sharing hidden state. */
typedef struct
{
  uint8_t  header[MCP_HDR_LEN];        /* most recent header seen          */
  uint16_t data_len;                   /* payload bytes per part           */
  uint16_t part_index;                 /* how many parts collected so far  */
  uint16_t part_total;                 /* parts expected                   */
  uint16_t total;                      /* payload bytes collected          */
  uint32_t started_tick;               /* for timeout accounting later     */
} mcp_rx_t;

/* Result of one receive attempt. HAL_ERROR alone cannot say WHY, and the
 * difference matters: a NACK while idle is normal, a checksum failure is not. */
typedef enum
{
  MCP_RX_OK = 0,          /* a complete, verified frame was assembled */
  MCP_RX_NOTHING,         /* the slave did not answer -- normal idle   */
  MCP_RX_BAD_HEADER,      /* magic, length or part numbering invalid    */
  MCP_RX_BAD_CHECKSUM,    /* XOR did not match                          */
  MCP_RX_FAULT            /* genuine bus fault: BUSY or TIMEOUT         */
} mcp_rx_result_t;

/* Reset the working state. Call once before the first use. */
void MCP_Rx_Init(mcp_rx_t *rx);

/* Try to receive one complete frame.
 *
 * `out` receives the reassembled payload and is always NUL-terminated when the
 * result is MCP_RX_OK, so strstr() over it is safe. `out_cap` must leave room
 * for that terminator.
 *
 * Callers must distinguish the results: MCP_RX_NOTHING is the ordinary state of
 * an idle bus and is NOT an error. Treating it as one is what previously caused
 * a bus recovery every two seconds while nothing was happening. */
mcp_rx_result_t MCP_Rx_Start(mcp_rx_t *rx, I2C_HandleTypeDef *hi2c,
                             uint8_t *out, uint16_t out_cap, uint16_t *out_len);

/* Map a result to the HAL error code the diagnostics report, so the log keeps
 * showing the underlying reason rather than just "failed". */
uint32_t MCP_Rx_LastHalError(void);

/* The raw HAL_StatusTypeDef of the last transfer. HAL_OK means the transfer
 * completed but the frame was rejected on its contents; anything else means the
 * transfer itself failed. */
uint32_t MCP_Rx_LastHalStatus(void);

#endif /* __MCP_PROTOCOL_H__ */
