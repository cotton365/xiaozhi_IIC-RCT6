/* ---------------------------------------------------------------------------
 * mcp_diag.h -- observation state and logging.
 *
 * Two things live here, both deliberately kept in one place:
 *
 *   1. The counters that describe what the link is doing. They are the only
 *      window into a board you cannot attach a debugger to, so they are
 *      `volatile` and they are global by design -- several modules increment
 *      them, and the whole point is that a debugger can read them at any time.
 *      (The project builds at -O3; a non-volatile file-scope counter can be
 *      kept in a register and simply not exist in memory to inspect.)
 *
 *   2. The log macros. Every one of them compiles to nothing when
 *      BOARD_DEBUG_UART is 0, so removing the serial port for the final build
 *      is a one-line change in board_config.h rather than an edit across the
 *      whole codebase.
 *
 * LOGGING DOES NOT COUNT AS LOGGING ONLY. A frame dump costs real time on the
 * bus-adjacent path, and at 115200 baud each byte is ~87 us of held CPU. The
 * ring buffer in retarget_io.c absorbs that so a log line can never stall the
 * caller -- see the note there.
 * -------------------------------------------------------------------------*/

#ifndef __MCP_DIAG_H__
#define __MCP_DIAG_H__

#include "board_config.h"
#include <stdint.h>

/* ==========================================================================
 * Live observation state.
 *
 * Every counter below is incremented from exactly one place in normal
 * operation, so a drift between them is meaningful:
 *
 *   probe_ok   -- 0x55 answered an address probe
 *   nack       -- read refused because nothing was waiting (NORMAL while idle)
 *   read_fail  -- genuine bus fault, BUSY/TIMEOUT. Resets on any success.
 *   frames_ok  -- fully valid frames: magic, part_id, checksum all correct
 *   json       -- frames whose payload starts with '{', i.e. the model replied
 * ======================================================================== */

extern volatile uint32_t mcp_state;
extern volatile uint32_t mcp_probe_count;
extern volatile uint32_t mcp_probe_ok_count;
extern volatile uint32_t mcp_register_attempts;
extern volatile uint32_t mcp_register_ok;
extern volatile uint32_t mcp_frames_ok;
extern volatile uint32_t mcp_json_frames;
extern volatile uint32_t mcp_bad_header;
extern volatile uint32_t mcp_bad_checksum;
extern volatile uint32_t mcp_read_fail;
extern volatile uint32_t mcp_nack_count;
extern volatile uint32_t mcp_tool_calls;
extern volatile uint32_t mcp_finish_sent;
extern volatile uint32_t mcp_bus_recover_count;
extern volatile uint32_t mcp_tx_total_bytes;
extern volatile uint32_t mcp_last_status;
extern volatile uint32_t mcp_last_error;
extern volatile uint32_t mcp_i2c_speed_hz;

/* The most recent frame, for the debugger and for printing. */
extern volatile uint8_t  mcp_last_header[MCP_HDR_LEN];
extern volatile uint16_t mcp_last_len;
extern volatile uint8_t  mcp_last_payload[MCP_RX_BUF_LEN];

/* Bring-up state machine position. Purely for observation.
 * NOTE: MCP_STATE_ACTIVE is numerically 2, which collides with the module's
 * own status code 2 = "configuring". That collision predates this refactor and
 * is left alone -- see README section 6. */
typedef enum
{
  MCP_STATE_PROBING = 0U,   /* nothing ACKed at 0x55 yet                     */
  MCP_STATE_SILENT  = 1U,   /* 0x55 ACKs, registration sent, no frames yet   */
  MCP_STATE_ACTIVE  = 2U    /* frames are arriving                           */
} MCP_State_t;

/* ==========================================================================
 * Logging.
 *
 * MCP_LOG(fmt, ...)  -- compiles away entirely when BOARD_DEBUG_UART is 0.
 * Anything that only exists to be printed belongs inside one of these, so the
 * final build carries no dead string constants either.
 * ======================================================================== */

#if BOARD_DEBUG_UART
  #define MCP_LOG(...)      printf(__VA_ARGS__)
  #define MCP_LOG_ENABLED   1
#else
  #define MCP_LOG(...)      do { } while (0)
  #define MCP_LOG_ENABLED   0
#endif

/* Human-readable name for the module's 1-byte status broadcast. */
const char *MCP_StateName(uint8_t code);

/* Print a buffer as hex and as printable ASCII. The ASCII pass renders
 * non-printable bytes as '.', because a 1-byte payload of 0x01 would otherwise
 * print as nothing at all and look exactly like "no data received". */
void MCP_Dump(const uint8_t *data, uint16_t len);

/* One-line counter summary. Without it, silence on the console is ambiguous
 * between "the module is quiet" (normal) and "every read is failing". */
void MCP_StatusReport(uint32_t now_ms);

/* Reset the counters. Only used at startup. */
void MCP_DiagReset(void);

#endif /* __MCP_DIAG_H__ */
