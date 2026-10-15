/* ---------------------------------------------------------------------------
 * mcp_diag.c -- counter state, status naming, frame dumping, status summary.
 *
 * Pure observation. Nothing in this file affects what goes on the bus, which
 * is deliberate: it is the one module that can be switched off (BOARD_DEBUG_UART
 * = 0) with no behavioural consequence at all.
 * -------------------------------------------------------------------------*/

#include "mcp_diag.h"
#include <stdio.h>
#include <string.h>

/* --------------------------------------------------------------------------
 * Counter state.
 *
 * ALL volatile, and none of them static. The project builds with Optim=4
 * (-O3, ARMCC 5), where a non-volatile file-scope variable that is only ever
 * written can be optimised into a register -- and then it does not exist to
 * look at in the debugger, which defeats the entire purpose of having it.
 * ------------------------------------------------------------------------ */

volatile uint32_t mcp_state              = MCP_STATE_PROBING;
volatile uint32_t mcp_probe_count        = 0U;
volatile uint32_t mcp_probe_ok_count     = 0U;
volatile uint32_t mcp_register_attempts  = 0U;
volatile uint32_t mcp_register_ok        = 0U;
volatile uint32_t mcp_frames_ok          = 0U;
volatile uint32_t mcp_json_frames        = 0U;
volatile uint32_t mcp_bad_header         = 0U;
volatile uint32_t mcp_bad_checksum       = 0U;
volatile uint32_t mcp_read_fail          = 0U;
volatile uint32_t mcp_nack_count         = 0U;
volatile uint32_t mcp_tool_calls         = 0U;
volatile uint32_t mcp_finish_sent        = 0U;
volatile uint32_t mcp_bus_recover_count  = 0U;
volatile uint32_t mcp_tx_total_bytes     = 0U;
volatile uint32_t mcp_last_status        = 0U;
volatile uint32_t mcp_last_error         = 0U;
volatile uint32_t mcp_i2c_speed_hz       = 0U;

volatile uint8_t  mcp_last_header[MCP_HDR_LEN];
volatile uint16_t mcp_last_len   = 0U;
volatile uint8_t  mcp_last_payload[MCP_RX_BUF_LEN];

void MCP_DiagReset(void)
{
  mcp_state             = (uint32_t)MCP_STATE_PROBING;
  mcp_probe_count       = 0U;
  mcp_probe_ok_count    = 0U;
  mcp_register_attempts = 0U;
  mcp_register_ok       = 0U;
  mcp_frames_ok         = 0U;
  mcp_json_frames       = 0U;
  mcp_bad_header        = 0U;
  mcp_bad_checksum      = 0U;
  mcp_read_fail         = 0U;
  mcp_nack_count        = 0U;
  mcp_tool_calls        = 0U;
  mcp_finish_sent       = 0U;
  mcp_bus_recover_count = 0U;
  mcp_tx_total_bytes    = 0U;
  mcp_last_status       = 0U;
  mcp_last_error        = 0U;
  mcp_last_len          = 0U;
}

/* --------------------------------------------------------------------------
 * The module broadcasts a single status byte as a 1-byte frame. Naming it
 * makes the log readable and shows where the module is in its boot sequence --
 * in particular 5 (listening), which means it is up, connected, and ready to
 * accept speech.
 * ------------------------------------------------------------------------ */
const char *MCP_StateName(uint8_t code)
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

/* --------------------------------------------------------------------------
 * Print a received frame three ways: hex, ASCII, and raw. The raw dump lets a
 * UTF-8 terminal render the Chinese JSON directly, which makes reading a tool
 * call easy.
 * ------------------------------------------------------------------------ */
void MCP_Dump(const uint8_t *data, uint16_t len)
{
#if MCP_LOG_ENABLED
  uint16_t i;
  uint8_t  b;

  MCP_LOG("HEX: ");
  for (i = 0U; i < len; i++)
  {
    MCP_LOG("%02X ", (unsigned)data[i]);
  }
  MCP_LOG("\r\n");

  /* Render non-printable bytes as '.' -- a 1-byte payload of 0x01/0x03 is a
   * control character and would otherwise print as nothing at all, which
   * looks exactly like "no data received". */
  MCP_LOG("RAW: ");
  for (i = 0U; i < len; i++)
  {
    b = data[i];
    putchar(((b >= 0x20U) && (b <= 0x7EU)) ? (int)b : (int)'.');
  }
  MCP_LOG("\r\n");
#else
  (void)data;
  (void)len;
#endif
}

/* --------------------------------------------------------------------------
 * Periodic counter summary.
 *
 * Read the failure counters together, never singly:
 *   - nack climbing while frames_ok is flat is the NORMAL idle state
 *   - read_fail climbing is a real fault and is what triggers bus recovery
 *   - register_ok flat while json==0 means registration is not landing
 * ------------------------------------------------------------------------ */
void MCP_StatusReport(uint32_t now_ms)
{
#if MCP_LOG_ENABLED
  MCP_LOG("[%lu ms] state=%lu ack=%lu reg=%lu frames=%lu json=%lu tool=%lu "
          "nack=%lu rdfail=%lu badhdr=%lu badchk=%lu\r\n",
          (unsigned long)now_ms,
          (unsigned long)mcp_state,
          (unsigned long)mcp_probe_ok_count,
          (unsigned long)mcp_register_ok,
          (unsigned long)mcp_frames_ok,
          (unsigned long)mcp_json_frames,
          (unsigned long)mcp_tool_calls,
          (unsigned long)mcp_nack_count,
          (unsigned long)mcp_read_fail,
          (unsigned long)mcp_bad_header,
          (unsigned long)mcp_bad_checksum);
#else
  (void)now_ms;
#endif
}
