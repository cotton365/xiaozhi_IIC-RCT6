/* ---------------------------------------------------------------------------
 * led.h -- heartbeat LED, non-blocking.
 *
 *     no ACK at 0x55        -> 2 Hz blink
 *     ACK, no frames yet    -> 1 Hz blink
 *     frames arriving       -> solid ON
 *
 * This is a view of the link state that does not need a debugger or a serial
 * port -- which matters once the serial port is compiled out.
 * -------------------------------------------------------------------------*/

#ifndef __LED_H__
#define __LED_H__

#include "board_config.h"
#include <stdint.h>

#if BOARD_USE_HEARTBEAT_LED

/* Claim the pin and start with the LED off. */
void MCP_LedInit(void);

/* Drive the LED forward. Call every pass of the main loop; it self-gates on
 * tick, so calling it too often is harmless. */
void MCP_LedService(void);

/* Note that a valid frame just arrived. This is what turns the LED solid -- it
 * is deliberately fed by the receive path rather than read out of mcp_state,
 * because "frames are arriving now" and "frames have arrived at some point"
 * are different claims and the LED must show the first one. Keying it off
 * mcp_state made it latch on forever after the first frame. */
void MCP_LedNoteFrame(uint32_t now_ms);

/* Note that the module answered a probe at all. Switches the blink from 2 Hz
 * to 1 Hz so it is visible at a glance whether the module is present. */
void MCP_LedNoteProbeOk(void);

#else  /* BOARD_USE_HEARTBEAT_LED == 0 */

/* Compile to nothing so the call sites need no #if of their own. */
#define MCP_LedInit()                     do { } while (0)
#define MCP_LedService()                  do { } while (0)
#define MCP_LedNoteFrame(now_ms)          do { (void)(now_ms); } while (0)
#define MCP_LedNoteProbeOk()              do { } while (0)

#endif /* BOARD_USE_HEARTBEAT_LED */

#endif /* __LED_H__ */
