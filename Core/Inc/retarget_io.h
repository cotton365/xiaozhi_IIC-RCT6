/* ---------------------------------------------------------------------------
 * retarget_io.h -- printf onto USART1, without ever blocking the caller.
 *
 * WHY A RING BUFFER
 *
 * The obvious retarget is a blocking HAL_UART_Transmit per byte, which is what
 * this project used to do. Measured at 115200 8N1 that is ~87 us of held CPU
 * per byte, so a single frame dump (header line + HEX line + RAW line, the RAW
 * line growing with the payload) costs 8-10 ms of dead time on the receive
 * path -- and it grows with the frame. On a board that has other work to do
 * while the module is transmitting, that is the expensive part, not the
 * HAL_Delay calls.
 *
 * So fputc() only appends to a ring buffer and returns. If the buffer is full
 * the byte is DROPPED and counted, never waited for: losing log bytes is
 * survivable, stalling the bus mid-frame is not.
 *
 * The buffer is drained by MCP_RetargetService(), called from the main loop.
 * Draining happens between transfers, not inside one, so it cannot disturb
 * bus timing.
 *
 * ORDERING NOTE: the UART is initialised by MX_USART1_UART_Init() before
 * MCP_RetargetInit() is called, so the handle is valid by the time anything
 * can be drained. Failing that order, the drain simply sees no handle and
 * holds the bytes until one appears.
 * -------------------------------------------------------------------------*/

#ifndef __RETARGET_IO_H__
#define __RETARGET_IO_H__

#include "board_config.h"
#include <stdint.h>

/* Bytes lost because the ring buffer was full. If this is climbing, the log is
 * being truncated and the console is no longer a complete record. */
extern volatile uint32_t g_uart_dropped;

/* Bytes currently waiting to be sent. Useful when tuning the drain. */
extern volatile uint32_t g_uart_pending;

/* Hand the UART over. Call once, after MX_USART1_UART_Init(). */
void MCP_RetargetInit(void *huart);

/* Append one byte. Never blocks. Drops and counts when full. */
void MCP_RetargetPutc(uint8_t byte);

/* Push whatever is buffered out to the UART. Call from the main loop.
 * Sends at most max_bytes, so a long backlog is spread across loop passes
 * rather than monopolising one. */
void MCP_RetargetService(uint16_t max_bytes);

#endif /* __RETARGET_IO_H__ */
