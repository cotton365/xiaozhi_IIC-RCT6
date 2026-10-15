/* ---------------------------------------------------------------------------
 * retarget_io.c -- printf retarget and the non-blocking log ring buffer.
 *
 * Two separate concerns in one file because they are two halves of one
 * mechanism: fputc() appends, MCP_RetargetService() drains.
 *
 * ---------------------------------------------------------------------------
 * THE SEMIHOSTING TRAP (this has bitten this project before -- do not remove)
 *
 * MicroLIB is OFF in this project, so the full C library is linked. Without
 * the `__use_no_semihosting` pragma below, the linker pulls in the semihosting
 * stubs, and the first printf() executes a BKPT that only a debugger can
 * service. The board then stops silently with no output whatsoever, which
 * looks like a firmware hang and is indistinguishable from one unless you know
 * this. _sys_exit and _ttywrch must exist for the same reason.
 *
 * All four parts -- the pragma, _sys_exit, _ttywrch, fputc -- are load-bearing.
 * -------------------------------------------------------------------------*/

#include "retarget_io.h"
#include "mcp_diag.h"
#include <stdio.h>

#if BOARD_DEBUG_UART

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

/* --------------------------------------------------------------------------
 * Ring buffer.
 *
 * head = where the next byte is written, tail = where the next byte is read.
 * Both indices are uint16 and the mask keeps them inside the buffer, so no
 * modulo is needed. Empty is head == tail; full is detected separately against
 * the computed head, which is why the buffer never holds more than
 * BOARD_LOG_RING_SIZE - 1 bytes.
 *
 * volatile: fputc can in principle be reached from an ISR context, and the
 * service routine reads these from main. The indices are only ever advanced
 * by one writer and one reader respectively.
 * ------------------------------------------------------------------------ */
static volatile uint8_t  s_ring[BOARD_LOG_RING_SIZE];
static volatile uint16_t s_head = 0U;
static volatile uint16_t s_tail = 0U;

static void *s_huart = 0;   /* UART_HandleTypeDef*, opaque to this file */

volatile uint32_t g_uart_dropped = 0U;
volatile uint32_t g_uart_pending = 0U;

void MCP_RetargetInit(void *huart)
{
  s_huart = huart;
}

void MCP_RetargetPutc(uint8_t byte)
{
  uint16_t head = s_head;
  uint16_t next = (uint16_t)((head + 1U) & (uint16_t)(BOARD_LOG_RING_SIZE - 1U));

  if (next == s_tail)
  {
    /* Full. Dropping one byte keeps the caller on its way; blocking here would
     * hold the bus. The counter makes the loss visible. */
    g_uart_dropped++;
    return;
  }

  s_ring[head] = byte;
  s_head = next;
  g_uart_pending++;
}

void MCP_RetargetService(uint16_t max_bytes)
{
  uint16_t sent = 0U;

  if (s_huart == 0)
  {
    return;
  }

  while ((s_tail != s_head) && (sent < max_bytes))
  {
    uint8_t byte = s_ring[s_tail];

    /* Blocking, but only for the bytes we chose to send, and only one at a
     * time. max_bytes bounds how long this can hold the caller. */
    if (HAL_UART_Transmit((UART_HandleTypeDef *)s_huart, &byte, 1U, 10U) != HAL_OK)
    {
      /* UART is not accepting bytes. Stop draining rather than spinning --
       * the buffer stays intact and the next pass retries. */
      break;
    }

    s_tail = (uint16_t)((s_tail + 1U) & (uint16_t)(BOARD_LOG_RING_SIZE - 1U));
    if (g_uart_pending > 0U)
    {
      g_uart_pending--;
    }
    sent++;
  }
}

int fputc(int ch, FILE *f)
{
  (void)f;
  MCP_RetargetPutc((uint8_t)ch);
  return ch;
}

#else  /* BOARD_DEBUG_UART == 0 */

/* Logging switched off for the final build: no printf, no ring buffer, no
 * UART dependency. The serial port is configured away entirely, which is the
 * intended end state. */

volatile uint32_t g_uart_dropped = 0U;
volatile uint32_t g_uart_pending = 0U;

void MCP_RetargetInit(void *huart)   { (void)huart; }
void MCP_RetargetPutc(uint8_t byte)  { (void)byte; }
void MCP_RetargetService(uint16_t max_bytes) { (void)max_bytes; }

#endif /* BOARD_DEBUG_UART */
