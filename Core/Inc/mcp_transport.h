/* ---------------------------------------------------------------------------
 * mcp_transport.h -- the I2C bus layer. The only module that talks to hi2c.
 *
 * Owns: address probe, byte transmission, runtime speed switching, bus
 * recovery. Does not know anything about frames, tools, or JSON -- it moves
 * bytes and reports whether the bus cooperated.
 *
 * The handle is passed in rather than reached for, so this module has no
 * opinion about which I2C instance is in use. That matters if a future board
 * puts the module on I2C2 rather than I2C1: only main.c changes.
 *
 * ---------------------------------------------------------------------------
 * PINS ARE NOT WRITTEN HERE. Every port/pin below comes from board_config.h,
 * including in bus recovery, which drives SCL/SDA as plain GPIO. That is the
 * point of the abstraction -- see board_config.h for why the remap call
 * travels with the pin pair.
 * -------------------------------------------------------------------------*/

#ifndef __MCP_TRANSPORT_H__
#define __MCP_TRANSPORT_H__

#include "board_config.h"
#include "stm32f1xx_hal.h"

/* Program the bus speed. Switches between MCP_I2C_SPEED_READ (100 kHz) and
 * MCP_I2C_SPEED_WRITE (400 kHz). Idempotent -- returns immediately if the bus
 * is already at the requested speed.
 *
 * NOTE ON THE F1 HAL: this calls HAL_I2C_Init() directly, which only invokes
 * HAL_I2C_MspInit() when State == HAL_I2C_STATE_RESET. So MspInit does not
 * re-run here and the pins stay AF_OD from the initial init. That is correct
 * and intentional -- do NOT "fix" it by calling HAL_I2C_DeInit() first, which
 * would drop the pins out of alternate-function mode and leave them as plain
 * inputs. See MCP_BusRecover() for the one place DeInit is right. */
HAL_StatusTypeDef MCP_SetSpeed(I2C_HandleTypeDef *hi2c, uint32_t speed);

/* Address-only ACK test at 0x55. Forces read speed first, since the probe
 * precedes a read. */
HAL_StatusTypeDef MCP_Probe(I2C_HandleTypeDef *hi2c);

/* Send a byte stream as ONE transaction, with no chunking.
 *
 * The absence of chunking is not an optimisation, it is a correctness
 * requirement. An earlier version split the payload into 32-byte chunks with a
 * STOP after each, which made the slave see ~10 separate short messages instead
 * of one registration JSON, so the registration was fragmented and discarded.
 * The vendor's Python host sends the whole thing in one transaction, and its
 * Arduino host chunks with a repeated START and no STOP. Neither releases the
 * bus mid-message.
 *
 * Chunking is also unnecessary: HAL_I2C_Master_Transmit takes a uint16_t size
 * and clocks it all out in one blocking transfer. At 400 kHz, 386 bytes take
 * about 8.7 ms, well inside MCP_TX_TIMEOUT_MS.
 *
 * Forces write speed (400 kHz) -- mandatory for MCP JSON, see board_config.h. */
HAL_StatusTypeDef MCP_SendBytes(I2C_HandleTypeDef *hi2c, const uint8_t *data, uint16_t len);

/* Read into a caller-supplied buffer. Pure read: NO register address is sent
 * first, because this bus has no register addressing. Forces read speed. */
HAL_StatusTypeDef MCP_ReadBytes(I2C_HandleTypeDef *hi2c, uint8_t *out, uint16_t len);

/* --------------------------------------------------------------------------
 * Bus recovery: a slave stuck mid-transfer can hold SDA low forever, and the
 * only way out is for the master to clock it free -- nine SCL pulses followed
 * by a manual STOP.
 *
 * This tears the peripheral down and drives the pins by hand, so it is
 * disruptive: if the module is mid-transmission, recovery will cut across it.
 * Only call it on a genuine fault (BUSY latched, or the bus never clocked),
 * never on an idle NACK.
 * ------------------------------------------------------------------------ */
HAL_StatusTypeDef MCP_BusRecover(I2C_HandleTypeDef *hi2c);

#endif /* __MCP_TRANSPORT_H__ */
