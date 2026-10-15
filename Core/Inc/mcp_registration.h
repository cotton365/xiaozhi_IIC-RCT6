/* ---------------------------------------------------------------------------
 * mcp_registration.h -- push the tool definitions to the module.
 *
 * ---------------------------------------------------------------------------
 * WHY THIS STAYS BLOCKING
 *
 * The receive path was made non-blocking because the board has other work to
 * do while frames arrive. Registration is different: it happens once, in the
 * window between power-up and the first reply, and during that window there is
 * nothing else to do anyway -- the module only honours registration during its
 * own startup window, so waiting for anything else would only risk arriving
 * late.
 *
 * So this keeps the original shape: probe, three blocking writes, a settle
 * delay between each. Making it interrupt-driven would add states to reason
 * about in exchange for nothing.
 * -------------------------------------------------------------------------*/

#ifndef __MCP_REGISTRATION_H__
#define __MCP_REGISTRATION_H__

#include "board_config.h"
#include "stm32f1xx_hal.h"

/* Send every tool in MCP_ToolTable, in order, with MCP_REG_SETTLE_MS between
 * each and at the end. Returns HAL_OK only if all of them went out.
 *
 * The I2C speed is left at READ speed on return, since a read follows. */
HAL_StatusTypeDef MCP_ToolsRegister(I2C_HandleTypeDef *hi2c);

/* Settle time between consecutive registration writes. The module needs a
 * moment to process each one; this mirrors the original 100 ms spacing. */
#define MCP_REG_SETTLE_MS    100U

#endif /* __MCP_REGISTRATION_H__ */
