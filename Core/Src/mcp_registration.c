/* ---------------------------------------------------------------------------
 * mcp_registration.c -- walk the tool table and push it to the module.
 *
 * The loop is the whole point of the table: adding a tool means adding a row in
 * mcp_tools.c, not adding a fourth copy-pasted send call here. The previous
 * version had the three sends written out longhand, with a hand-tuned delay
 * after each, so the shape of the registration was encoded in the code's
 * repetition rather than in data.
 *
 * ---------------------------------------------------------------------------
 * WHY IT DOES NOT STOP AFTER THE FIRST FRAME
 *
 * Registration retries until a frame arrives whose payload actually looks like
 * JSON -- not merely until any frame arrives. An earlier version stopped at the
 * first frame of any kind, so the module's very first 1-byte power-on notice
 * permanently switched registration off; and because the module only honours
 * registration during its startup window, a later module reboot could then
 * never be registered at all. That is why talking to the model produced nothing
 * on the bus. The retry decision lives in main.c; this file just does the send.
 * -------------------------------------------------------------------------*/

#include "mcp_registration.h"
#include "mcp_tools.h"
#include "mcp_transport.h"
#include "mcp_diag.h"
#include <stdio.h>   /* MCP_LOG expands to printf */

HAL_StatusTypeDef MCP_ToolsRegister(I2C_HandleTypeDef *hi2c)
{
  HAL_StatusTypeDef st = HAL_OK;
  uint16_t i;

  for (i = 0U; i < MCP_ToolCount; i++)
  {
    const mcp_tool_t *tool = &MCP_ToolTable[i];

    /* One transaction per tool, the length taken from the table rather than
     * from strlen() here -- see mcp_tools.h for why the two differ. */
    st = MCP_SendBytes(hi2c, (const uint8_t *)tool->data, tool->len);

    if (st != HAL_OK)
    {
      break;
    }

    MCP_LOG("    tool #%u: %u bytes\r\n", (unsigned)(i + 1U), (unsigned)tool->len);
    HAL_Delay(MCP_REG_SETTLE_MS);
  }

  /* Back to read speed for the short frame reads that follow. */
  (void)MCP_SetSpeed(hi2c, MCP_I2C_SPEED_READ);

  return st;
}
