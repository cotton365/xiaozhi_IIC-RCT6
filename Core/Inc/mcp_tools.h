/* ---------------------------------------------------------------------------
 * mcp_tools.h -- the tools registered with the cloud model.
 *
 * ADDING A TOOL
 *
 *   1. put the new command sentence in  _tools/_ref_strings.txt
 *   2. run  python -I _tools/make_c_escapes.py _tools/_ref_strings.txt
 *   3. paste the escaped lines into MCP_TOOL_<NAME> in mcp_tools.c
 *   4. add one row to MCP_ToolTable with its exact byte count
 *   5. add the tool's name to the list in  _tools/verify_tool_strings.py
 *   6. build, then run the verifier against the new .axf
 *
 * Step 5 is the only one outside mcp_tools.c, and it is there because the
 * verifier locates each literal in the linked image BY NAME.
 *
 * WHY THE BYTE COUNT IS WRITTEN OUT INSTEAD OF USING sizeof()
 *
 * sizeof would be the obvious choice and it is the wrong one here, because the
 * three tools are NOT all sent the same way. move_linear and drive_arc are sent
 * as strlen (no trailing NUL); mcp_setting has always been sent as sizeof,
 * i.e. WITH a trailing 0x00 byte on the wire. That inconsistency is historical
 * and it is preserved deliberately -- "tidying" it would change the bytes the
 * module receives, which is a behaviour change with no evidence to support it.
 *
 * Writing the number out makes each tool's real on-wire length visible at a
 * glance, which is exactly how that extra NUL stayed hidden for so long. The
 * static assertions in mcp_tools.c then keep the written number honest, so the
 * visible form is also the checked form.
 * -------------------------------------------------------------------------*/

#ifndef __MCP_TOOLS_H__
#define __MCP_TOOLS_H__

#include <stdint.h>

/* One tool: the JSON to send, and exactly how many bytes of it go on the wire.
 *
 * `data` is `const char *` rather than `const uint8_t *` so the literals need
 * no cast at their definition, and broadcasting a char array to the transport's
 * uint8_t* is done at the one call site in mcp_registration.c. */
typedef struct
{
  const char *data;
  uint16_t    len;      /* bytes actually transmitted -- NOT sizeof */
} mcp_tool_t;

extern const mcp_tool_t MCP_ToolTable[];
extern const uint16_t   MCP_ToolCount;

/* Indices into MCP_ToolTable. Named rather than numbered at the call sites,
 * because "the third tool" is not something a reader should have to count. */
#define MCP_TOOL_IDX_MOVE      0U
#define MCP_TOOL_IDX_ARC       1U
#define MCP_TOOL_IDX_FINISH    2U

#endif /* __MCP_TOOLS_H__ */
