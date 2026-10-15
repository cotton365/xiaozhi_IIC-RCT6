/* ---------------------------------------------------------------------------
 * mcp_tools.c -- the JSON payloads registered with the cloud model.
 *
 * GENERATED ONCE from the literals that used to live in main.c, then hand-
 * maintained from here. The escape text below is byte-for-byte what it was;
 * see mcp_tools.h for why the lengths are written out and why the trailing NUL
 * on MCP_TOOL_FINISH is deliberate.
 *
 * ARM COMPILER 5 READS SOURCE IN THE SYSTEM CODEPAGE (GBK ON A CHINESE WINDOWS
 * INSTALL), NOT UTF-8. Literal Chinese here is therefore misparsed and can even
 * swallow a closing quote via a stray 0x5C byte, so the Chinese is stored as
 * explicit UTF-8 \xNN escapes split into separate literals at every ASCII
 * boundary. The split matters: C's \x escape is greedy, so "\x82duration"
 * would be read as the single escape \x82d. Splitting costs nothing and emits
 * identical bytes.
 *
 * The bytes on the wire are still UTF-8, so the JSON stays readable to the
 * model. Do not "simplify" these into plain text.
 * -------------------------------------------------------------------------*/

#include "mcp_tools.h"

/* self.robot.move_linear -- 386 bytes sent (strlen, no NUL) */
static const char MCP_TOOL_MOVE[] =
  "{\"tool_name\":\"self.robot.move_linear\","
  "\"command\":\""
  "\xE4\xBB\x85\xE5\x9C\xA8\xE7\x94\xA8\xE6\x88\xB7\xE8\xA6\x81\xE6\xB1\x82\xE7\x9B\xB4\xE7\xBA\xBF\xE7\xA7\xBB\xE5\x8A\xA8\xEF\xBC\x8C\xE4\xB8\x94\xE8\xB7\xAF\xE5\xBE\x84\xE5\xAE\x89\xE5\x85\xA8\xE6\x98\x8E\xE7\xA1\xAE\xE6\x97\xB6\xE8\xB0\x83\xE7\x94\xA8\xE3\x80\x82"
  "heading_deg"
  "\xE6\x98\xAF\xE7\x9B\xB8\xE5\xAF\xB9\xE5\xBD\x93\xE5\x89\x8D\xE6\x9C\x9D\xE5\x90\x91\xE7\x9A\x84\xE8\xA7\x92\xE5\xBA\xA6\xEF\xBC\x8C\xE5\x8F\xB3\xE8\xBD\xAC\xE4\xB8\xBA\xE6\xAD\xA3\xEF\xBC\x8C\xE5\xB7\xA6\xE8\xBD\xAC\xE4\xB8\xBA\xE8\xB4\x9F\xE3\x80\x82"
  "speed_pct"
  "\xE6\x98\xAF\xE9\x80\x9F\xE5\xBA\xA6\xE7\x99\xBE\xE5\x88\x86\xE6\xAF\x94\xE3\x80\x82"
  "duration_ms"
  "\xE6\x98\xAF\xE6\x9C\x89\xE9\x99\x90\xE8\xBF\x90\xE5\x8A\xA8\xE6\x97\xB6\xE9\x97\xB4\xE3\x80\x82\","
  "\"params\":[[\"heading_deg\",\"int\",-180,180],"
  "[\"speed_pct\",\"int\",0,50],[\"duration_ms\",\"int\",100,3000]],"
  "\"block\":\"true\",\"return\":\"false\"}"
  ;

/* self.robot.drive_arc -- 387 bytes sent (strlen, no NUL) */
static const char MCP_TOOL_ARC[] =
  "{\"tool_name\":\"self.robot.drive_arc\","
  "\"command\":\""
  "\xE4\xBB\x85\xE5\x9C\xA8\xE7\x94\xA8\xE6\x88\xB7\xE8\xA6\x81\xE6\xB1\x82\xE8\xBF\x9E\xE7\xBB\xAD\xE6\x9B\xB2\xE7\xBA\xBF\xE8\xA1\x8C\xE9\xA9\xB6\xEF\xBC\x8C\xE4\xB8\x94\xE8\xB7\xAF\xE5\xBE\x84\xE5\xAE\x89\xE5\x85\xA8\xE6\x98\x8E\xE7\xA1\xAE\xE6\x97\xB6\xE8\xB0\x83\xE7\x94\xA8\xE3\x80\x82"
  "turn"
  "\xE5\x8F\xAA\xE8\x83\xBD\xE6\x98\xAF"
  "left"
  "\xE6\x88\x96"
  "right"
  "\xE3\x80\x82"
  "radius_mm"
  "\xE6\x98\xAF\xE6\x9B\xB2\xE7\xBA\xBF\xE5\x8D\x8A\xE5\xBE\x84\xE3\x80\x82"
  "speed_pct"
  "\xE6\x98\xAF\xE9\x80\x9F\xE5\xBA\xA6\xE7\x99\xBE\xE5\x88\x86\xE6\xAF\x94\xE3\x80\x82"
  "duration_ms"
  "\xE6\x98\xAF\xE6\x9C\x89\xE9\x99\x90\xE8\xBF\x90\xE5\x8A\xA8\xE6\x97\xB6\xE9\x97\xB4\xE3\x80\x82\","
  "\"params\":[[\"turn\",\"string\"],[\"radius_mm\",\"int\",200,2000],"
  "[\"speed_pct\",\"int\",0,35],[\"duration_ms\",\"int\",100,3000]],"
  "\"block\":\"true\",\"return\":\"false\"}"
  ;

/* {"command":"mcp_setting"} -- 42 bytes sent (sizeof, WITH trailing NUL) */
static const char MCP_TOOL_FINISH[] =
  "{\"command\":\"mcp_setting\",\"params\":\"true\"}"
  ;

/* --------------------------------------------------------------------------
 * The registry.
 *
 * Order matters only in that it is the order they go out in; registration
 * sends them top to bottom, with a settle delay between each.
 *
 * Every length here is the value transmitted, and the assertions below check
 * it against the array. Both are needed: the assertion catches a typo in the
 * number, and the written number is what makes FINISH's extra byte visible.
 * (FINISH genuinely is one longer than its text, because it is sent with the
 * terminating NUL -- see mcp_tools.h.)
 * ------------------------------------------------------------------------ */
const mcp_tool_t MCP_ToolTable[] =
{
  { MCP_TOOL_MOVE,   386U },   /* strlen  */
  { MCP_TOOL_ARC,    387U },   /* strlen  */
  { MCP_TOOL_FINISH, 42U },   /* sizeof  -- includes the trailing '\0' */
};

const uint16_t MCP_ToolCount =
  (uint16_t)(sizeof(MCP_ToolTable) / sizeof(MCP_ToolTable[0]));

/* Compile-time guards. A C99 array size of -1 is a hard error, which is the
 * portable way to assert before _Static_assert was widely available (ARMCC 5
 * in C99 mode has no _Static_assert). If one of these fires, the returned
 * length and the literal disagree -- fix the number by reading the value out of
 * the error, and check it against measure_tool_strings.py rather than guessing. */
typedef char mcp_assert_move[((sizeof(MCP_TOOL_MOVE)   == 387U) ? 1 : -1)];
typedef char mcp_assert_arc[((sizeof(MCP_TOOL_ARC)     == 388U) ? 1 : -1)];
typedef char mcp_assert_finish[((sizeof(MCP_TOOL_FINISH) == 42U) ? 1 : -1)];
