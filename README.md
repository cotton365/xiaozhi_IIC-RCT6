# IIC-RCT6 modularization

**Date**: 2026-10-13
**Repo**: `C:\Users\Lenovo\Desktop\wendang\keil\IIC-RCT6` (branch `master`)
**Status**: ✅ MCP bring-up **functionally complete and verified on hardware**. Next goal is **modularization**, not new features.

---

## 1. What this project is

STM32F103RCT6 (ALIENTEK MiniSTM32 V4) acting as **I2C master** to a Hiwonder **WonderMind V1.0** board (single PCB, two chips: U1 = CI1302 voice ASR, U2 = ESP32-S3 running the LLM). The STM32 registers MCP tools over I2C to the ESP32-S3 at 7-bit **0x55**, then receives LLM tool calls as JSON frames.

**The chain works end-to-end.** A real tool call was received and decoded:

```
AA 55 00 1F 01 00 02 00            header: len=31, part_id=1, part_num=2
{"heading_deg":0,"speed_pct":30,"duration_ms":1000}    payload (31 + 20 = 51 bytes)
```

That payload uses exactly the parameter names of the registered `self.robot.move_linear` tool, proving: registration reached the cloud model, the model selected the correct tool, the module pushed the call onto I2C, and the STM32's fragment reassembly + XOR checksum are all correct.

**Protocol details, bug history, and hard-won gotchas are already documented** in
`C:\Users\Lenovo\.claude\plans\wonderllm-handoff-md-linear-reddy.md` — read that file first; do not re-derive the protocol. It covers the MCP frame layout, byte order, the CI1302 UART-vs-I2C distinction, the five bug classes found and fixed, and the anti-pitfall list (F1 `MspInit` gating, ARMCC GBK source encoding, greedy `\x` escapes, semihosting, LED pin).

## 2. Immediate next task: modularize `Core/Src/main.c`

**Problem**: `main.c` is **1007 lines** and holds the entire application — protocol code, I2C transport, registration payloads, diagnostics, LED service, and `printf` retarget — all inside CubeMX `USER CODE` blocks. It is hard to navigate and every new feature makes it worse.

**Constraint that shapes the whole approach**: `main.c` is **CubeMX-generated**. CubeMX preserves only the text between `/* USER CODE BEGIN x */` and `/* USER CODE END x */`. Anything outside those markers is destroyed on regeneration. So the split must move code into **new files** and leave only thin calls behind.

### Current contents of `main.c` (all application code lives in USER CODE regions)

| Region | Lines | Contains |
|---|---|---|
| `Includes` | 26–31 | `stdio.h`, `string.h` |
| `PTD` | 34–44 | `MCP_State_t` enum |
| `PD` | 47–110 | Address/len/timeout/speed macros, buffer sizes |
| `PV` | 119–155 | ~18 `volatile` observability counters, last-frame buffers, tick timestamps |
| `PFP` | 159–174 | 10 static prototypes |
| `0` | 177–215 | `printf` retarget (`__use_no_semihosting`, `fputc`, …) |
| `2` | 248–274 | Startup banner, back-dated registration tick |
| `WHILE` | 277–481 | Main loop: registration retry, frame receive, tool-call reply, status summary |
| `4` | 527–976 | All 11 MCP functions + the 3 tool JSON constants |

**Functions to relocate** (currently `static` in `main.c`): `MCP_SetSpeed`, `MCP_Probe`, `MCP_SendBytes`, `MCP_RegisterTools`, `MCP_ReadFrame`, `MCP_BusRecover`, `MCP_Dump`, `MCP_StateName`, `MCP_LedInit`, `MCP_LedService`, plus the tool strings `MCP_TOOL_MOVE` / `MCP_TOOL_ARC` / `MCP_TOOL_FINISH`.

### Suggested target layout (adjust as the design warrants)

- `mcp_protocol.[ch]` — frame parse/build, XOR checksum, header byte order, JSON-prefix detection
- `mcp_transport.[ch]` — `HAL_I2C_Master_Receive/Transmit` wrappers, runtime speed switching, bus recovery
- `mcp_registration.[ch]` — the three tool JSON payloads (the `\xNN`-escaped literals) + `MCP_RegisterTools()`
- `mcp_diag.[ch]` — counters, `MCP_Dump`, `MCP_StateName`, periodic status summary
- `ws2812`-style LED service → `led.[ch]` (PA8, active low)
- `retarget_io.c` — the `printf`/semihosting block

Keep the public surface of each module **narrow** — a handful of functions taking explicit parameters rather than reaching into file-scope globals.

## 3. 🔴 Modularization gotchas (will cost a session if missed)

1. **New `.c` files must be added to the Keil project or you get a link error.** The project is MDK-ARM; sources are enumerated in `MDK-ARM/IIC-RCT6.uvprojx` as `<File>` entries inside `<Group>` blocks. Current app groups list `main.c`, `gpio.c`, `i2c.c`, `usart.c`, `stm32f1xx_it.c`, `stm32f1xx_hal_msp.c` plus HAL drivers. Adding a file means editing that XML. **Previous sessions deliberately avoided `.uvprojx` edits** — so if you add files, expect to do careful XML surgery or ask the user to add them via Keil's Project pane. Also note `.uvoptx` holds per-user state; don't hand-edit it.
2. **Never put application code outside `USER CODE` markers** in `main.c` — CubeMX deletes it. Thin calls + `#include`s inside the markers are fine.
3. **`printf` retarget must survive the split.** MicroLIB is **OFF**, so `#pragma import(__use_no_semihosting)` and the `fputc` override are mandatory; without them the first `printf` hits a semihosting `BKPT` and the board appears to hang with no output.
4. **Chinese string literals must stay `\xNN`-escaped.** ARMCC reads source in the system codepage (GBK), not UTF-8; literal Chinese breaks the build with `missing closing quote`. Two helper scripts already exist and are reusable:
   - `C:\Users\Lenovo\Desktop\wendang\keil\_tools\make_c_escapes.py` — text → C escapes
   - `C:\Users\Lenovo\Desktop\wendang\keil\_tools\verify_tool_strings.py` — byte-compares the JSON embedded in the built `.axf` against `act_tool_strings.h`
   Run the verifier after any move of the tool strings. It currently reports `EXACT=True` for both tools (386 / 387 bytes).
5. **`Optim=4` = `-O3`** (`uAC6=0`, i.e. ARMCC 5). Non-`volatile` file-scope variables can be optimised into registers and vanish from the debugger. Keep the observability counters `volatile` across the move.

## 4. Build & verify

```bash
cd "C:/Users/Lenovo/Desktop/wendang/keil/IIC-RCT6/MDK-ARM"
"E:/KEIL/UV4/UV4.exe" -b "IIC-RCT6.uvprojx" -o "build_check.log"
grep -iE "error|warning|Program Size" build_check.log
```

Last known-good: **0 errors, 0 warnings**, `Code=10812 RO-data=1920 RW-data=124 ZI-data=2300`.

Reference build config: `Device=STM32F103RC`, `Define=USE_HAL_DRIVER,STM32F103xE`, `uC99=1`, `Optim=4`, `IROM 0x8000000–0x803FFFF`, `IRAM 0x20000000–0x2000BFFF`.

After the split, re-run `verify_tool_strings.py` and build to the same clean result. **Modularization must not change behaviour** — the firmware currently works, so treat any functional delta as a regression to explain, not a fix to keep.

## 5. Hardware notes for testing (if the user re-flashes)

- Wiring: STM32 **PB6=SCL / PB7=SDA** ↔ module header **P6** (5V/GND/SCL1/SDA1). Module has onboard 4.7K pull-ups; user added ~5K in parallel. **Do not add more.**
- LED is **PA8** (red, active low). **Not PC13** — PC13 is the touch panel chip-select on this board.
- UART1 (PA9/PA10) → onboard CH340 → same USB cable. 115200 8N1.
- Registration retry is currently **200 ms** with an immediate first attempt at power-up (`MCP_REG_RETRY_MS` in the `PD` region; the back-dated tick is in `USER CODE 2`). The module only honours MCP registration during its startup window, so the intended procedure is: start the STM32 first, then power-cycle the ESP32-S3.
- Module status bytes arrive as 1-byte frames and are named by `MCP_StateName()`: `1=starting, 2=configuring, 3=idle, 4=connecting, 5=listening, 6=speaking, …`

## 6. Known non-blocking issues (do not "fix" unless asked)

- Registration retry stops permanently once any JSON frame arrives (`mcp_json_frames != 0`). If the module reboots after that, registration is not re-sent.
- The internal enum value `mcp_state == 2` collides numerically with the module's status `2 = configuring`, which is confusing in logs. Renaming the enum would help; it is cosmetic.
- Tool calls are detected by "payload starts with `{`" rather than by a `tool_name` field, because this firmware version sends bare parameters with no wrapper.

## 7. Suggested skills

- **`codebase-design`** — the core skill for this task. Use its deep-module vocabulary when deciding where the seams go and how narrow each module's interface should be.
- **`simplify`** — run after the split to catch reuse and altitude problems introduced by the move.
- **`code-review`** — review the modularization diff; the main risk is a behaviour change hiding in a "pure move".
- **`tdd`** — only if the user wants host-side tests for the pure logic (frame parse, XOR checksum, byte order). These are testable without hardware since they are pure functions over byte buffers.

## 8. Reference paths

| What | Where |
|---|---|
| Protocol, bugs, gotchas (read first) | `C:\Users\Lenovo\.claude\plans\wonderllm-handoff-md-linear-reddy.md` |
| File to split | `IIC-RCT6\Core\Src\main.c` |
| Keil project (source list) | `IIC-RCT6\MDK-ARM\IIC-RCT6.uvprojx` |
| Authoritative tool definitions | `WonderLLM大模型模块（ESP32-S3）\robot-planner-skill\i2c\act_tool_strings.h`, `..\acts.json` |
| Vendor Arduino master reference | `WonderLLM大模型模块（ESP32-S3）\1.课程资料\2.进阶开发\附录\04 Arduino程序文件\01 MCP工具使用\WonderLLM_example\` |
| Python I2C host (cross-check) | `WonderLLM大模型模块（ESP32-S3）\robot-planner-skill\i2c\ch341_wonder_mcp.py` |
| Vendor board schematic | `WonderLLM大模型模块（ESP32-S3）\3.芯片资料&技术参数\5.原理图\SCH_WonderLLM V1.0.pdf` |
| String-encoding helpers | `C:\Users\Lenovo\Desktop\wendang\keil\_tools\` |





## 9. Additional: Notes on Modularization
| File | Line | Responsibility |
|------|------|----------------|
| board_config.h | 193 | Sole pin configuration point |
| mcp_transport.[ch] | 169 | I2C bus layer (probe/send-receive/speed change/recovery) |
| mcp_protocol.[ch] | 209 | Frame parsing |
| mcp_tools.[ch] | 95 | Tool table |
| mcp_registration.[ch] | 54 | Registration flow (iterate table and send) |
| mcp_diag.[ch] | 155 | Counters + logging |
| led.[ch] | 89 | Heartbeat LED |
| retarget_io.[ch] | 141 | printf + ring buffer |
