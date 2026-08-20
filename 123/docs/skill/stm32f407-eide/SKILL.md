---
name: stm32f407-eide
description: Tool-neutral agent rules for STM32F407VET6 firmware development with EIDE (arm-none-eabi GCC) and Keil MDK dual toolchains, STM32CubeMX/HAL, SCServo serial-servo library, and ST-Link flashing. Use when an agent needs to inspect or modify STM32F407VET6 projects (.ioc, .eide/eide.yml, .uvprojx, SCServo.ld, startup_stm32f407xx_gcc.S), edit HAL user code, avoid generated/build files, keep EIDE and Keil configs in sync, debug HardFault/boot/clock/UART issues, or package reusable STM32F407 examples.
---

# STM32F407 (EIDE + Keil) Agent Skill

Use this skill for STM32F407VET6 firmware projects that use STM32CubeMX/HAL with the EIDE (Embedded IDE, arm-none-eabi GCC) or Keil MDK (AC5/AC6) toolchains and ST-Link flashing, including the Feetech SCServo serial-servo library. It is intended for CLI/editor agents and is independent of the MSPM0 skill: do not mix MSPM0/SysConfig/DriverLib assumptions here.

## Default Workflow

1. Locate project entrypoints before editing:
   - EIDE route: `.eide/eide.yml` (project config) plus `builder.params`, the linker script `SCServo.ld`, and GCC startup `startup_stm32f407xx_gcc.S`.
   - Keil route: `MDK-ARM/*.uvprojx` plus `*.sct` scatter file and Keil startup `startup_stm32f407xx.s`.
   - Hardware config source of truth: `.ioc` (CubeMX).
2. Confirm the full target is `STM32F407VET6` (512 KiB Flash member, 100-pin LQFP, Cortex-M4F), not ZGT6 or another density/package. Verify HSE/LSE frequencies, power, boot pins, and debug interface when they affect the task.
3. Establish a clean baseline build before changing code when a toolchain is available. Record existing failures separately.
4. Modify the smallest relevant source surface (HAL user-code regions or user-owned files). Preserve CubeMX-owned generated code.
5. Build with the project's native method (EIDE build, or Keil if the user is working in Keil). Do not invent a different toolchain merely because it is installed.
6. Verify the ELF (warnings, map memory use, target flags) before flashing. Flash via the established ST-Link route (STM32CubeProgrammer CLI or OpenOCD) only when flashing is requested.
7. State hardware assumptions, changed files, the exact build/flash command used, verification evidence, and anything needing physical-board confirmation.

## Core Rules

- Treat `.ioc` as the hardware source of truth for pinmux, clock tree, peripheral mode, DMA routing, NVIC, and generated HAL initialization. Make such changes in `.ioc` and regenerate, not by hand-editing `MX_*_Init` functions.
- Preserve code only in CubeMX `USER CODE BEGIN/END` regions or user-owned files. CubeMX may overwrite edits elsewhere. Inspect regeneration diffs for deleted user code, renamed handles, and middleware changes.
- Do not hand-edit generated/build outputs: `build/`, `MDK-ARM/123/` (objects, `.map`, `.lst`, `.d`, `.crf`), `.uvoptx`, `*.build_log.htm`, generated headers like `RTE_Components.h` unless a request explicitly targets them.
- Do not guess HAL symbol names or register values. Read the actual generated files (`main.c`, `*_hal_msp.c`, `stm32f4xx_hal_conf.h`) and the datasheet/RM0090.
- Do not invent pins, clock frequencies, peripheral instances, or tool versions. Validate against the board schematic, `.ioc`, datasheet, and errata.
- Never infer LED polarity, button pins, crystal frequency, or board wiring from the MCU model alone. The board is determined by the schematic, not the part number.
- Preserve unrelated user code, comments, headers, project layout, and existing settings. If a feature requires a larger rewrite, explain why first.
- Do not change target chip, toolchain, linker script, or debug probe without user confirmation.
- Keep interrupt handlers short; move substantial work to main loop or RTOS tasks. Use `volatile` only where access semantics require it.
- For DMA on this part, verify stream/channel/request mapping, transfer width, buffer lifetime, and callbacks. DMA cannot access CCM RAM.
- Check clock-tree consequences before changing PLL/prescalers: recalculate peripheral clocks, timer clock doubling, Flash latency, SysTick/RTOS tick, and UART baud rates.
- Avoid blocking HAL delays in ISRs and time-critical paths. Define timeouts for peripheral waits.
- If hardware behavior is not verified on a connected board, say that validation stopped at source, build, or flash level.

## EIDE Project Checks

- Treat `.eide/eide.yml` as the EIDE project config source and `builder.params` as the build input list. The workspace file is `<project>\SCServo.code-workspace`; open that, not the `MDK-ARM/` folder directly.
- The GCC toolchain is managed by EIDE (`EIDE.ARM.GCC.InstallDirectory` in VS Code user settings, currently `D:/gcc_arm`). Do not run bare `make`/`cmake` unless the project already uses them.
- EIDE caches project configuration in memory: after editing `eide.yml` externally, reload the EIDE project (or reload the window) BEFORE building, otherwise EIDE may overwrite the file with stale config.
- When CubeMX regenerates new peripherals (e.g. new I2C/DMA/UART), EIDE does not auto-add new sources: manually add the new `Core/Src/*.c` and `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_*.c` to `eide.yml` virtualFolders and to `builder.params` sourceList, or the link fails with undefined references (`MX_I2C2_Init`, `HAL_I2C_EV_IRQHandler`, etc.).
- Linker: `SCServo.ld` (GCC) is configured via EIDE custom scatter settings (`useCustomScatterFile` + `scatterFilePath`). Verify Flash/RAM regions match the actual device before changing memory layout.

## Keil Project Checks

- Treat `MDK-ARM/*.uvprojx` as the Keil project entrypoint, the `.sct` scatter file as the linker source of truth, and `Objects/`, `Listings/`, `*.uvoptx`, and build logs as inspection-only.
- Keil source files are commonly GBK/GB2312 encoded. Editing such files with UTF-8 tools rewrites them and corrupts Chinese comments into `U+FFFD`. Detect encoding before rewriting; preserve or re-decode original comment text (e.g. decode with GB2312 before editing).
- Keep `MDK-ARM/` contents for Keil use. Do not mix EIDE and Keil generated files; each toolchain owns its own output directories.

## Hardware Baseline (board-level defaults)

When the user confirms the board is the LCKFB (立创天空星) STM32F407VET6 development board:

- HSE = 8 MHz; board carries an on-board ST-Link and CH340 USB-serial.
- Servo control: Feetech SMS/STS serial servos via SCServo library on USART3, TX = PC10 / RX = PC11, AF7, 115200 baud; demo default servo ID = 1.
- Debug/flash: on-board ST-Link via STM32CubeProgrammer CLI or OpenOCD. Confirm probe and target voltage before flashing.
- Do not treat the above as universal: if the user names a different board, rely on its schematic and `.ioc` instead.

## High-Risk Operations

Require explicit user approval before changing option bytes, read/write protection, boot configuration, BOR level, watchdog hardware mode, debug-port availability, mass erase, or destructive recovery. Explain recovery implications before proceeding.

## Ambiguous Requests

If the user omits important hardware parameters, do not silently choose risky values.

- For low-risk defaults, use the existing project's settings or the SCServo examples, then tell the user which defaults were applied.
- For important parameters, ask before editing and offer a concrete recommendation. Important missing parameters include pin, peripheral instance, UART baud/data/parity/stop bits, servo ID, Timer period, PWM frequency/duty/polarity, ADC channel/reference/sample time, DMA direction/source/destination, and interrupt priority.

Read `references/device-and-safety.md` for device constraints and editing boundaries, `references/eide-keil-workflow.md` for the two build routes, and `references/debugging.md` when diagnosing runtime or hardware failures.
