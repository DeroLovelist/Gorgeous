# Toolchain Routes (EIDE + Keil)

This project supports two build routes from the same source tree. Determine which is active before building.

## EIDE route (arm-none-eabi GCC)

- Entrypoints: `.eide/eide.yml` (project config), `builder.params` (build input list), `SCServo.ld` (linker script), `startup_stm32f407xx_gcc.S` (GCC startup).
- Open the workspace via `<project>\SCServo.code-workspace`, not the `MDK-ARM/` folder directly. EIDE auto-loads the project from the `.code-workspace`'s directory.
- GCC path is configured in VS Code user settings `EIDE.ARM.GCC.InstallDirectory` (currently `D:/gcc_arm`). The toolchain must NOT live under a path containing non-ASCII/Chinese characters or `ld` may fail to find `crti.o`/`libgcc`/`libc_nano`.
- Build/flash through EIDE (its Build and Flash actions) or its underlying commands. Do not assume bare `make`/`cmake` exists.
- EIDE caches project config in memory. After editing `eide.yml` externally, reload the EIDE project / reload the window BEFORE building, or EIDE may write stale config back over your edits.
- After CubeMX regenerates new peripherals, manually register new sources:
  - Add `Core/Src/<new>.c` and matching `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_*.c` to `eide.yml` virtualFolders.
  - Add them to `builder.params` sourceList.
  - Otherwise the link fails with undefined references (e.g. `MX_I2C2_Init`, `HAL_I2C_EV_IRQHandler`).
- Linker script: `SCServo.ld` selected via EIDE custom scatter settings (`useCustomScatterFile:true` + `scatterFilePath`). Verify Flash/RAM sizes match the actual device before editing.

## Keil route (MDK-ARM, AC5/AC6)

- Entrypoint: `MDK-ARM/123.uvprojx`. Linker source of truth: `MDK-ARM/*.sct` scatter file. Keil startup: `startup_stm32f407xx.s`.
- `Objects/`, `Listings/`, `*.uvoptx`, `*.build_log.htm`, and the `MDK-ARM/123/` output folder are inspection-only unless the request explicitly targets them.
- Keil sources are often GBK/GB2312. See device-and-safety.md for the encoding rule.
- Keep EIDE and Keil generated outputs separate; each toolchain owns its own output directories.

## Migration rules (Keil -> GCC/EIDE)

1. Build the untouched source project and record size/output.
2. Inventory sources, definitions, include paths, compiler/assembler flags, linker options, startup, and linker script.
3. Reproduce those inputs in EIDE before refactoring (startup file must be the GCC variant `startup_stm32f407xx_gcc.S`, not ARMCC `startup_stm32f407xx.s`).
4. Compare section sizes and symbols between ELFs.
5. Keep `.ioc` regeneration working and document which build files CubeMX owns.

Do not mix artifacts from different compilers or float ABIs. A successful link does not prove identical startup, constructors, syscall stubs, or library behavior.

## Flash and debug

- Prefer the project's established ST-Link configuration: on-board ST-Link, STM32CubeProgrammer CLI, or OpenOCD.
- Verify probe identity, interface (normally SWD), reset method, target voltage, and ELF path. Flash the ELF during debugging so symbols and load addresses remain aligned.
- Use BIN only with an explicitly verified base address.
- Mass erase and option-byte changes are approval-gated (see device-and-safety.md).
