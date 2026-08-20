# Device and Editing Constraints (STM32F407VET6)

## Target identity

- Treat the full order code as `STM32F407VET6`.
- `VE` denotes the 512 KiB Flash member in the 100-pin LQFP package family. Verify exact memory regions against the project's linker script and current ST documentation before changing memory layout. (`F405RG` and `F407VET6` have different Flash/RAM; do not mix them.)
- The Cortex-M4F core supports hardware floating point. Keep `-mcpu=cortex-m4`, Thumb, FPU selection, and float ABI consistent across application objects and prebuilt libraries.
- Maximum core frequency is 168 MHz under valid voltage scaling, regulator, Flash wait-state, and clock-source conditions. Never change the PLL from frequency alone; validate the complete clock tree (HSE/PLLM/N/Q/P, Flash latency, APB1/APB2 prescalers).
- Main SRAM is split across distinct regions, including core-coupled memory (CCM) with access limitations. **DMA cannot access CCM.** Do not place DMA buffers in CCM. Verify addresses/sizes from datasheet/RM0090 and the linker script.
- Typical board flash config on this toolchain: `HSE_VALUE=8000000`, `USE_HAL_DRIVER`, `STM32F407xx`, `ARM_MATH_CM4`.

## Sources of truth

Use this priority order:

1. Board schematic, BOM, and board revision for pins, oscillators, power rails, pull resistors, transceivers, and chip-select polarity.
2. Project `.ioc` for CubeMX-managed peripheral and clock intent.
3. STM32F407VET6 datasheet for pin alternate functions and electrical limits.
4. RM0090 reference manual for peripheral behavior and registers.
5. Device errata for silicon limitations.
6. Generated code and local comments only after checking consistency with the above.

Never infer LED polarity, button pins, crystal frequency, SDRAM/FSMC wiring, PHY address, or external Flash layout from the MCU model.

## CubeMX ownership

- Change pinout, clock tree, peripheral mode, DMA routing, NVIC settings, and generated middleware configuration in `.ioc`, then regenerate.
- Preserve code only in `USER CODE BEGIN/END` regions or user-owned files. CubeMX may overwrite edits elsewhere.
- Inspect regeneration diffs for deleted user code, renamed handles, middleware version changes, and reordered initialization.
- Do not regenerate solely to format code or silence a warning.

## High-risk operations

Require explicit approval before changing option bytes, read/write protection, boot configuration, BOR level, watchdog hardware mode, debug-port availability, or performing destructive erase/recovery. Explain recovery implications before proceeding. Disabling SWD or enabling read protection can brick normal flashing and require recovery procedures.

## Encoding

Keil-side source files are often GBK/GB2312 encoded (Chinese comments). UTF-8 rewrites corrupt them. Detect encoding first; decode with GB2312 before rewriting comment text, or keep ASCII-only edits.
