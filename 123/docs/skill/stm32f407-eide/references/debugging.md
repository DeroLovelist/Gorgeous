# Debugging Workflow (STM32F407VET6)

Separate failures into: build/link, flash/debug connection, early boot, clock, interrupt/DMA, peripheral protocol (UART/servo), memory/stack, or RTOS scheduling. Reproduce first, collect direct evidence, form one hypothesis at a time, make the smallest diagnostic change.

## Build or link failures

- Read the first causal error, not only the final make/keil failure.
- Check missing sources/includes/definitions after CubeMX regeneration (EIDE does not auto-add new files — see eide-keil-workflow.md).
- For ABI errors, compare CPU, Thumb, FPU, and float-ABI flags on every library.
- For overflow, inspect the map file and linker regions (`SCServo.ld` / `.sct`); do not enlarge a region beyond physical memory.
- For undefined callbacks/handles, check symbol spelling, C/C++ linkage, weak definitions, and source inclusion. Also check `stm32f4xx_hal_conf.h` has the HAL module enabled.

## Cannot connect or flash

Verify target power and common ground, SWDIO/SWCLK wiring, NRST behavior, probe firmware, selected device, interface speed, and whether firmware reconfigured debug pins. Try connect-under-reset before considering erase. Treat option-byte changes and mass erase as approval-gated recovery actions. On-board ST-Link: verify the target voltage rail and that the debug port was not disabled.

## No early boot

Breakpoint in `Reset_Handler`, then `SystemInit`, then `main`. Check vector-table address, stack pointer, reset cause, boot pins, clock start-up timeouts, power flags, and whether execution enters an exception before `main`. Confirm the startup file matches the active toolchain (GCC `.S` for EIDE, ARMCC `.s` for Keil).

## HardFault and related faults

Capture stacked R0-R3, R12, LR, PC, xPSR plus SCB `CFSR`, `HFSR`, `MMFAR`, `BFAR`, and `SHCSR`. Resolve PC/LR against the exact ELF. Check invalid pointers, stack overflow, unaligned/illegal access, bad function pointers, ISR priority misuse, and FPU context assumptions. Do not reset before collecting registers when evidence can be preserved.

## Interrupt and DMA failures

Confirm peripheral event flags, NVIC enable/pending state, handler symbol, vector table, priority, and HAL handle association. For DMA confirm stream/channel, direction, widths, increment modes, request source, flag clearing, and buffer accessibility. DMA cannot access CCM on this device.

## UART / serial-servo (SCServo) failures

- Verify clocks and GPIO alternate functions first (e.g. USART3 on PC10/PC11 AF7), then electrical conditions and protocol timing.
- Confirm baud (115200), servo ID, and the single-wire/two-wire wiring of the Feetech bus (half-duplex TX/RX direction control).
- Compare register state with `.ioc`. Use a logic analyzer or oscilloscope for UART/PWM timing. Distinguish missing signal, incorrect timing, electrical contention, and incorrect higher-level data.
- Servos expect a specific protocol frame (header `0xFF 0xFF`, ID, length, command, checksum via SCServo lib); a wrong checksum or ID explains silent no-response. Test with a single servo (ID=1) before multi-servo sync.

## Peripheral protocol failures (general)

Verify peripheral clocks and GPIO alternate functions first, then electrical conditions and protocol timing. Use a logic analyzer or oscilloscope for UART/SPI/I2C/PWM timing. Distinguish missing signal, incorrect timing, electrical contention, and incorrect higher-level data.

## FreeRTOS failures

Check interrupt priority rules, stack high-water marks, heap failure hooks, task states, blocking calls, and tick configuration. Never call an ISR-unsafe API from an interrupt. Confirm CMSIS-RTOS wrapper behavior matches the bundled FreeRTOS version.
