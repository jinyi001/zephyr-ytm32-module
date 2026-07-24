.. zephyr:board:: ytm32b1md1_evb

Overview
********

YTM32B1MD1-EVB-Q100 is a minimal evaluation-board definition for the
YTMicro YTM32B1MD1 Arm Cortex-M33 microcontroller.

Hardware
========

- YTM32B1MD1 Arm Cortex-M33 MCU
- 512 KB Flash
- 64 KB SRAM

This board definition intentionally leaves all board peripherals disabled. No
LED, GPIO, UART, watchdog, counter, or other optional board peripheral is
enabled by default. Applications can enable the peripherals they need with a
devicetree overlay.

Supported Features
==================

.. zephyr:board-supported-hw::

Programming and Debugging
*************************

The board uses the J-Link runner with the ``YTM32B1MD14`` device name. No
console is enabled by default.
