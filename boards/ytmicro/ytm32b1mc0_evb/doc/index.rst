.. zephyr:board:: ytm32b1mc0_evb

Overview
********

YTM32B1MC0-EVB-Q64 is an evaluation board for the YTMicro YTM32B1MC0
Arm Cortex-M33 microcontroller.

Hardware
********

- YTM32B1MC0 Arm Cortex-M33 MCU @ 80 MHz
- 256 KB Flash
- 32 KB SRAM

Supported Features
==================

.. zephyr:board-supported-hw::

Programming and Debugging
*************************

.. zephyr:board-supported-runners::

Flashing
========

Here is an example for the :zephyr:code-sample:`hello_world` application.

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: ytm32b1mc0_evb
   :goals: flash

Debugging
=========

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: ytm32b1mc0_evb
   :goals: debug
