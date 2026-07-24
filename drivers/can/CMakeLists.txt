if(CONFIG_CAN_YTM32)
  zephyr_library()
  zephyr_library_sources(
    ytm32_can_hal.c
    can_ytm32.c
  )
endif()
