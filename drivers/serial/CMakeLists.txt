if(TARGET drivers__serial)
    set_property(TARGET drivers__serial PROPERTY ALLOW_EMPTY TRUE)
endif()

if(CONFIG_UART_YTM32)
zephyr_library()
zephyr_library_sources(uart_ytm32.c)
endif()