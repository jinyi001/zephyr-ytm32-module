if(TARGET drivers__watchdog)
	set_property(TARGET drivers__watchdog PROPERTY ALLOW_EMPTY TRUE)
endif()

if(CONFIG_WDT_YTM32)
zephyr_library()
zephyr_library_sources(wdt_ytm32.c)
endif()
