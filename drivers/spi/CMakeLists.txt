if(CONFIG_SPI_YTM32)
  zephyr_library()
  # spi_context.h lives in Zephyr's drivers/spi source tree, not in the
  # public include path, so we must add it explicitly.
  zephyr_library_include_directories(${ZEPHYR_BASE}/drivers/spi)
  zephyr_library_sources(
    ytm32_spi_hal.c
    spi_ytm32_transaction.c
    spi_ytm32.c
  )
endif()
