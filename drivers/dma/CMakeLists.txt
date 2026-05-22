if(CONFIG_DMA_YTM32)
  zephyr_library_sources(
    ytm32_dma_hal.c
    dma_ytm32.c
  )
endif()
