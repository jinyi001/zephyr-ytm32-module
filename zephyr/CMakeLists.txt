# Make dt-bindings and other public headers available to C compilation.
# (dts_root only covers DTS preprocessing, not the C compiler.)
zephyr_include_directories(${ZEPHYR_CURRENT_MODULE_DIR}/include)

add_subdirectory(${ZEPHYR_CURRENT_MODULE_DIR}/drivers drivers)