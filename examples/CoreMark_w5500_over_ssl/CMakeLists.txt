set(TARGET_NAME CoreMark_w5500_over_ssl)

add_executable(${TARGET_NAME}
        ${TARGET_NAME}.c
        core_list_join.c
        core_main.c
        core_matrix.c
        core_portme.c
        core_state.c
        core_util.c
        cvt.c
        ee_printf.c
        )

target_link_libraries(${TARGET_NAME} PRIVATE
        pico_stdlib
        hardware_spi
        hardware_dma
	hardware_timer
	hardware_clocks
        FREERTOS_FILES
        ETHERNET_FILES
        IOLIBRARY_FILES
        TIMER_FILES        
        mbedcrypto
        mbedx509
        mbedtls
        )

pico_enable_stdio_usb(${TARGET_NAME} 1)
pico_enable_stdio_uart(${TARGET_NAME} 0)

pico_add_extra_outputs(${TARGET_NAME})
