cmake_minimum_required(VERSION 3.20)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/arm-none-eabi-toolchain.cmake)

project(BLINK VERSION 1.0)

# If you need set actual firmware version and access to it from
# the source code, you may consider using of the following function
# that will create a corresponding 'config.h' file that you could
# include in the project' sources
configure_file(config.h.in ${CMAKE_SOURCE_DIR}/config.h)

set(CMAKE_C_STANDARD 11)
# I'm not using the 20th standard because it deprecates compound assignment with 'volatile'-qualified left operand
# (like `REG |= BIT`), while a lot of HAL's macros use it
set(CMAKE_CXX_STANDARD 17)

set(DEV_FAMILY STM32F103xB)
set(CPU cortex-m3)
set(ARCH armv7-m)
set(HSE_VALUE 8000000UL)

# PATHS =====================================================
set(LIBS_PATH
  ${CMAKE_SOURCE_DIR}
  )
set(HAL_PATH
  ${LIBS_PATH}/stm32f1xx_hal_driver
  )
set(CMSIS_CORE_PATH
  ${LIBS_PATH}/cmsis_core
  )
set(CMSIS_DEV_PATH
  ${LIBS_PATH}/cmsis_device_f1
  )
# ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

# C/C++ SOURCES =============================================
# Here we can not use cool 'file()' function to assing all the source files to HAL_LIBS_SRC
# because the HAL library contains incomprehensible "template" files that redefine some of functions
set(HAL_LIBS_SRC
  ${HAL_PATH}/Src/stm32f1xx_hal.c
  ${HAL_PATH}/Src/stm32f1xx_hal_adc.c
  ${HAL_PATH}/Src/stm32f1xx_hal_adc_ex.c
  ${HAL_PATH}/Src/stm32f1xx_hal_can.c
  ${HAL_PATH}/Src/stm32f1xx_hal_cec.c
  ${HAL_PATH}/Src/stm32f1xx_hal_cortex.c
  ${HAL_PATH}/Src/stm32f1xx_hal_crc.c
  ${HAL_PATH}/Src/stm32f1xx_hal_dac.c
  ${HAL_PATH}/Src/stm32f1xx_hal_dac_ex.c
  ${HAL_PATH}/Src/stm32f1xx_hal_dma.c
  ${HAL_PATH}/Src/stm32f1xx_hal_eth.c
  ${HAL_PATH}/Src/stm32f1xx_hal_exti.c
  ${HAL_PATH}/Src/stm32f1xx_hal_flash.c
  ${HAL_PATH}/Src/stm32f1xx_hal_flash_ex.c
  ${HAL_PATH}/Src/stm32f1xx_hal_gpio.c
  ${HAL_PATH}/Src/stm32f1xx_hal_gpio_ex.c
  ${HAL_PATH}/Src/stm32f1xx_hal_hcd.c
  ${HAL_PATH}/Src/stm32f1xx_hal_i2c.c
  ${HAL_PATH}/Src/stm32f1xx_hal_i2s.c
  ${HAL_PATH}/Src/stm32f1xx_hal_irda.c
  ${HAL_PATH}/Src/stm32f1xx_hal_iwdg.c
  ${HAL_PATH}/Src/stm32f1xx_hal_mmc.c
  ${HAL_PATH}/Src/stm32f1xx_hal_nand.c
  ${HAL_PATH}/Src/stm32f1xx_hal_nor.c
  ${HAL_PATH}/Src/stm32f1xx_hal_pccard.c
  ${HAL_PATH}/Src/stm32f1xx_hal_pcd.c
  ${HAL_PATH}/Src/stm32f1xx_hal_pcd_ex.c
  ${HAL_PATH}/Src/stm32f1xx_hal_pwr.c
  ${HAL_PATH}/Src/stm32f1xx_hal_rcc.c
  ${HAL_PATH}/Src/stm32f1xx_hal_rcc_ex.c
  ${HAL_PATH}/Src/stm32f1xx_hal_rtc.c
  ${HAL_PATH}/Src/stm32f1xx_hal_rtc_ex.c
  ${HAL_PATH}/Src/stm32f1xx_hal_sd.c
  ${HAL_PATH}/Src/stm32f1xx_hal_smartcard.c
  ${HAL_PATH}/Src/stm32f1xx_hal_spi.c
  ${HAL_PATH}/Src/stm32f1xx_hal_sram.c
  ${HAL_PATH}/Src/stm32f1xx_hal_tim.c
  ${HAL_PATH}/Src/stm32f1xx_hal_tim_ex.c
  ${HAL_PATH}/Src/stm32f1xx_hal_uart.c
  ${HAL_PATH}/Src/stm32f1xx_hal_usart.c
  ${HAL_PATH}/Src/stm32f1xx_hal_wwdg.c
  )
file(GLOB PRJ_SRC CONFIGURE_DEPENDS
  ${CMAKE_SOURCE_DIR}/*.c
  ${CMAKE_SOURCE_DIR}/*.cc
  ${CMAKE_SOURCE_DIR}/src/*.c
  ${CMAKE_SOURCE_DIR}/src/*.cc
  )
# ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

# INCLUDE DIRs ==============================================
set(HAL_LIBS_INC
  ${HAL_PATH}/Inc
  )
set(CMSIS_CORE_LIBS_INC
  ${CMSIS_CORE_PATH}/Include
  )
set(CMSIS_DEV_LIBS_INC
  ${CMSIS_DEV_PATH}/Include
  )
set(PRJ_INC
  ${CMAKE_SOURCE_DIR}/
  ${CMAKE_SOURCE_DIR}/src/
  )
# ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

# LINKER SCRIPT =============================================
set(LINKER_SCRIPT
  ${CMAKE_SOURCE_DIR}/STM32F103xB.ld
  )
# ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

add_definitions(
  -DHSE_VALUE=${HSE_VALUE}
  -D${DEV_FAMILY}
  -DUSE_HAL_DRIVER
  )

add_executable(${PROJECT_NAME}.elf
  ${HAL_LIBS_SRC}
  # This file provides two functions SystemInit() and SystemCoreClockUpdate().
  ${CMSIS_DEV_PATH}/Source/Templates/system_stm32f1xx.c
  ${PRJ_SRC}
  )

include_directories(
  ${HAL_LIBS_INC}
  ${CMSIS_CORE_LIBS_INC}
  ${CMSIS_DEV_LIBS_INC}
  ${PRJ_INC}
  )

target_compile_options(${PROJECT_NAME}.elf PRIVATE
  -Wno-psabi
  -fdata-sections
  -ffunction-sections
  -Wl,--gc-sections
  -ffreestanding
  -Wall
  -Wpedantic
  -mcpu=${CPU}
  -march=${ARCH}
  -mlittle-endian
  -mthumb
  -masm-syntax-unified
  -fno-exceptions
  -fno-unwind-tables
  )

target_link_options(${PROJECT_NAME}.elf PRIVATE
  -T${LINKER_SCRIPT}
  -Wl,-Map=${PROJECT_NAME}.map,--cref
  -mthumb
  -mcpu=${CPU}
  -march=${ARCH}
  -specs=nosys.specs
  -specs=nano.specs
  -lc
  -lm
  -static
  -lnosys
  -Wl,--gc-sections
  -Wl,--print-memory-usage
  )

add_custom_command(TARGET ${PROJECT_NAME}.elf
  POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
  COMMAND ${CMAKE_OBJCOPY} -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
  COMMAND ${CMAKE_OBJDUMP} -S ${PROJECT_NAME}.elf > ${PROJECT_NAME}.lss
  COMMAND ${CMAKE_SIZE_UTIL} -B ${PROJECT_NAME}.elf
  COMMENT "Generating ${PROJECT_NAME}.hex, ${PROJECT_NAME}.bin"
  )
