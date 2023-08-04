# CMSIS Build CMakeLists generated on 2023-08-05T00:02:40

cmake_minimum_required(VERSION 3.22)

# Target options

set(TARGET Project)
set(CPU Cortex-M3)
set(PRJ_DIR "d:/OneDrive - email.szu.edu.cn/Codes/Github insiders/Examples/10-2 RNW MPU6050 via Hardware")
set(OUT_DIR "D:/OneDrive - email.szu.edu.cn/Codes/Github insiders/Examples/10-2 RNW MPU6050 via Hardware/out/Project/Target_1")
set(INT_DIR "D:/OneDrive - email.szu.edu.cn/Codes/Github insiders/Examples/10-2 RNW MPU6050 via Hardware/tmp/Project/Target_1")
set(FPU NO_FPU)
set(SECURE Non-secure)
set(BYTE_ORDER Little-endian)
set(OPTIMIZE none)
set(DEBUG on)
set(AS_FLAGS_GLOBAL "-masm=auto")
set(CC_FLAGS_GLOBAL "-fno-rtti -funsigned-char -fshort-enums -fshort-wchar -ffunction-sections -std=c11 -Wno-packed -Wno-missing-variable-declarations -Wno-missing-prototypes -Wno-missing-noreturn -Wno-sign-conversion -Wno-nonportable-include-path -Wno-reserved-id-macro -Wno-unused-macros -Wno-documentation-unknown-command -Wno-documentation -Wno-license-management -Wno-parentheses-equality")
set(CXX_FLAGS_GLOBAL "-fno-rtti -funsigned-char -fshort-enums -fshort-wchar -ffunction-sections -std=c11 -Wno-packed -Wno-missing-variable-declarations -Wno-missing-prototypes -Wno-missing-noreturn -Wno-sign-conversion -Wno-nonportable-include-path -Wno-reserved-id-macro -Wno-unused-macros -Wno-documentation-unknown-command -Wno-documentation -Wno-license-management -Wno-parentheses-equality")
set(LD_FLAGS_GLOBAL "--map --load_addr_map_info --xref --callgraph --symbols --info sizes --info totals --info unused --info veneers --list --strict --summary_stderr --info summarysizes")
set(LD_SCRIPT "d:/OneDrive - email.szu.edu.cn/Codes/Github insiders/Examples/10-2 RNW MPU6050 via Hardware/Project_Target_1.sct")

set(DEFINES
  STM32F10X_MD
  USE_STDPERIPH_DRIVER
)

set(INC_PATHS
  "d:/OneDrive - email.szu.edu.cn/Codes/Github insiders/Examples/10-2 RNW MPU6050 via Hardware/Start"
  "d:/OneDrive - email.szu.edu.cn/Codes/Github insiders/Examples/10-2 RNW MPU6050 via Hardware/Library"
  "d:/OneDrive - email.szu.edu.cn/Codes/Github insiders/Examples/10-2 RNW MPU6050 via Hardware/User"
  "d:/OneDrive - email.szu.edu.cn/Codes/Github insiders/Examples/10-2 RNW MPU6050 via Hardware/System"
  "d:/OneDrive - email.szu.edu.cn/Codes/Github insiders/Examples/10-2 RNW MPU6050 via Hardware/Hardware"
  "C:/Users/Wendo/AppData/Local/Arm/Packs/Keil/STM32F1xx_DFP/2.4.1/Device/Include"
)

set(AS_ARM_SRC_FILES
  "d:/OneDrive - email.szu.edu.cn/Codes/Github insiders/Examples/10-2 RNW MPU6050 via Hardware/Start/startup_stm32f10x_md.s"
)

set(CC_SRC_FILES
  "d:/OneDrive - email.szu.edu.cn/Codes/Github insiders/Examples/10-2 RNW MPU6050 via Hardware/Hardware/IIC.c"
  "d:/OneDrive - email.szu.edu.cn/Codes/Github insiders/Examples/10-2 RNW MPU6050 via Hardware/Hardware/Key.c"
  "d:/OneDrive - email.szu.edu.cn/Codes/Github insiders/Examples/10-2 RNW MPU6050 via Hardware/Hardware/LED.c"
  "d:/OneDrive - email.szu.edu.cn/Codes/Github insiders/Examples/10-2 RNW MPU6050 via Hardware/Hardware/MPU6050.c"
  "d:/OneDrive - email.szu.edu.cn/Codes/Github insiders/Examples/10-2 RNW MPU6050 via Hardware/Hardware/OLED.c"
  "d:/OneDrive - email.szu.edu.cn/Codes/Github insiders/Examples/10-2 RNW MPU6050 via Hardware/Library/misc.c"
  "d:/OneDrive - email.szu.edu.cn/Codes/Github insiders/Examples/10-2 RNW MPU6050 via Hardware/Library/stm32f10x_adc.c"
  "d:/OneDrive - email.szu.edu.cn/Codes/Github insiders/Examples/10-2 RNW MPU6050 via Hardware/Library/stm32f10x_bkp.c"
  "d:/OneDrive - email.szu.edu.cn/Codes/Github insiders/Examples/10-2 RNW MPU6050 via Hardware/Library/stm32f10x_can.c"
  "d:/OneDrive - email.szu.edu.cn/Codes/Github insiders/Examples/10-2 RNW MPU6050 via Hardware/Library/stm32f10x_cec.c"
  "d:/OneDrive - email.szu.edu.cn/Codes/Github insiders/Examples/10-2 RNW MPU6050 via Hardware/Library/stm32f10x_crc.c"
  "d:/OneDrive - email.szu.edu.cn/Codes/Github insiders/Examples/10-2 RNW MPU6050 via Hardware/Library/stm32f10x_dac.c"
  "d:/OneDrive - email.szu.edu.cn/Codes/Github insiders/Examples/10-2 RNW MPU6050 via Hardware/Library/stm32f10x_dbgmcu.c"
  "d:/OneDrive - email.szu.edu.cn/Codes/Github insiders/Examples/10-2 RNW MPU6050 via Hardware/Library/stm32f10x_dma.c"
  "d:/OneDrive - email.szu.edu.cn/Codes/Github insiders/Examples/10-2 RNW MPU6050 via Hardware/Library/stm32f10x_exti.c"
  "d:/OneDrive - email.szu.edu.cn/Codes/Github insiders/Examples/10-2 RNW MPU6050 via Hardware/Library/stm32f10x_flash.c"
  "d:/OneDrive - email.szu.edu.cn/Codes/Github insiders/Examples/10-2 RNW MPU6050 via Hardware/Library/stm32f10x_fsmc.c"
  "d:/OneDrive - email.szu.edu.cn/Codes/Github insiders/Examples/10-2 RNW MPU6050 via Hardware/Library/stm32f10x_gpio.c"
  "d:/OneDrive - email.szu.edu.cn/Codes/Github insiders/Examples/10-2 RNW MPU6050 via Hardware/Library/stm32f10x_i2c.c"
  "d:/OneDrive - email.szu.edu.cn/Codes/Github insiders/Examples/10-2 RNW MPU6050 via Hardware/Library/stm32f10x_iwdg.c"
  "d:/OneDrive - email.szu.edu.cn/Codes/Github insiders/Examples/10-2 RNW MPU6050 via Hardware/Library/stm32f10x_pwr.c"
  "d:/OneDrive - email.szu.edu.cn/Codes/Github insiders/Examples/10-2 RNW MPU6050 via Hardware/Library/stm32f10x_rcc.c"
  "d:/OneDrive - email.szu.edu.cn/Codes/Github insiders/Examples/10-2 RNW MPU6050 via Hardware/Library/stm32f10x_rtc.c"
  "d:/OneDrive - email.szu.edu.cn/Codes/Github insiders/Examples/10-2 RNW MPU6050 via Hardware/Library/stm32f10x_sdio.c"
  "d:/OneDrive - email.szu.edu.cn/Codes/Github insiders/Examples/10-2 RNW MPU6050 via Hardware/Library/stm32f10x_spi.c"
  "d:/OneDrive - email.szu.edu.cn/Codes/Github insiders/Examples/10-2 RNW MPU6050 via Hardware/Library/stm32f10x_tim.c"
  "d:/OneDrive - email.szu.edu.cn/Codes/Github insiders/Examples/10-2 RNW MPU6050 via Hardware/Library/stm32f10x_usart.c"
  "d:/OneDrive - email.szu.edu.cn/Codes/Github insiders/Examples/10-2 RNW MPU6050 via Hardware/Library/stm32f10x_wwdg.c"
  "d:/OneDrive - email.szu.edu.cn/Codes/Github insiders/Examples/10-2 RNW MPU6050 via Hardware/Start/core_cm3.c"
  "d:/OneDrive - email.szu.edu.cn/Codes/Github insiders/Examples/10-2 RNW MPU6050 via Hardware/Start/system_stm32f10x.c"
  "d:/OneDrive - email.szu.edu.cn/Codes/Github insiders/Examples/10-2 RNW MPU6050 via Hardware/System/Delay.c"
  "d:/OneDrive - email.szu.edu.cn/Codes/Github insiders/Examples/10-2 RNW MPU6050 via Hardware/User/main.c"
  "d:/OneDrive - email.szu.edu.cn/Codes/Github insiders/Examples/10-2 RNW MPU6050 via Hardware/User/stm32f10x_it.c"
)

# Toolchain config map

set(REGISTERED_TOOLCHAIN_ROOT "c:/Users/Wendo/.vcpkg/artifacts/fd3d78d7/compilers.arm.armclang/6.20.0/bin")
set(REGISTERED_TOOLCHAIN_VERSION "6.20.0")
set(TOOLCHAIN_VERSION_MIN "6.20.0")
include ("C:/Users/Wendo/.vcpkg/artifacts/fd3d78d7/tools.open.cmsis.pack.cmsis.toolbox/2.0.0/etc/AC6.6.18.0.cmake")
include ("C:/Users/Wendo/.vcpkg/artifacts/fd3d78d7/tools.open.cmsis.pack.cmsis.toolbox/2.0.0/etc/CMSIS-Build-Utils.cmake")

# Setup project

project(${TARGET} LANGUAGES AS_ARM C)

cbuild_get_running_toolchain(TOOLCHAIN_ROOT TOOLCHAIN_VERSION C)

# Global Flags

set(CMAKE_AS_ARM_FLAGS "${AS_ARM_CPU} ${AS_ARM_BYTE_ORDER} ${AS_ARM_DEFINES} ${AS_ARM_OPTIONS_FLAGS} ${AS_ARM_FLAGS} ${AS_FLAGS_GLOBAL}")
cbuild_get_system_includes(CC_SYS_INC_PATHS_LIST CC_SYS_INC_PATHS)
set(CMAKE_C_FLAGS "${CC_CPU} ${CC_BYTE_ORDER} ${CC_DEFINES} ${CC_SECURE} ${CC_OPTIONS_FLAGS} ${CC_FLAGS} ${CC_FLAGS_GLOBAL} ${CC_SYS_INC_PATHS}")
set(CMAKE_C_LINK_FLAGS "${LD_CPU} ${_LS}\"${LD_SCRIPT}\" ${LD_SECURE} ${LD_FLAGS_GLOBAL} ${LD_OPTIONS_FLAGS} ${LD_FLAGS}")

# Local Flags

foreach(SRC ${AS_ARM_SRC_FILES})
  set_source_files_properties(${SRC} PROPERTIES LANGUAGE AS_ARM)
endforeach()

# Compilation Database

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_custom_target(database COMMAND ${CMAKE_COMMAND} -E copy_if_different "${INT_DIR}/compile_commands.json" "${OUT_DIR}")

# Setup Target

add_executable(${TARGET} ${AS_ARM_SRC_FILES} ${CC_SRC_FILES})
set_target_properties(${TARGET} PROPERTIES PREFIX "" SUFFIX ".axf" OUTPUT_NAME "Project")
set_target_properties(${TARGET} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${OUT_DIR} LINK_DEPENDS ${LD_SCRIPT})
target_include_directories(${TARGET} PUBLIC ${INC_PATHS})
