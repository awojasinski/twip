file(GLOB_RECURSE HAL_HEADERS CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/*.h
)

file(GLOB_RECURSE HAL_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/*.s
)

add_library(hal OBJECT)

target_include_directories(hal
    SYSTEM PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Inc
        ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Inc/Legacy
        ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32G4xx/Include
        ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Include
        ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc
        ${CMAKE_CURRENT_SOURCE_DIR}/FATFS/App
        ${CMAKE_CURRENT_SOURCE_DIR}/FATFS/Target
        ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FatFs/src
)

target_compile_definitions(hal
    PUBLIC
        USE_HAL_DRIVER
        STM32G431xx
)

target_sources(hal
    PUBLIC
        ${HAL_HEADERS}
    PRIVATE
        ${HAL_SOURCES}
)

utils_target_set_linker_script(hal ${CMAKE_CURRENT_SOURCE_DIR}/STM32G431KBTx_FLASH.ld)
