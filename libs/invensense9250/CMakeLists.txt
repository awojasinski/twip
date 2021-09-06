file(GLOB_RECURSE INVENSENSE_HEADERS CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/mpl/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/mllite/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/eMPL-hal/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/driver/stm32G/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/driver/include/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/driver/eMPL/*.h
)

file(GLOB_RECURSE INVENSENSE_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/mpl/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/mllite/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/eMPL-hal/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/driver/stm32G/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/driver/include/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/driver/eMPL/*.c
)

add_compile_definitions(
    EMPL
    EMPL_TARGET_STM32
    MPU9250
    USE_DMP
    MPL_LOG_NDEBUG=1
)

add_library(invensense OBJECT)

set_target_properties(invensense
    PROPERTIES
        C_STANDARD 11
        C_EXTENSIONS ON
        CMAKE_C_STANDARD_REQUIRED ON
)

target_include_directories(invensense
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/
        ${CMAKE_CURRENT_SOURCE_DIR}/mpl/
        ${CMAKE_CURRENT_SOURCE_DIR}/mllite/
        ${CMAKE_CURRENT_SOURCE_DIR}/eMPL-hal/
        ${CMAKE_CURRENT_SOURCE_DIR}/driver/stm32G/
        ${CMAKE_CURRENT_SOURCE_DIR}/driver/include/
        ${CMAKE_CURRENT_SOURCE_DIR}/driver/eMPL/
)

target_link_libraries(invensense
    PRIVATE
        hal
        cli
        ${CMAKE_CURRENT_SOURCE_DIR}/mpl/libmpllib.a
)

target_sources(invensense
    PUBLIC
        ${INVENSENSE_HEADERS}
    PRIVATE
        ${INVENSENSE_SOURCES}
)