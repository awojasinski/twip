file(GLOB_RECURSE UART_HEADERS CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/inc/*.h
)

file(GLOB_RECURSE UART_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.s
)

add_library(uart STATIC)

set_target_properties(uart
    PROPERTIES
        C_STANDARD 11
        C_EXTENSIONS ON
        CMAKE_C_STANDARD_REQUIRED ON
)

target_compile_options(uart
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
        -Werror
        -Wstrict-prototypes
        -Wmissing-prototypes
        -Wduplicated-cond
        -Wduplicated-branches
        -Wlogical-op
        -Wnull-dereference
        -Wshadow
        -Wconversion
        -Wsign-conversion
)

target_include_directories(uart
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/inc
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(uart
    PRIVATE
        hal
        fifo
        cli
)

target_sources(uart
    PUBLIC
        ${UART_HEADERS}
    PRIVATE
        ${UART_SOURCES}
)