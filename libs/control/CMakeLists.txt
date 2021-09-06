file(GLOB_RECURSE CONTROL_HEADERS CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/inc/*.h
)

file(GLOB_RECURSE CONTROL_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.s
)

add_library(control STATIC)

set_target_properties(control
    PROPERTIES
        C_STANDARD 11
        C_EXTENSIONS ON
        CMAKE_C_STANDARD_REQUIRED ON
)

target_compile_options(control
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

target_include_directories(control
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/inc
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(control
    PRIVATE
        hal
        logger
        encoder
)

target_sources(control
    PUBLIC
        ${CONTROL_HEADERS}
    PRIVATE
        ${CONTROL_SOURCES}
)
