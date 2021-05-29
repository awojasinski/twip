file(GLOB_RECURSE INA219_HEADERS CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/inc/*.h
)

file(GLOB_RECURSE INA219_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.s
)

add_library(ina219 OBJECT)

set_target_properties(ina219
    PROPERTIES
        C_STANDARD 11
        C_EXTENSIONS ON
        CMAKE_C_STANDARD_REQUIRED ON
)

target_compile_options(ina219
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
        -Wno-unused-function
)

target_include_directories(ina219
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/inc
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(ina219
    PRIVATE
        hal
)

target_sources(ina219
    PUBLIC
        ${INA219_HEADERS}
    PRIVATE
        ${INA219_SOURCES}
)