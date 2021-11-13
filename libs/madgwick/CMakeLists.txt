file(GLOB_RECURSE MADGWICK_HEADERS CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/inc/*.h
)

file(GLOB_RECURSE MADGWICK_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.s
)

add_library(madgwick STATIC)

set_target_properties(madgwick
    PROPERTIES
        C_STANDARD 11
        C_EXTENSIONS ON
        CMAKE_C_STANDARD_REQUIRED ON
)

target_compile_options(madgwick
    PRIVATE
        #-Wall
        -Wextra
        -Wpedantic
        #-Werror
        -Wstrict-prototypes
        #-Wmissing-prototypes
        -Wduplicated-cond
        -Wduplicated-branches
        -Wlogical-op
        -Wnull-dereference
        -Wshadow
        #-Wconversion
        #-Wsign-conversion
)

target_include_directories(madgwick
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/inc
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(madgwick
    PRIVATE
        hal
)

target_sources(madgwick
    PUBLIC
        ${MADGWICK_HEADERS}
    PRIVATE
        ${MADGWICK_SOURCES}
)
