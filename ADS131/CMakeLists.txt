add_library(ADS131 INTERFACE)
target_include_directories(ADS131 INTERFACE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(ADS131 INTERFACE pico_stdlib hardware_spi)
target_sources(ADS131 INTERFACE 
    ${CMAKE_CURRENT_LIST_DIR}/ADS131m0x.c
    ${CMAKE_CURRENT_LIST_DIR}/ADS131m0x.h
    ${CMAKE_CURRENT_LIST_DIR}/hal.c
    ${CMAKE_CURRENT_LIST_DIR}/hal.h
)