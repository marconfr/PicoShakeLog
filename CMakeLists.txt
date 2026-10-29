# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(PicoShakeLog C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()
include_directories(${CMAKE_CURRENT_LIST_DIR}/include
)


# Add executable. Default name is the project name, version 0.1

add_executable(PicoShakeLog 
    src/main.c 
    src/psl_api.c
    src/psl_shared.c
    src/psl_fifo.c
    src/psl_core0.c
    src/psl_core1.c
)

pico_set_program_name(PicoShakeLog "PicoShakeLog")
pico_set_program_version(PicoShakeLog "0.2.0-rc.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(PicoShakeLog 1)
pico_enable_stdio_usb(PicoShakeLog 1)

# ---------------------------------------------------------------------------
# Debugging helper (Core 1)
# Log verbosity is selected by LOG_LEVEL (see include/psl_log.h), default is
# LOG_LEVEL_INFO. To enable DEBUG output (including the Core1 diagnostics in
# src/psl_core1.c), define it at compile time. Two ways to do it:
#  - Globally for the project (uncomment the line below):
#    add_compile_definitions(LOG_LEVEL=LOG_LEVEL_DEBUG)
#  - Or for this target only (uncomment the preferred line below):
#    target_compile_definitions(PicoShakeLog PRIVATE LOG_LEVEL=LOG_LEVEL_DEBUG)
# ---------------------------------------------------------------------------

#==============================================================================
# ADD USERS LIBRARY
#==============================================================================

#/* -- ICM42688 -- */
add_library(icm42688_driver
    lib/icm42688/icm42688_driver.c
)

target_include_directories(icm42688_driver PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/lib/icm42688
)

target_link_libraries(icm42688_driver
    pico_stdlib
    hardware_spi
    hardware_i2c
)

#/* -- DHCPSERVER -- */
add_library(dhcpserver
    lib/dhcpserver/dhcpserver.c
)

target_include_directories(dhcpserver PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/lib/dhcpserver
)

target_link_libraries(dhcpserver
    pico_stdlib
    pico_cyw43_arch_lwip_threadsafe_background
)
#/* -- DNSSERVER -- */
add_library(dnsserver
    lib/dnsserver/dnsserver.c
)

target_include_directories(dnsserver PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/lib/dnsserver
)

target_link_libraries(dnsserver
    pico_stdlib
    pico_cyw43_arch_lwip_threadsafe_background
)
# Add the standard library to the build
target_link_libraries(PicoShakeLog
        pico_cyw43_arch_lwip_threadsafe_background
        pico_stdlib
        pico_multicore)

# Add the standard include files to the build
target_include_directories(PicoShakeLog PRIVATE
        ${CMAKE_CURRENT_LIST_DIR},
        ${CMAKE_CURRENT_LIST_DIR}/lib
)

# Add any user requested libraries
target_link_libraries(PicoShakeLog 
        icm42688_driver
        dhcpserver
        dnsserver
        hardware_dma
        hardware_spi
        hardware_irq
        )

pico_add_extra_outputs(PicoShakeLog)

