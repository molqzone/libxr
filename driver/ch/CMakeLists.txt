if(DEFINED LIBXR_CH32_CONFIG_FILE)
    target_compile_definitions(${PROJECT_NAME} PUBLIC LIBXR_CH32_CONFIG_FILE=${LIBXR_CH32_CONFIG_FILE})
else()
    message(FATAL_ERROR "Please set LIBXR_CH32_CONFIG_FILE. e.g. set(LIBXR_CH32_CONFIG_FILE=ch32v30x_conf.h)")
endif()

file(
    GLOB ${PROJECT_NAME}_DRIVER_CPP_SOURCES "${CMAKE_CURRENT_LIST_DIR}/*.cpp")

file(
    GLOB ${PROJECT_NAME}_DRIVER_C_SOURCES "${CMAKE_CURRENT_LIST_DIR}/*.c")

if("${LIBXR_CH32_CONFIG_FILE}" STREQUAL "ch32x035_conf.h")
    set(${PROJECT_NAME}_DRIVER_CPP_SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/ch32_dma.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/ch32_gpio.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/ch32_i2c.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/ch32_i2c_def.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/ch32_timebase.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/ch32_uart.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/ch32_uart_def.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/ch32_usbcan_shared.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/ch32_usb_endpoint_devfs.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/ch32_usb_devfs.cpp")
endif()

target_sources(${PROJECT_NAME}
    PRIVATE ${${PROJECT_NAME}_DRIVER_CPP_SOURCES}
    PRIVATE ${${PROJECT_NAME}_DRIVER_C_SOURCES})

target_include_directories(
    ${PROJECT_NAME}
    PUBLIC ${CMAKE_CURRENT_LIST_DIR})
