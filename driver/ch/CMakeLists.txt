if(DEFINED LIBXR_CH32_CONFIG_FILE)
    target_compile_definitions(${PROJECT_NAME} PUBLIC LIBXR_CH32_CONFIG_FILE=${LIBXR_CH32_CONFIG_FILE})
else()
    message(FATAL_ERROR "Please set LIBXR_CH32_CONFIG_FILE. e.g. set(LIBXR_CH32_CONFIG_FILE=ch32v30x_conf.h)")
endif()

file(
    GLOB ${PROJECT_NAME}_DRIVER_CPP_SOURCES "${CMAKE_CURRENT_LIST_DIR}/*.cpp")

file(
    GLOB ${PROJECT_NAME}_DRIVER_C_SOURCES "${CMAKE_CURRENT_LIST_DIR}/*.c")

if(LIBXR_CH32_CONFIG_FILE STREQUAL "ch32h417_conf.h")
    list(REMOVE_ITEM ${PROJECT_NAME}_DRIVER_CPP_SOURCES
        ${CMAKE_CURRENT_LIST_DIR}/ch32_usb_devfs.cpp
        ${CMAKE_CURRENT_LIST_DIR}/ch32_usb_endpoint_devfs.cpp
        ${CMAKE_CURRENT_LIST_DIR}/ch32_usb_endpoint_otghs.cpp
        ${CMAKE_CURRENT_LIST_DIR}/ch32_usb_otghs.cpp
        ${CMAKE_CURRENT_LIST_DIR}/ch32_usbcan_shared.cpp)
endif()

target_sources(${PROJECT_NAME}
    PRIVATE ${${PROJECT_NAME}_DRIVER_CPP_SOURCES}
    PRIVATE ${${PROJECT_NAME}_DRIVER_C_SOURCES})

target_include_directories(
    ${PROJECT_NAME}
    PUBLIC ${CMAKE_CURRENT_LIST_DIR})
