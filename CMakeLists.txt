cmake_minimum_required(VERSION 3.13)

if(NOT TARGET ${PROJECT_NAME})
  message(FATAL_ERROR "Need LibXR to expose the EtherCAT headers.")
endif()

# SOES sources, ESC/PDI ports, and object dictionaries are selected by the
# application or by a board driver.
target_include_directories(${PROJECT_NAME} PUBLIC "${CMAKE_CURRENT_LIST_DIR}")
target_sources(${PROJECT_NAME} PRIVATE "${CMAKE_CURRENT_LIST_DIR}/slave_core.cpp")
