cmake_minimum_required(VERSION 3.13)

if(NOT TARGET ${PROJECT_NAME})
  message(FATAL_ERROR "Need LibXR to expose the EtherCAT headers.")
endif()

# XRECAT is header-only. SOES sources, ESC/PDI ports, callbacks, and object
# dictionaries are selected by the application or by a board driver.
target_include_directories(${PROJECT_NAME} PUBLIC "${CMAKE_CURRENT_LIST_DIR}")

