cmake_minimum_required(VERSION 3.13)

if(NOT TARGET ${PROJECT_NAME})
  message(FATAL_ERROR "Need LibXR to expose the EtherCAT headers.")
endif()

# SOES sources, ESC/PDI ports, and object dictionaries are selected by the
# application or by a board driver.
set(LIBXR_ETHERCAT_SOES_DIR "" CACHE PATH
    "Path to the SOES source directory containing ecat_slv.h")
set(LIBXR_ETHERCAT_CONFIG_DIR "" CACHE PATH
    "Path to the application directory containing ecat_options.h")

if(NOT EXISTS "${LIBXR_ETHERCAT_SOES_DIR}/ecat_slv.h")
  message(FATAL_ERROR
          "LIBXR_ETHERCAT_SOES_DIR must point to the SOES source directory containing "
          "ecat_slv.h.")
endif()

if(NOT EXISTS "${LIBXR_ETHERCAT_SOES_DIR}/include/sys/gcc/cc.h")
  message(FATAL_ERROR
          "LIBXR_ETHERCAT_SOES_DIR must provide include/sys/gcc/cc.h for the GNU SOES "
          "port.")
endif()

if(NOT EXISTS "${LIBXR_ETHERCAT_CONFIG_DIR}/ecat_options.h")
  message(FATAL_ERROR
          "LIBXR_ETHERCAT_CONFIG_DIR must point to the application directory containing "
          "ecat_options.h.")
endif()

target_include_directories(${PROJECT_NAME} PUBLIC "${CMAKE_CURRENT_LIST_DIR}")
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE "${LIBXR_ETHERCAT_SOES_DIR}"
          "${LIBXR_ETHERCAT_SOES_DIR}/include/sys/gcc"
          "${LIBXR_ETHERCAT_CONFIG_DIR}"
)
target_sources(${PROJECT_NAME} PRIVATE "${CMAKE_CURRENT_LIST_DIR}/slave_core.cpp")
