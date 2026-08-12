cmake_minimum_required(VERSION 3.13)

if(NOT TARGET xr)
  message(FATAL_ERROR "Need LibXR to build EtherCAT.")
endif()

set(LIBXR_ETHERCAT_SOES_DIR "" CACHE PATH "Path to the SOES source tree used by EtherCAT")
set(LIBXR_ETHERCAT_PROFILE "rm" CACHE STRING "EtherCAT slave profile")
set_property(CACHE LIBXR_ETHERCAT_PROFILE PROPERTY STRINGS rm foc)

if(NOT EXISTS "${LIBXR_ETHERCAT_SOES_DIR}/ecat_slv.h")
  message(FATAL_ERROR
    "EtherCAT requires SOES. Set LIBXR_ETHERCAT_SOES_DIR to the directory containing ecat_slv.h.")
endif()

set(LIBXR_ETHERCAT_PROFILE_DIR
  "${CMAKE_CURRENT_LIST_DIR}/device/slave/profile/${LIBXR_ETHERCAT_PROFILE}"
)

if(NOT EXISTS "${LIBXR_ETHERCAT_PROFILE_DIR}/profile.c")
  message(FATAL_ERROR "Unknown EtherCAT profile: ${LIBXR_ETHERCAT_PROFILE}")
endif()

file(GLOB_RECURSE LIBXR_ETHERCAT_PORT_C CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_LIST_DIR}/core/*.c"
  "${CMAKE_CURRENT_LIST_DIR}/port/hpm/*.c"
)
file(GLOB LIBXR_ETHERCAT_SLAVE_C CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_LIST_DIR}/device/slave/*.c"
)
list(APPEND LIBXR_ETHERCAT_ALL_C
  ${LIBXR_ETHERCAT_PORT_C}
  ${LIBXR_ETHERCAT_SLAVE_C}
  "${LIBXR_ETHERCAT_PROFILE_DIR}/profile.c"
  "${LIBXR_ETHERCAT_PROFILE_DIR}/objectlist.c"
)
list(SORT LIBXR_ETHERCAT_ALL_C)

target_sources(xr PRIVATE ${LIBXR_ETHERCAT_ALL_C})

target_include_directories(xr PUBLIC
  "${CMAKE_CURRENT_LIST_DIR}"
  "${CMAKE_CURRENT_LIST_DIR}/core"
  "${CMAKE_CURRENT_LIST_DIR}/device/slave"
  "${CMAKE_CURRENT_LIST_DIR}/device/slave/profile"
  "${LIBXR_ETHERCAT_PROFILE_DIR}"
  "${CMAKE_CURRENT_LIST_DIR}/port/hpm"
  "${LIBXR_ETHERCAT_SOES_DIR}"
)
