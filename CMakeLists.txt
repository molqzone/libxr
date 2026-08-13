cmake_minimum_required(VERSION 3.13)

if(NOT TARGET xr)
  message(FATAL_ERROR "Need LibXR to build EtherCAT.")
endif()

set(LIBXR_ETHERCAT_SOES_DIR "" CACHE PATH
    "Path to the SOES soes directory containing ecat_slv.h")
set(LIBXR_ETHERCAT_PROFILE "rm" CACHE STRING "EtherCAT slave profile")
set_property(CACHE LIBXR_ETHERCAT_PROFILE PROPERTY STRINGS rm foc)

if(NOT EXISTS "${LIBXR_ETHERCAT_SOES_DIR}/ecat_slv.h")
  message(FATAL_ERROR
    "EtherCAT requires SOES. Set LIBXR_ETHERCAT_SOES_DIR to the directory containing ecat_slv.h.")
endif()

# XRECAT supplies the LibXR facade and board port; SOES supplies the EtherCAT slave
# state machine. Keep the dependency external so XRECAT can follow upstream SOES.
set(LIBXR_ETHERCAT_SOES_SOURCES
  "${LIBXR_ETHERCAT_SOES_DIR}/ecat_slv.c"
  "${LIBXR_ETHERCAT_SOES_DIR}/esc.c"
  "${LIBXR_ETHERCAT_SOES_DIR}/esc_coe.c"
  "${LIBXR_ETHERCAT_SOES_DIR}/esc_eep.c"
)

if(LIBXR_ETHERCAT_PROFILE STREQUAL "foc")
  list(APPEND LIBXR_ETHERCAT_SOES_SOURCES
    "${LIBXR_ETHERCAT_SOES_DIR}/esc_foe.c"
  )
endif()

foreach(LIBXR_ETHERCAT_SOES_SOURCE IN LISTS LIBXR_ETHERCAT_SOES_SOURCES)
  if(NOT EXISTS "${LIBXR_ETHERCAT_SOES_SOURCE}")
    message(FATAL_ERROR "SOES source is missing: ${LIBXR_ETHERCAT_SOES_SOURCE}")
  endif()
endforeach()

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
file(GLOB LIBXR_ETHERCAT_SLAVE_CPP CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_LIST_DIR}/device/slave/*.cpp"
)
list(APPEND LIBXR_ETHERCAT_SOURCES
  ${LIBXR_ETHERCAT_SOES_SOURCES}
  ${LIBXR_ETHERCAT_PORT_C}
  ${LIBXR_ETHERCAT_SLAVE_C}
  ${LIBXR_ETHERCAT_SLAVE_CPP}
  "${LIBXR_ETHERCAT_PROFILE_DIR}/profile.c"
  "${LIBXR_ETHERCAT_PROFILE_DIR}/objectlist.c"
)
list(SORT LIBXR_ETHERCAT_SOURCES)

target_sources(xr PRIVATE ${LIBXR_ETHERCAT_SOURCES})

target_include_directories(xr PUBLIC
  "${CMAKE_CURRENT_LIST_DIR}"
  "${CMAKE_CURRENT_LIST_DIR}/core"
  "${CMAKE_CURRENT_LIST_DIR}/device/slave"
  "${CMAKE_CURRENT_LIST_DIR}/device/peripheral"
  "${CMAKE_CURRENT_LIST_DIR}/device/slave/profile"
  "${LIBXR_ETHERCAT_PROFILE_DIR}"
  "${CMAKE_CURRENT_LIST_DIR}/port/hpm"
  "${LIBXR_ETHERCAT_SOES_DIR}"
  "${LIBXR_ETHERCAT_SOES_DIR}/include/sys/gcc"
)
