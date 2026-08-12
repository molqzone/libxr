cmake_minimum_required(VERSION 3.13)

if(NOT TARGET xr)
  message(FATAL_ERROR "Need LibXR to build XRECAT.")
endif()

set(XRECAT_SOES_DIR "" CACHE PATH "Path to the SOES source tree used by XRECAT")

if(NOT EXISTS "${XRECAT_SOES_DIR}/ecat_slv.h")
  message(FATAL_ERROR
    "XRECAT requires SOES. Set XRECAT_SOES_DIR to the directory containing ecat_slv.h.")
endif()

file(GLOB_RECURSE XRECAT_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_LIST_DIR}/*.c"
)
list(SORT XRECAT_SOURCES)

target_sources(xr PRIVATE ${XRECAT_SOURCES})

target_include_directories(xr PUBLIC
  "${CMAKE_CURRENT_LIST_DIR}"
  "${CMAKE_CURRENT_LIST_DIR}/rm"
  "${XRECAT_SOES_DIR}"
)
