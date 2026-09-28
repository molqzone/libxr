# Fail the build when the generated XRobot entry is older than one of its inputs.
#
# Run in script mode:
#   cmake -DXROBOT_MAIN_HEADER=<header> -DXROBOT_BSP_ROOT=<dir> -DXROBOT_CONFIG=<config>
#         -DXROBOT_INPUTS="<a>|<b>" -P XRobotFreshness.cmake
#
# XROBOT_INPUTS holds the absolute config and depends paths read from the header's
# "// xrobot:" lines at configure time; XROBOT_CONFIG is the config path relative to
# XROBOT_BSP_ROOT. Only modification times are compared. Equal times count as stale.
# The header is never regenerated here.

if(NOT EXISTS "${XROBOT_MAIN_HEADER}")
  message(
    FATAL_ERROR
      "[XRobot] ${XROBOT_MAIN_HEADER} is missing. "
      "Run `xrobot gen -c ${XROBOT_CONFIG}` in ${XROBOT_BSP_ROOT}."
  )
endif()

string(REPLACE "|" ";" _inputs "${XROBOT_INPUTS}")
foreach(_input IN LISTS _inputs)
  if(NOT EXISTS "${_input}")
    message(
      FATAL_ERROR
        "[XRobot] ${XROBOT_MAIN_HEADER} was generated from ${_input}, which no longer "
        "exists. Run `xrobot gen -c ${XROBOT_CONFIG}` in ${XROBOT_BSP_ROOT}."
    )
  endif()
  if("${_input}" IS_NEWER_THAN "${XROBOT_MAIN_HEADER}")
    message(
      FATAL_ERROR
        "[XRobot] ${XROBOT_MAIN_HEADER} is stale: ${_input} is newer. "
        "Run `xrobot gen -c ${XROBOT_CONFIG}` in ${XROBOT_BSP_ROOT} before building."
    )
  endif()
endforeach()
