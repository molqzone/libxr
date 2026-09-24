# Fail the build when a generated XRobot entry header is older than its inputs.
#
# Run in script mode:  cmake -DHEADERS="a.hpp|b.hpp" -P XRobotStampCheck.cmake
#
# xrobot_gen_main writes "// xrobot-stamp: config=<path> sha256=<hex>" and a matching
# "lock=" line; paths are relative to the header. Hashes use LF line endings so
# Windows and Linux checkouts of the same file agree. Headers without a stamp
# (hand-written entries, Module compile probes) are not checked.

function(_xrobot_normalized_sha256 path out)
  file(READ "${path}" _content)
  string(REPLACE "\r\n" "\n" _content "${_content}")
  string(SHA256 _digest "${_content}")
  set(${out} "${_digest}" PARENT_SCOPE)
endfunction()

string(REPLACE "|" ";" _headers "${HEADERS}")
foreach(_header IN LISTS _headers)
  if(NOT EXISTS "${_header}")
    continue()
  endif()
  get_filename_component(_base "${_header}" DIRECTORY)
  file(STRINGS "${_header}" _stamps LIMIT_COUNT 8 REGEX "^// xrobot-stamp: (config|lock)=")
  foreach(_stamp IN LISTS _stamps)
    if(NOT _stamp MATCHES "^// xrobot-stamp: (config|lock)=([^ ]+) sha256=([0-9a-f]+)")
      continue()
    endif()
    set(_kind "${CMAKE_MATCH_1}")
    set(_input "${_base}/${CMAKE_MATCH_2}")
    set(_expected "${CMAKE_MATCH_3}")
    if(NOT EXISTS "${_input}")
      message(FATAL_ERROR "[XRobot] ${_header} was generated from ${_input}, which no longer exists. Regenerate it with xrobot_gen_main.")
    endif()
    _xrobot_normalized_sha256("${_input}" _actual)
    if(NOT _actual STREQUAL _expected)
      message(FATAL_ERROR
        "[XRobot] ${_header} is stale: its ${_kind} input ${_input} changed after generation "
        "(sha256 ${_expected} -> ${_actual}). Regenerate it with xrobot_gen_main "
        "(or xrobot_setup) before building.")
    endif()
  endforeach()
endforeach()
