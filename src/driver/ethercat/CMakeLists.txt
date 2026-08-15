cmake_minimum_required(VERSION 3.13)

if(NOT TARGET ${PROJECT_NAME})
  message(FATAL_ERROR "Need LibXR to expose the EtherCAT headers.")
endif()

target_include_directories(${PROJECT_NAME} PUBLIC "${CMAKE_CURRENT_LIST_DIR}")
target_sources(
  ${PROJECT_NAME}
  PRIVATE "${CMAKE_CURRENT_LIST_DIR}/core/object_dictionary.cpp"
          "${CMAKE_CURRENT_LIST_DIR}/device/device_pool.cpp"
          "${CMAKE_CURRENT_LIST_DIR}/device/device_composition.cpp"
          "${CMAKE_CURRENT_LIST_DIR}/device/device_core.cpp"
)

option(XRECAT_TEST_BUILD "Build XRECAT host protocol tests" OFF)
if(XRECAT_TEST_BUILD)
  enable_testing()
  add_executable(xrecat_device_core_test "${CMAKE_CURRENT_LIST_DIR}/test/device_core_test.cpp")
  target_link_libraries(xrecat_device_core_test PRIVATE ${PROJECT_NAME})
  add_test(NAME xrecat_device_core_test COMMAND xrecat_device_core_test)
endif()
