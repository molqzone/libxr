cmake_minimum_required(VERSION 3.13)

if(NOT TARGET ${PROJECT_NAME})
  message(FATAL_ERROR "Need LibXR to expose the EtherCAT headers.")
endif()

target_include_directories(${PROJECT_NAME} PUBLIC "${CMAKE_CURRENT_LIST_DIR}")
target_sources(
  ${PROJECT_NAME}
  PRIVATE "${CMAKE_CURRENT_LIST_DIR}/core/object_dictionary.cpp"
          "${CMAKE_CURRENT_LIST_DIR}/device/slave_pool.cpp"
          "${CMAKE_CURRENT_LIST_DIR}/device/slave_composition.cpp"
          "${CMAKE_CURRENT_LIST_DIR}/device/slave_core.cpp"
)
