file(GLOB _xr_mspm0_driver_sources CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_LIST_DIR}/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/*.c"
)
list(SORT _xr_mspm0_driver_sources)

target_sources(${PROJECT_NAME} PRIVATE ${_xr_mspm0_driver_sources})
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR})
