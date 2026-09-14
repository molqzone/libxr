file(GLOB ${PROJECT_NAME}_DRIVER_SOURCES "${CMAKE_CURRENT_LIST_DIR}/*.cpp")

target_sources(${PROJECT_NAME} PRIVATE ${${PROJECT_NAME}_DRIVER_SOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR})

find_package(PkgConfig REQUIRED)

# wpa_client + libnm are needed by linux_wifi_client.hpp and nothing else.  Keep
# them optional: a board whose image deliberately has no NetworkManager (a
# headless flight build, for instance) must still be able to build the rest of
# the Linux drivers.  linux_wifi_client.hpp is guarded by HAVE_WPA_CLIENT, so
# leaving that undefined simply disables the Wi-Fi client.
find_library(WPA_CLIENT_LIB wpa_client)
pkg_check_modules(LIBNM libnm)

if(WPA_CLIENT_LIB AND LIBNM_FOUND)
  message(STATUS "wpa_client + libnm found: enabling the Wi-Fi client")
  target_compile_definitions(xr PUBLIC HAVE_WPA_CLIENT)
  target_link_libraries(xr PUBLIC ${WPA_CLIENT_LIB})
  target_compile_options(xr PUBLIC ${LIBNM_CFLAGS_OTHER})
  target_include_directories(xr PUBLIC ${LIBNM_INCLUDE_DIRS})
  target_link_libraries(xr PUBLIC ${LIBNM_LIBRARIES})
else()
  message(STATUS "wpa_client/libnm not found: Wi-Fi client disabled")
endif()

pkg_check_modules(LIBUDEV REQUIRED libudev)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${LIBUDEV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${LIBUDEV_LIBRARIES})
