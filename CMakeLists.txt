# CMakeLists.txt
cmake_minimum_required(VERSION 3.0.0)
project(main VERSION 0.1.0)

add_compile_options(-DGLIB_DISABLE_DEPRECATION_WARNINGS -g -O0 -W -static)

set(MAVLINK_V2_0_INC_DIR "mavlink/include/mavlink/v2.0")

set(SRC_LIST "main.c" "ardusub_interface.c")

find_package(PkgConfig REQUIRED)
pkg_check_modules(GLIB REQUIRED glib-2.0)
pkg_search_module(GLIB REQUIRED glib-2.0)
pkg_check_modules(GTHREAD REQUIRED gthread-2.0)
pkg_search_module(GTHREAD REQUIRED gthread-2.0)
pkg_check_modules(GIO REQUIRED gio-2.0)
pkg_search_module(GIO REQUIRED gio-2.0)

include_directories(${GLIB_INCLUDE_DIRS} ${GTHREAD_INCLUDE_DIRS} ${MAVLINK_V2_0_INC_DIR})
link_libraries(${GLIB_LDFLAGS} ${GTHREAD_LDFLAGS} ${GIO_LDFLAGS})
# target_link_libraries(${PROJECT_NAME} ${GLIB_LIBRARIES} ${GTHREAD_LIBRARIES})

IF (CMAKE_SYSTEM_NAME MATCHES "Linux")
    MESSAGE(STATUS "current platform: Linux ")

ELSEIF (CMAKE_SYSTEM_NAME MATCHES "Windows")
    MESSAGE(STATUS "current platform: Windows")

ELSEIF (CMAKE_SYSTEM_NAME MATCHES "FreeBSD")
    MESSAGE(STATUS "current platform: FreeBSD")

ELSE ()
    MESSAGE(STATUS "other platform: ${CMAKE_SYSTEM_NAME}")

ENDIF (CMAKE_SYSTEM_NAME MATCHES "Linux")

add_executable(${PROJECT_NAME} ${SRC_LIST})

if(WIN32)
  target_link_libraries(${PROJECT_NAME} wsock32 ws2_32)
endif()

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
