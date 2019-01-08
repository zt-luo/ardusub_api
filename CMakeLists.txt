# CMakeLists.txt
cmake_minimum_required(VERSION 3.13.0)
project(ardusub_api VERSION 0.1.0)

# check and search moudles
find_package(PkgConfig REQUIRED)
pkg_check_modules(GLIB REQUIRED glib-2.0)
pkg_search_module(GLIB REQUIRED glib-2.0)
pkg_check_modules(GTHREAD REQUIRED gthread-2.0)
pkg_search_module(GTHREAD REQUIRED gthread-2.0)
pkg_check_modules(GIO REQUIRED gio-2.0)
pkg_search_module(GIO REQUIRED gio-2.0)

include_directories(${GLIB_INCLUDE_DIRS} ${GTHREAD_INCLUDE_DIRS} ${GIO_INCLUDE_DIRS})
link_libraries(${GLIB_LDFLAGS} ${GTHREAD_LDFLAGS} ${GIO_LDFLAGS})

# add mavlink include dir
set(MAVLINK_V2_0_INC_DIR "mavlink/v2.0")
include_directories(${MAVLINK_V2_0_INC_DIR})

IF (CMAKE_SYSTEM_NAME MATCHES "Linux")
    MESSAGE(STATUS "current platform: Linux ")

ELSEIF (CMAKE_SYSTEM_NAME MATCHES "Windows")
    MESSAGE(STATUS "current platform: Windows")

ELSEIF (CMAKE_SYSTEM_NAME MATCHES "FreeBSD")
    MESSAGE(STATUS "current platform: FreeBSD")

ELSE ()
    MESSAGE(STATUS "other platform: ${CMAKE_SYSTEM_NAME}")

ENDIF (CMAKE_SYSTEM_NAME MATCHES "Linux")

ADD_SUBDIRECTORY(lib)
ADD_SUBDIRECTORY(examples)
ADD_SUBDIRECTORY("tools/msg decoder")

# set(CPACK_PROJECT_NAME ${PROJECT_NAME})
# set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
# include(CPack)
