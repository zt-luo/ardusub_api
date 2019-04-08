# CMakeLists.txt
cmake_minimum_required(VERSION 3.13.0)
project(ardusub_api VERSION 0.1.0)

set(DEBUG_WITH_VS "no")



if (DEBUG_WITH_VS MATCHES "yes")

link_directories("C:/gstreamer/1.0/x86_64/lib")
link_libraries(glib-2.0 intl gthread-2.0 gobject-2.0 json-glib-1.0 gio-2.0)
include_directories("C:/gstreamer/1.0/x86_64/include/glib-2.0" 
                    "C:/gstreamer/1.0/x86_64/include/json-glib-1.0"
                    "C:/gstreamer/1.0/x86_64/lib/glib-2.0/include")
                    
else ()

add_compile_options(-std=c99
                    -W -Wall)

# check and search moudles
find_package(PkgConfig REQUIRED)
pkg_check_modules(GLIB REQUIRED glib-2.0)
pkg_search_module(GLIB REQUIRED glib-2.0)
pkg_check_modules(GTHREAD REQUIRED gthread-2.0)
pkg_search_module(GTHREAD REQUIRED gthread-2.0)
pkg_check_modules(GIO REQUIRED gio-2.0)
pkg_search_module(GIO REQUIRED gio-2.0)
pkg_check_modules(LIBSERIALPORT REQUIRED libserialport)
pkg_search_module(LIBSERIALPORT REQUIRED libserialport)

include_directories(${GLIB_INCLUDE_DIRS} ${GTHREAD_INCLUDE_DIRS} ${GIO_INCLUDE_DIRS} ${LIBSERIALPORT_INCLUDE_DIRS})
link_libraries(${GLIB_LDFLAGS} ${GTHREAD_LDFLAGS} ${GIO_LDFLAGS} ${LIBSERIALPORT_LDFLAGS})

endif (DEBUG_WITH_VS MATCHES "yes")


# add mavlink include dir
set(MAVLINK_V2_0_INC_DIR "mavlink_c_library_v2")
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

ADD_SUBDIRECTORY(api)
ADD_SUBDIRECTORY(sqlite)
ADD_SUBDIRECTORY(examples)
ADD_SUBDIRECTORY(tools/msg_decoder)

# set(CPACK_PROJECT_NAME ${PROJECT_NAME})
# set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
# include(CPack)
