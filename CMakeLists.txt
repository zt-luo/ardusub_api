# CMakeLists.txt
cmake_minimum_required(VERSION 3.13.0)
project(ardusub_api VERSION 0.1.0)

set (with_serial YES)

# check platform
if (CMAKE_SYSTEM_NAME MATCHES "Linux")
    message(STATUS "current platform: Linux ")

elseif (CMAKE_SYSTEM_NAME MATCHES "Windows")
    message(STATUS "current platform: Windows")

    if(MINGW)
        message(STATUS "Using MinGW version compiler")
        set (with_serial NO)
        add_compile_definitions(NO_SERISL)
    else(MINGW)
        message(STATUS "NOT a MinGW version compiler!")
        message(FATAL_ERROR "Only support MinGW version compiler!")
    endif(MINGW)

else ()
    message(STATUS "other platform: ${CMAKE_SYSTEM_NAME}")
    message(FATAL_ERROR "NOT supported platform: ${CMAKE_SYSTEM_NAME}!")

endif (CMAKE_SYSTEM_NAME MATCHES "Linux")

if (CMAKE_COMPILER_IS_GNUCC)
message(STATUS "Using gcc for c")
else (CMAKE_COMPILER_IS_GNUCC)
message(FATAL_ERROR "only support gcc!")
endif (CMAKE_COMPILER_IS_GNUCC)

if (CMAKE_COMPILER_IS_GNUCXX)
message(STATUS "Using g++ for cpp")
else (CMAKE_COMPILER_IS_GNUCXX)
message(FATAL_ERROR "only support g++!")
endif (CMAKE_COMPILER_IS_GNUCXX)

add_compile_options(-std=c99
                    -W -Wall -Wextra -Wshadow -Wunreachable-code -Wpointer-arith 
                    -Wcast-qual -Wcast-align 
                    -Wno-varargs -Wno-address-of-packed-member)

if (CMAKE_BUILD_TYPE STREQUAL Release)
add_compile_options(-Wno-maybe-uninitialized -Wno-uninitialized)
endif ()

# check and search moudles
find_package(PkgConfig REQUIRED)
pkg_check_modules(GLIB REQUIRED glib-2.0)
pkg_search_module(GLIB REQUIRED glib-2.0)
pkg_check_modules(GTHREAD REQUIRED gthread-2.0)
pkg_search_module(GTHREAD REQUIRED gthread-2.0)
pkg_check_modules(GIO REQUIRED gio-2.0)
pkg_search_module(GIO REQUIRED gio-2.0)

if (with_serial)
pkg_check_modules(LIBSERIALPORT REQUIRED libserialport)
pkg_search_module(LIBSERIALPORT REQUIRED libserialport)
endif (with_serial)

include_directories(${GLIB_INCLUDE_DIRS} ${GTHREAD_INCLUDE_DIRS} ${GIO_INCLUDE_DIRS} ${LIBSERIALPORT_INCLUDE_DIRS})
link_libraries(${GLIB_LDFLAGS} ${GTHREAD_LDFLAGS} ${GIO_LDFLAGS} ${LIBSERIALPORT_LDFLAGS})


# add mavlink include dir
include_directories("mavlink_c_library_v2")

add_subdirectory(api)
add_subdirectory(examples)
add_subdirectory(tools/msg_decoder)

# set(CPACK_PROJECT_NAME ${PROJECT_NAME})
# set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
# include(CPack)
