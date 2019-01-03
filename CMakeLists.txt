# CMakeLists.txt
cmake_minimum_required(VERSION 3.0.0)
project(ardusub_api VERSION 0.1.0)

add_compile_options(-DGLIB_DISABLE_DEPRECATION_WARNINGS -g -O0 -W -static)

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
ADD_SUBDIRECTORY(test)


# if(WIN32)
#   target_link_libraries(${PROJECT_NAME} wsock32 ws2_32)
# endif()

# set(CPACK_PROJECT_NAME ${PROJECT_NAME})
# set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
# include(CPack)
