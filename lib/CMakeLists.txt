# CMakeLists.txt
cmake_minimum_required(VERSION 3.0.0)
project(ardusub VERSION 0.1.0)
add_compile_options(-DGLIB_DISABLE_DEPRECATION_WARNINGS -g -O0 -W -static)
# TODO: separate debug options

set(MAVLINK_V2_0_INC_DIR "../mavlink/include/mavlink/v2.0")

set(LIB_SRC_LIST "ardusub_interface.c")

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

add_library(ardusub_shared SHARED ${LIB_SRC_LIST})
SET_TARGET_PROPERTIES(ardusub_shared PROPERTIES OUTPUT_NAME  ${PROJECT_NAME})

add_library(ardusub_static STATIC ${LIB_SRC_LIST})
SET_TARGET_PROPERTIES(ardusub_static PROPERTIES OUTPUT_NAME  ${PROJECT_NAME})
