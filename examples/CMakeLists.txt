# CMakeLists.txt
cmake_minimum_required(VERSION 3.13.0)
project(examples VERSION 0.1.0)

if (DEBUG_WITH_VS MATCHES "yes")

else ()

add_compile_options(-g -O0 
                    # -Wno-unused-parameter
                    -static)
# TODO: add different compile options for different compiler

endif (DEBUG_WITH_VS MATCHES "yes")

set(ARDUSUB_API_LIB_INC_DIR "../lib/inc")

set(SIMPLE_SRC_LIST "simple_example.c")

include_directories(${ARDUSUB_API_LIB_INC_DIR})
link_libraries(ardusub_static)
# link_libraries(ardusub_shared)

add_executable(simple_example ${SIMPLE_SRC_LIST})

pkg_check_modules(LIBSERIALPORT REQUIRED libserialport)
pkg_search_module(LIBSERIALPORT REQUIRED libserialport)

include_directories(${LIBSERIALPORT_INCLUDE_DIRS})
link_libraries(${LIBSERIALPORT_LDFLAGS})

set(SERIAL_SRC_LIST "libserialport_example.c")
add_executable(libserialport_example ${SERIAL_SRC_LIST})
