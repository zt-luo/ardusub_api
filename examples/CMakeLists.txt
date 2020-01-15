# CMakeLists.txt
cmake_minimum_required(VERSION 3.13.0)
project(examples VERSION 0.1.0)

add_compile_options(-g -O0 
                    # -Wno-unused-parameter
                    -static)
                    # -shared)
# TODO: add different compile options for different compiler


set(ARDUSUB_API_LIB_INC_DIR "../api/inc")

include_directories(${ARDUSUB_API_LIB_INC_DIR})
link_libraries(ardusub_static)
# link_libraries(ardusub_shared)

add_executable(simple_example "simple_example.c")

add_executable(depth_pid "depth_pid.c")

add_executable(depth_hold "depth_hold.c")

if(with_serial)
add_executable(libserialport_example "libserialport_example.c")
endif(with_serial)
