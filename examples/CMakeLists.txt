# CMakeLists.txt
cmake_minimum_required(VERSION 3.13.0)
project(simple_example VERSION 0.1.0)

add_compile_options(-g -O0 -W -static)
# TODO: add different compile options for different compiler

set(ARDUSUB_API_LIB_INC_DIR "../lib/inc")

set(SRC_LIST "simple_example.c")

include_directories(${ARDUSUB_API_LIB_INC_DIR})
link_libraries(ardusub_static)

add_executable(${PROJECT_NAME} ${SRC_LIST})
