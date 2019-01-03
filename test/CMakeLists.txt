# CMakeLists.txt
cmake_minimum_required(VERSION 3.0.0)
project(ardusub_api_test VERSION 0.1.0)

add_compile_options(-g -O0 -W -static)
# TODO: add different compile options for different compiler

set(ARDUSUB_API_LIB_INC_DIR "../include")

set(SRC_LIST "main.c")

include_directories(${ARDUSUB_API_LIB_INC_DIR})
link_libraries(ardusub_static)

add_executable(${PROJECT_NAME} ${SRC_LIST})
