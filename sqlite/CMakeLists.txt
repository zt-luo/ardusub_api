# CMakeLists.txt
cmake_minimum_required(VERSION 3.13.0)
project(sqlite3 VERSION 0.1.0)

add_compile_options(-g -O0
                    -w
                    -static)

set(SQLITE_SRC_LIST "sqlite3.c")

# generate shared library
add_library(sqlite_shared SHARED ${SQLITE_SRC_LIST})
set_target_properties(sqlite_shared PROPERTIES OUTPUT_NAME  ${PROJECT_NAME})

# generate static library
add_library(sqlite_static STATIC ${SQLITE_SRC_LIST})
set_target_properties(sqlite_static PROPERTIES OUTPUT_NAME  ${PROJECT_NAME})
