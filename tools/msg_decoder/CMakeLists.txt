# CMakeLists.txt
cmake_minimum_required(VERSION 3.13.0)
project(decoder VERSION 0.1.0)

add_compile_options(-g -O0 
                    # -Wno-unused-parameter 
                    -Wno-cast-qual
                    -static)

pkg_check_modules(JSON_GLIB REQUIRED json-glib-1.0)
pkg_search_module(JSON_GLIB REQUIRED json-glib-1.0)

include_directories(${JSON_GLIB_INCLUDE_DIRS})
link_libraries(${JSON_GLIB_LDFLAGS})

set(SRC_LIST "decoder.c")

add_executable(${PROJECT_NAME} ${SRC_LIST})
