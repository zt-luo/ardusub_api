# CMakeLists.txt
cmake_minimum_required(VERSION 3.13.0)
project(simple_example VERSION 0.1.0)

if (DEBUG_WITH_VS matches "yes")

link_directories("C:/gstreamer/1.0/x86_64/lib")

else ()

add_compile_options(-g -O0 
                    # -Wno-unused-parameter
                    -static)
# TODO: add different compile options for different compiler

endif (DEBUG_WITH_VS matches "yes")

set(ARDUSUB_API_LIB_INC_DIR "../lib/inc")

set(SRC_LIST "simple_example.c")

include_directories(${ARDUSUB_API_LIB_INC_DIR})
link_libraries(ardusub_static)
# link_libraries(ardusub_shared)

add_executable(${PROJECT_NAME} ${SRC_LIST})
