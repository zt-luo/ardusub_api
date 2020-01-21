# CMakeLists.txt
cmake_minimum_required(VERSION 3.13.0)
project(ardusub VERSION 0.1.0)

# compile options
add_compile_options(-DGLIB_DISABLE_DEPRECATION_WARNINGS
                    # -shared)
                    -static)
# TODO: separate debug options

# src list
set(LIB_SRC_LIST 
    "src/ardusub_interface.c"
    "src/ardusub_io.c"
    "src/ardusub_thread.c"
    "src/ardusub_msg.c"
    "src/ardusub_sqlite.c"
    "src/ardusub_log.c"
    "src/ardusub_ini.c"
    )

# sqlite
pkg_check_modules(SQLITE3 REQUIRED sqlite3)
pkg_search_module(SQLITE3 REQUIRED sqlite3)
include_directories(${SQLITE3_INCLUDE_DIRS})
link_libraries(${SQLITE3_LDFLAGS})

# generate shared library
add_library(ardusub_shared SHARED ${LIB_SRC_LIST})
set_target_properties(ardusub_shared PROPERTIES OUTPUT_NAME  ${PROJECT_NAME})

# generate static library
add_library(ardusub_static STATIC ${LIB_SRC_LIST})
set_target_properties(ardusub_static PROPERTIES OUTPUT_NAME  ${PROJECT_NAME})

# if(CMAKE_SYSTEM_NAME MATCHES "Windows")
# # run post build
# set(Library_Manager_PARTH 
#     "C:\\Program Files (x86)\\Microsoft Visual Studio\\2017\\Community\\VC\\Tools\\MSVC\\14.16.27023\\bin\\Hostx64\\x64\\lib.exe")
# add_custom_command(TARGET ardusub_shared POST_BUILD 
#     COMMAND pexports -v libardusub.dll > libardusub.def
#     COMMAND ${Library_Manager_PARTH} /def:libardusub.def /MACHINE:X64 > NUL)

# endif (CMAKE_SYSTEM_NAME MATCHES "Windows")
