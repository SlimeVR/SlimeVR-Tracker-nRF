#
# Copyright (c) 2023 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#
cmake_minimum_required(VERSION 3.20.0)

# Configure VERSION from repository branch and tag
find_package(Git)

if(GIT_EXECUTABLE)
  execute_process(
    COMMAND ${GIT_EXECUTABLE} branch --show-current
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_BRANCH
    )
  execute_process(
    COMMAND ${GIT_EXECUTABLE} describe --dirty --tags --long --match [0-9]*.[0-9]*.[0-9]*
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_VERSION
    )
endif()

if(GIT_BRANCH STREQUAL "")
  set(GIT_BRANCH unknown)
  message(WARNING "Unknown Git Branch")
endif()

if(GIT_VERSION STREQUAL "")
  set(GIT_VERSION 0.0.0-0)
  message(WARNING "Unknown Tag/Version")
endif()

string(STRIP ${GIT_BRANCH} GIT_BRANCH)
string(STRIP ${GIT_VERSION} GIT_VERSION)
string(REPLACE "-" ";" GIT_VERSION ${GIT_VERSION})
list(GET GIT_VERSION 0 GIT_RELEASE)
list(GET GIT_VERSION 1 GIT_TWEAK)
string(REPLACE "v" "" GIT_RELEASE ${GIT_RELEASE})
string(REPLACE "." ";" GIT_RELEASE ${GIT_RELEASE})
list(GET GIT_RELEASE 0 GIT_MAJOR)
list(GET GIT_RELEASE 1 GIT_MINOR)
list(GET GIT_RELEASE 2 GIT_PATCH)

configure_file(${CMAKE_SOURCE_DIR}/VERSION_SRC ${CMAKE_SOURCE_DIR}/VERSION @ONLY)

# Zephyr
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(NONE)

# NORDIC SDK APP START
FILE(GLOB_RECURSE app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})

# NORDIC SDK APP END

FILE(GLOB_RECURSE files vqf-c/src/*.c)
target_sources(app PRIVATE ${files})

target_include_directories(app PRIVATE src)
target_include_directories(app PRIVATE vqf-c/src)

# Force recompile for updated build timestamp
add_custom_target(touch_util_h ALL
    COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_SOURCE_DIR}/src/build_defines.h
)

# clangd wants compile_commands.json in the root of build directory
# try to find the build root clangd will be happy with
cmake_path(GET CMAKE_CURRENT_BINARY_DIR PARENT_PATH build_dir)
cmake_path(COMPARE ${CMAKE_CURRENT_LIST_DIR} NOT_EQUAL ${build_dir} build_dir_is_subdirectory)

if(build_dir_is_subdirectory)
  add_custom_target(link_target ALL
    COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_BINARY_DIR}/compile_commands.json ${build_dir}/compile_commands.json)
endif()
