## ======================================================================== ##
## Copyright 2019 Intel Corporation                                         ##
##                                                                          ##
## Licensed under the Apache License, Version 2.0 (the "License");          ##
## you may not use this file except in compliance with the License.         ##
## You may obtain a copy of the License at                                  ##
##                                                                          ##
##     http://www.apache.org/licenses/LICENSE-2.0                           ##
##                                                                          ##
## Unless required by applicable law or agreed to in writing, software      ##
## distributed under the License is distributed on an "AS IS" BASIS,        ##
## WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. ##
## See the License for the specific language governing permissions and      ##
## limitations under the License.                                           ##
## ======================================================================== ##

# NOTE: This CMakeLists.txt is intended to be used to exercise an OSPRay
#       install and demonstrate how external applications can build against
#       OSPRay using CMake.
#
#       Once you have done a 'make install' of an existing OSPRay
#       build (will install to CMAKE_INSTALL_PREFIX), create a separate build
#       directory and invoke CMake on this directory. If you have 'ospray_DIR'
#       setup correctly to point to where you just installed OSPRay, then this
#       should build the ospTutorial app from that install and NOT use your
#       local build.

cmake_minimum_required(VERSION 3.1)

project(ospTutorial LANGUAGES C)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})

find_package(ospray REQUIRED)

add_executable(ospTutorial ${CMAKE_CURRENT_SOURCE_DIR}/../ospTutorial.c)
target_link_libraries(ospTutorial ospray::ospray)
