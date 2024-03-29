## ======================================================================== ##
## Copyright 2009-2019 Intel Corporation                                    ##
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

cmake_minimum_required(VERSION 3.1)

project(ospray_test_suite)

# Baseline images for tests and benchmarks
include(ExternalData)

set(ExternalData_URL_TEMPLATES
  "http://sdvis.org/ospray/download/baseline/test-data/%(algo)/%(hash)"
  "http://sdvis.org/ospray/download/baseline/benchmark-data/%(algo)/%(hash)"
)

set(ExternalData_OBJECT_STORES
  "${CMAKE_BINARY_DIR}/regression_test_baseline"
  "/opt/ospray/test-data"
  "//sdvis-nas/NAS/OSPRay/test-data"
)

set(ExternalData_BINARY_ROOT ${CMAKE_BINARY_DIR}/regression_test_baseline)
set(ExternalData_SOURCE_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/baseline)

ExternalData_Expand_Arguments(ospray_test_data
  ospray_test_suite
  DATA{${ExternalData_SOURCE_ROOT}/,REGEX:.*}
)

ExternalData_Add_Target(ospray_test_data)

set_target_properties(ospray_test_data PROPERTIES EXCLUDE_FROM_ALL TRUE)
