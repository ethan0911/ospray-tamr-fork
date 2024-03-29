@echo off
rem ======================================================================== rem
rem Copyright 2015-2019 Intel Corporation                                    rem
rem                                                                          rem
rem Licensed under the Apache License, Version 2.0 (the "License");          rem
rem you may not use this file except in compliance with the License.         rem
rem You may obtain a copy of the License at                                  rem
rem                                                                          rem
rem     http://www.apache.org/licenses/LICENSE-2.0                           rem
rem                                                                          rem
rem Unless required by applicable law or agreed to in writing, software      rem
rem distributed under the License is distributed on an "AS IS" BASIS,        rem
rem WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. rem
rem See the License for the specific language governing permissions and      rem
rem limitations under the License.                                           rem
rem ======================================================================== rem

setlocal

md build
cd build

cmake --version

cmake -L ^
-G "%~1" ^
-T "%~2" ^
-D BUILD_JOBS=4 ^
-D BUILD_OSPRAY_CI_TESTS=ON ^
-D BUILD_TBB_FROM_SOURCE=OFF ^
-D BUILD_EMBREE_FROM_SOURCE=OFF ^
-D INSTALL_IN_SEPARATE_DIRECTORIES=OFF ^
../scripts/superbuild

cmake --build . --config Release --target ALL_BUILD -- /m /nologo

:abort
endlocal
:end

rem propagate any error to calling PowerShell script:
exit
