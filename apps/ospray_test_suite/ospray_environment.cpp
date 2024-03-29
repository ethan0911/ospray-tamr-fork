// ======================================================================== //
// Copyright 2017-2019 Intel Corporation                                    //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

#include "ospray_environment.h"

OSPRayEnvironment::OSPRayEnvironment(int argc, char **argv)
    : dumpImg(false),
      rendererType("scivis"),
      deviceType("default"),
      baselineDir("regression_tests/baseline"),
      failedDir("failed")
{
  ParsArgs(argc, argv);
  ospLoadModule("ispc");
  device = ospNewDevice(GetDeviceType().c_str());
  if (device == NULL) {
    std::cout << "Failed to init the ospray device" << std::endl << std::flush;
    std::exit(EXIT_FAILURE);
  }
  ospDeviceCommit(device);
  ospSetCurrentDevice(device);
}

void OSPRayEnvironment::ParsArgs(int argc, char **argv)
{
  std::vector<std::string> testArgs;
  for (int idx = 0; idx < argc; ++idx) {
    testArgs.push_back(argv[idx]);
  }
  for (size_t idx = 0; idx < testArgs.size(); ++idx) {
    if (testArgs.at(idx) == "--help") {
      std::cout << "--help : display this help msg\n"
                << "--dump-img : dump the rendered image to file\n"
                << "--imgsize-x=XX : change the length of an image\n"
                << "--imgsize-y=XX : change the height of an image\n"
                << "--device-type=XX : change the device type which is used by "
                   "OSPRay\n"
                << "--renderer-type=XX : change the renderer used for tests\n"
                << "--baseline-dir=XX : Change the directory used for baseline "
                   "images during tests\n";
      std::exit(EXIT_SUCCESS);
    } else if (testArgs.at(idx) == "--dump-img") {
      dumpImg = true;
    } else if (testArgs.at(idx).find("--renderer-type=") == 0) {
      rendererType = GetStrArgValue(&testArgs.at(idx));
    } else if (testArgs.at(idx).find("--imgsize-x=") == 0) {
      imgSize.x = GetNumArgValue(&testArgs.at(idx));
    } else if (testArgs.at(idx).find("--imgsize-y=") == 0) {
      imgSize.y = GetNumArgValue(&testArgs.at(idx));
    } else if (testArgs.at(idx).find("--device-type=") == 0) {
      deviceType = GetStrArgValue(&testArgs.at(idx));
    } else if (testArgs.at(idx).find("--baseline-dir=") == 0) {
      baselineDir = GetStrArgValue(&testArgs.at(idx));
    } else if (testArgs.at(idx).find("--failed-dir=") == 0) {
      failedDir = GetStrArgValue(&testArgs.at(idx));
    }
  }
}

int OSPRayEnvironment::GetNumArgValue(std::string *arg) const
{
  int ret              = 0;
  size_t valueStartPos = arg->find_first_of('=');
  if (valueStartPos != std::string::npos) {
    ret = std::stoi(arg->substr(valueStartPos + 1));
    if (ret <= 0) {
      std::cout << "Incorrect value specified for argument %s " << *arg
                << std::endl
                << std::flush;
    }
  }
  return ret;
}

std::string OSPRayEnvironment::GetStrArgValue(std::string *arg) const
{
  std::string ret;
  size_t valueStartPos = arg->find_first_of('=');
  if (valueStartPos != std::string::npos) {
    ret = arg->substr(valueStartPos + 1);
  }
  return ret;
}
