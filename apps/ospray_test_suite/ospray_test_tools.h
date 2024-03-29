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

#pragma once

#include <stdint.h>
#include <stdio.h>
#include <cmath>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <ospray/ospray.h>
#include <limits>

#include "ospray_environment.h"

#include "stb_image.h"
#include "stb_image_write.h"

using pixelColorValue = unsigned char;

const pixelColorValue pixelThreshold = 10;
const float errorRate                = 0.03;

enum class OsprayStatus
{
  Ok,
  Fail,
  Error,
};

enum ImgType
{
  Y = 1,
  YA,
  RGB,
  RGBA,
};

class OSPImageTools
{
 protected:
  vec2i size;
  std::string fileFormat;
  std::string imgName;

  // helper method to write the rendered image as PPM file
  OsprayStatus writePPM(std::string fileName, const uint32_t *pixel);
  // helper method to write the rendered image as PNG file
  OsprayStatus writePNG(std::string fileName, const uint32_t *pixel);
  // helper method to write the rendered image as HDR file
  OsprayStatus writeHDR(std::string fileName, const float *pixel);
  // helper method to write the image with given format
  OsprayStatus writeImg(std::string fileName, const void *pixel);
  std::string GetFileFormat() const
  {
    return fileFormat;
  }

 public:
  OSPImageTools(vec2i imgSize,
                std::string testName,
                OSPFrameBufferFormat frameBufferFormat);
  ~OSPImageTools();

  // helper method to saved rendered file
  OsprayStatus saveTestImage(const void *pixel);
  // helper method to compare gold image with current framebuffer render
  OsprayStatus compareImgWithBaseline(const uint32_t *testImg);
};
