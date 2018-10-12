// ======================================================================== //
// Copyright 2009-2018 Intel Corporation                                    //
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

// sg
#include "Volume.h"

namespace ospray {
  namespace sg {

    /*! a plain old structured volume */
    struct OSPSG_INTERFACE UnstructuredVolume : public Volume
    {
      UnstructuredVolume();

      std::string toString() const override;

      void preCommit(RenderContext &ctx) override;

      TimeStamp vertexFieldTime;
      TimeStamp cellFieldTime;
    };

  } // ::ospray::sg
} // ::ospray