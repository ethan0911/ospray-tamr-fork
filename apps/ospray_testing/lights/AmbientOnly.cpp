// ======================================================================== //
// Copyright 2009-2019 Intel Corporation                                    //
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

#include "Lights.h"

namespace ospray {
  namespace testing {

    struct AmbientOnly : public Lights
    {
      ~AmbientOnly() override = default;

      OSPData createLights() const override;
    };

    OSPData AmbientOnly::createLights() const
    {
      auto ambientLight = ospNewLight("ambient");

      ospSetFloat(ambientLight, "intensity", 1.25f);
      ospSetVec3f(ambientLight, "color", 1.f, 1.f, 1.f);
      ospCommit(ambientLight);

      auto lightsData = ospNewData(1, OSP_LIGHT, &ambientLight);
      ospRelease(ambientLight);

      return lightsData;
    }

    OSP_REGISTER_TESTING_LIGHTS(AmbientOnly, ambient);
    OSP_REGISTER_TESTING_LIGHTS(AmbientOnly, ambient_only);

  }  // namespace testing
}  // namespace ospray
