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

//ospray
#include "Light.h"
#include "common/Util.h"
#include "Light_ispc.h"

namespace ospray {

  void Light::commit()
  {
    color     = getParam3f("color", vec3f(1.f));
    intensity = getParam1f("intensity", 1.f);
    isVisible = getParam1b("visible", true);

    ispc::Light_set(getIE(), (ispc::vec3f &)color, intensity, isVisible);
  }

  std::string Light::toString() const
  {
    return "ospray::Light";
  }

  Light *Light::createInstance(const char *type)
  {
    return createInstanceHelper<Light, OSP_LIGHT>(type);
  }

}
