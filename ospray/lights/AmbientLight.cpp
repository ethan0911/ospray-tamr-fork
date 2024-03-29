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

#include "AmbientLight.h"
#include "AmbientLight_ispc.h"

namespace ospray {

  AmbientLight::AmbientLight()
  {
    ispcEquivalent = ispc::AmbientLight_create();
  }

  std::string AmbientLight::toString() const
  {
    return "ospray::AmbientLight";
  }

  void AmbientLight::commit()
  {
    Light::commit();
    ispc::AmbientLight_set(getIE());
  }

  OSP_REGISTER_LIGHT(AmbientLight, AmbientLight);
  OSP_REGISTER_LIGHT(AmbientLight, ambient);

} // ::ospray
