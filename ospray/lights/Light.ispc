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

#include "Light.ih"

Light_EvalRes Light_eval(const uniform Light* uniform,
                         const DifferentialGeometry&,
                         const vec3f&,
                         const float,
                         const float)
{
  Light_EvalRes res;
  res.radiance = make_vec3f(0.f);
  return res;
}


// Exports (called from C++)
//////////////////////////////////////////////////////////////////////////////

//! Set the parameters of an ispc-side Light object
export void Light_set(void *uniform _self,
                      const uniform vec3f &color,
                      const uniform float intensity,
                      const uniform bool isVisible)
{
  Light *uniform self = (Light * uniform) _self;

  self->color     = color;
  self->intensity = intensity;
  self->radiance  = color * intensity;
  self->isVisible = isVisible;
}

