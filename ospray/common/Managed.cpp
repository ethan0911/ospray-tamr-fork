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

#include "Managed.h"
#include "OSPCommon_ispc.h"

namespace ospray {

  ManagedObject::~ManagedObject()
  {
    ispc::delete_uniform(ispcEquivalent);
    ispcEquivalent = nullptr;

    std::for_each(params_begin(),
                  params_end(),
                  [&](std::shared_ptr<Param> &p) {
                    auto &param = *p;
                    if (param.data.is<OSP_PTR>()) {
                      auto *obj = param.data.get<OSP_PTR>();
                      if (obj != nullptr) obj->refDec();
                    }
                  });
  }

  void ManagedObject::commit()
  {
  }

  std::string ManagedObject::toString() const
  {
    return "ospray::ManagedObject";
  }

#define define_getparam(T,ABB)                                      \
  T ManagedObject::getParam##ABB(const char *name, T valIfNotFound) \
  {                                                                 \
    return getParam<T>(name, valIfNotFound);                        \
  }

  define_getparam(ManagedObject *, Object)
  define_getparam(std::string,     String)
  define_getparam(void*,           VoidPtr)

  define_getparam(int32,  1i)
  define_getparam(vec3i,  3i)
  define_getparam(vec3f,  3f)
  define_getparam(vec4f,  4f)
  define_getparam(vec2f,  2f)
  define_getparam(float,  1f)
  define_getparam(bool,   1b)

  define_getparam(affine3f, Affine3f)

#undef define_getparam

} // ::ospray
