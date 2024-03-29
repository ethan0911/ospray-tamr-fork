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

// ospray
#include "Geometry.h"
#include "common/Data.h"
#include "common/Util.h"

namespace ospray {

  Geometry::Geometry()
  {
    managedObjectType = OSP_GEOMETRY;
  }

  std::string Geometry::toString() const
  {
    return "ospray::Geometry";
  }

  Geometry *Geometry::createInstance(const char *type)
  {
    return createInstanceHelper<Geometry, OSP_GEOMETRY>(type);
  }

  void Geometry::postCreationInfo(size_t numVerts) const
  {
    std::stringstream ss;
    ss << toString() << " created: #primitives=" << numPrimitives();
    if (numVerts > 0)
      ss << ", #vertices=" << numVerts;
    postStatusMsg(2) << ss.str();
  }

}  // namespace ospray
