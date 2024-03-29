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
#include "Isosurfaces.h"
#include "common/Data.h"
#include "common/World.h"
// ispc-generated files
#include "Isosurfaces_ispc.h"

namespace ospray {

  std::string Isosurfaces::toString() const
  {
    return "ospray::Isosurfaces";
  }

  void Isosurfaces::commit()
  {
    isovaluesData = getParamDataT<float>("isovalue", true);

    volume = (VolumetricModel *)getParamObject("volume");

    if (isovaluesData->stride() != sizeof(float)) {
      // get rid of stride
      auto data = new Data(OSP_FLOAT, vec3ui(isovaluesData->size(), 1, 1));
      data->copy(*isovaluesData, vec3ui(0));
      isovaluesData = &(data->as<float>());
      data->refDec();
    }

    postCreationInfo();
  }

  size_t Isosurfaces::numPrimitives() const
  {
    return isovaluesData->size();
  }

  LiveGeometry Isosurfaces::createEmbreeGeometry()
  {
    LiveGeometry retval;

    retval.ispcEquivalent = ispc::Isosurfaces_create(this);
    retval.embreeGeometry =
        rtcNewGeometry(ispc_embreeDevice(), RTC_GEOMETRY_TYPE_USER);

    ispc::Isosurfaces_set(retval.ispcEquivalent,
        retval.embreeGeometry,
        isovaluesData->size(),
        isovaluesData->data(),
        volume->getIE());

    return retval;
  }

  OSP_REGISTER_GEOMETRY(Isosurfaces, isosurfaces);

}  // namespace ospray
