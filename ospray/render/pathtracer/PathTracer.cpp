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

#undef NDEBUG

#include "PathTracer.h"
// ospray
#include "common/Data.h"
#include "common/Instance.h"
#include "geometry/GeometricModel.h"
#include "lights/Light.h"
// ispc exports
#include "GeometryLight_ispc.h"
#include "Material_ispc.h"
#include "PathTracer_ispc.h"
// std
#include <map>

namespace ospray {

  PathTracer::PathTracer()
  {
    ispcEquivalent = ispc::PathTracer_create(this);
  }

  PathTracer::~PathTracer()
  {
    destroyGeometryLights();
  }

  std::string PathTracer::toString() const
  {
    return "ospray::PathTracer";
  }

  void PathTracer::generateGeometryLights(const World &world)
  {
    auto *instances = world.instances.ptr;

    if (!instances)
      return;

    for (auto &&instance : instances->as<Instance *>()) {
      auto *geometries = instance->group->geometricModels.ptr;

      if (!geometries)
        return;

      affine3f xfm = instance->xfm();

      for (auto &&model : geometries->as<GeometricModel *>()) {
        if (model->materialList) {
          // check whether the modelmetry has any emissive materials
          bool hasEmissive = false;
          for (auto mat : model->ispcMaterialPtrs) {
            if (mat && ispc::PathTraceMaterial_isEmissive(mat)) {
              hasEmissive = true;
              break;
            }
          }

          if (hasEmissive) {
            if (ispc::GeometryLight_isSupported(model->getIE())) {
              void *light = ispc::GeometryLight_create(model->getIE(), &xfm);

              // check whether the geometry has any emissive primitives
              if (light)
                lightArray.push_back(light);
            } else {
              postStatusMsg(1) << "#osp:pt Geometry " << model->toString()
                               << " does not implement area sampling! "
                               << "Cannot use importance sampling for that "
                               << "geometry with emissive material!";
            }
          }
        }
      }
    }
  }

  void PathTracer::destroyGeometryLights()
  {
    for (size_t i = 0; i < geometryLights; i++)
      ispc::GeometryLight_destroy(lightArray[i]);
  }

  void PathTracer::commit()
  {
    Renderer::commit();

    world = (World *)getParamObject("world");

    destroyGeometryLights();
    lightArray.clear();
    geometryLights = 0;

    const bool useGeometryLights = getParam1b("geometryLights", true);

    if (world && useGeometryLights) {
      generateGeometryLights(*world);
      geometryLights = lightArray.size();
    }

    lightData = (Data *)getParamData("light");
    if (lightData) {
      for (uint32_t i = 0; i < lightData->size(); i++)
        lightArray.push_back(((Light **)lightData->data())[i]->getIE());
    }

    void **lightPtr = lightArray.empty() ? nullptr : &lightArray[0];

    const int32 rouletteDepth = getParam1i("rouletteDepth", 5);
    const float maxRadiance = getParam1f("maxContribution", inf);
    Texture2D *backplate = (Texture2D *)getParamObject("backplate", nullptr);
    vec4f shadowCatcherPlane = getParam4f("shadowCatcherPlane", vec4f(0.f));

    ispc::PathTracer_set(getIE(),
                         rouletteDepth,
                         maxRadiance,
                         backplate ? backplate->getIE() : nullptr,
                         (ispc::vec4f &)shadowCatcherPlane,
                         lightPtr,
                         lightArray.size(),
                         geometryLights);
  }

  OSP_REGISTER_RENDERER(PathTracer, pathtracer);
  OSP_REGISTER_RENDERER(PathTracer, pt);

}  // namespace ospray
