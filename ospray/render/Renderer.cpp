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
#include "Renderer.h"
#include "common/Instance.h"
#include "common/Util.h"
#include "geometry/GeometricModel.h"
// ispc exports
#include "Renderer_ispc.h"
// ospray
#include "LoadBalancer.h"

namespace ospray {

  std::string Renderer::toString() const
  {
    return "ospray::Renderer";
  }

  void Renderer::commit()
  {
    spp                         = std::max(1, getParam1i("spp", 1));
    const int32 maxDepth        = std::max(0, getParam1i("maxDepth", 20));
    const float minContribution = getParam1f("minContribution", 0.001f);
    errorThreshold              = getParam1f("varianceThreshold", 0.f);

    maxDepthTexture = (Texture2D *)getParamObject("maxDepthTexture", nullptr);

    if (maxDepthTexture) {
      if (maxDepthTexture->format != OSP_TEXTURE_R32F
          || maxDepthTexture->filter != OSP_TEXTURE_FILTER_NEAREST) {
        static WarnOnce warning(
            "maxDepthTexture provided to the renderer "
            "needs to be of type OSP_TEXTURE_R32F and have "
            "the OSP_TEXTURE_FILTER_NEAREST flag");
      }
    }

    vec3f bgColor3 = getParam3f("bgColor", vec3f(getParam1f("bgColor", 0.f)));
    bgColor        = getParam4f("bgColor", vec4f(bgColor3, 0.f));

    if (getIE()) {
      ispc::Renderer_set(getIE(),
                         spp,
                         maxDepth,
                         minContribution,
                         (ispc::vec4f &)bgColor,
                         maxDepthTexture ? maxDepthTexture->getIE() : nullptr);
    }
  }

  Renderer *Renderer::createInstance(const char *type)
  {
    return createInstanceHelper<Renderer, OSP_RENDERER>(type);
  }

  void Renderer::renderTile(FrameBuffer *fb,
                            Camera *camera,
                            World *world,
                            void *perFrameData,
                            Tile &tile,
                            size_t jobID) const
  {
    ispc::Renderer_renderTile(getIE(),
                              fb->getIE(),
                              camera->getIE(),
                              world->getIE(),
                              perFrameData,
                              (ispc::Tile &)tile,
                              jobID);
  }

  void *Renderer::beginFrame(FrameBuffer *fb, World *world)
  {
    fb->beginFrame();
    return ispc::Renderer_beginFrame(getIE(), world->getIE());
  }

  void Renderer::endFrame(FrameBuffer *fb, void *perFrameData)
  {
    fb->setCompletedEvent(OSP_FRAME_FINISHED);
    ispc::Renderer_endFrame(getIE(), perFrameData);
  }

  float Renderer::renderFrame(FrameBuffer *fb, Camera *camera, World *world)
  {
    return TiledLoadBalancer::instance->renderFrame(fb, this, camera, world);
  }

  OSPPickResult Renderer::pick(FrameBuffer *fb,
                               Camera *camera,
                               World *world,
                               const vec2f &screenPos)
  {
    OSPPickResult res;

    res.instance = nullptr;
    res.model    = nullptr;
    res.primID   = -1;

    int instID = -1;
    int geomID = -1;
    int primID = -1;

    ispc::Renderer_pick(getIE(),
                        fb->getIE(),
                        camera->getIE(),
                        world->getIE(),
                        (const ispc::vec2f &)screenPos,
                        (ispc::vec3f &)res.worldPosition[0],
                        instID,
                        geomID,
                        primID,
                        res.hasHit);

    if (res.hasHit) {
      auto *instance = world->instances->as<Instance *>()[instID];
      auto *group    = instance->group.ptr;
      auto *model = group->geometricModels->as<GeometricModel *>()[geomID];

      res.instance = (OSPInstance)instance;
      res.model    = (OSPGeometricModel)model;
      res.primID   = static_cast<uint32_t>(primID);
    }

    return res;
  }

}  // namespace ospray
