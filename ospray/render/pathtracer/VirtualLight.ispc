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

#include "render/pathtracer/VirtualLight.ih"
#include "render/pathtracer/PathStructs.ih"
#include "render/pathtracer/PathTracerUtil.ih"
#include "render/pathtracer/volumes/VolumeSampler.ih"

inline box1f getMinMaxDistForVirtualLights(const PathContext& pathContext,
                                           const PathState& pathState,
                                           const PathVertex& lastVertex,
                                           const PathVertex& pathVertex,
                                           const Ray& ray)
{
  box1f interval;

  // minimal distance is not always zero (when previously sampled a Dirac transmission)
  interval.lower = distance(lastVertex.dg.P, ray.org);

  // virtual lights are occluded by hit geometry because lastVertex.dg.P can be
  // different from ray.org (when previously sampled a Dirac transmission) we
  // cannot just use ray.t as maximum distance
  interval.upper = distance(lastVertex.dg.P, ray.org + ray.t * ray.dir);

  if (pathVertex.type == ENVIRONMENT) {
    interval.upper = inf;
    if (pathState.straightPath && (bool)pathContext.context->backplate) 
      interval.upper = 1e38; // backplate hides envLights (i.e. the ones at infinity)
  }
  return interval;
}

vec3f evaluateVirtualLights(const PathContext& pathContext, 
                            const PathState& pathState, 
                            const PathVertex& lastVertex, 
                            const PathVertex& pathVertex, 
                            Ray& ray)
{
  // add light from *virtual* lights by intersecting them
  vec3f L = make_vec3f(0.f);

  box1f intervalLightDist; // [minLightDist, maxLightDist] for virtual light visibility
  if (pathContext.numLights > pathContext.context->numGeoLights)
    intervalLightDist = getMinMaxDistForVirtualLights(pathContext, pathState, lastVertex, pathVertex, ray);

  for (uniform int i = pathContext.context->numGeoLights; i < pathContext.numLights; i++) 
  {
    const uniform Light *uniform light = pathContext.context->lights[i];
    if (!pathState.straightPath || light->isVisible) {
      // to correctly handle MIS through transparencies the light pdf needs to be calculated wrt. lastVertex.dg
      // however, we only have a valid intersection with the light in [minLightDist, maxLightDist],
      // otherwise light could be added twice
      Light_EvalRes le = light->eval(light, lastVertex.dg, ray.dir, intervalLightDist.lower, intervalLightDist.upper);
      if (reduce_max(le.radiance) > 0.0f) 
      {            
        Ray shadowRay;
        setRay(shadowRay, lastVertex.dg.P, ray.dir, intervalLightDist.lower, intervalLightDist.upper, ray.time);
        const float T = volumeTransmittance(pathState, pathContext.world, shadowRay, pathContext.ldSampler, pathContext.randomSampler);
        L = L + pathState.throughput * le.radiance * misHeuristic(pathState, lastVertex.pdf_w * T, le.pdf);
      }
    }
  }
  return L;
}