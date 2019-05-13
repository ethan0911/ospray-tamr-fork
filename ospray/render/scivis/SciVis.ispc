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

#include "render/Renderer.ih"
// ao renderer
#include "SciVis.ih"
#include "surfaces.ih"
#include "volumes.ih"

void SciVis_renderSample(Renderer *uniform _self,
                         FrameBuffer *uniform fb,
                         World *uniform world,
                         void *uniform perFrameData,
                         varying ScreenSample &sample)
{
  uniform SciVis *uniform self = (uniform SciVis * uniform) _self;

  const uniform vec4f &bgColor = self->super.bgColor;

  vec4f outputColor = make_vec4f(0.f);

  uniform bool firstHit       = true;
  const float originalRayTFar = sample.ray.t;

  // NOTE(jda) - On AVX CPUs, removing the iteration counter can cause infinite
  //             loops (seen stochastically in CI)...needs more attention, may
  //             end up being a compiler bug.
  uniform uint32 niters = 0;

  while (outputColor.w < 0.99f && niters < 1000) {
    Ray &geomRay  = sample.ray;
    Ray volumeRay = sample.ray;

    if (geomRay.t0 < geomRay.t)
      traceRay(world, geomRay);

    VolumeInterval volumeInterval;
    traceVolumeRay(world, volumeRay, volumeInterval);

    const bool haveGeometryHit = hadHit(geomRay);
    const bool haveVolumeHit   = volumeInterval.hasInterval;

    const bool bothHit   = haveGeometryHit && haveVolumeHit;
    const bool eitherHit = haveGeometryHit || haveVolumeHit;

    const bool volumeFirst = volumeInterval.interval.x < geomRay.t;

    cif(firstHit)
    {
      sample.z = geomRay.t;
      if (!(eitherHit)) {
        outputColor   = bgColor;
        sample.albedo = make_vec3f(bgColor);
        sample.normal = sample.ray.dir;
      }
    }

    if (!eitherHit)
      break;

    SSI surfaceShading;

    vec4f volumeColor   = make_vec4f(0.f);
    vec4f &surfaceColor = surfaceShading.shadedColor;

    DifferentialGeometry dg;
    if (haveGeometryHit) {
      computeDG(world, geomRay, dg);
      surfaceShading = computeShading(self, fb, world, dg, sample);
    }

    if (haveVolumeHit && volumeFirst) {
      volumeInterval.interval.y = min(geomRay.t, volumeInterval.interval.y);
      setRay(volumeRay, volumeInterval.interval.x, volumeInterval.interval.y);
      volumeColor = integrateVolumeInterval(sample, volumeInterval, volumeRay);
    }

    vec4f blendedColor = make_vec4f(0.f);

    if (bothHit) {
      blendedColor = volumeColor + (1.0f - volumeColor.w) * surfaceColor;
    } else {
      if (haveGeometryHit)
        blendedColor = surfaceColor;
      else
        blendedColor = volumeColor;
    }

    outputColor = outputColor + (1.0f - outputColor.w) * blendedColor;

    if (haveGeometryHit)
      setRay(geomRay, geomRay.t + dg.epsilon, originalRayTFar);
    else
      setRay(geomRay, geomRay.t, geomRay.t0);


    cif(firstHit)
    {
      sample.albedo =
          haveGeometryHit ? surfaceShading.albedo : make_vec3f(bgColor);
      sample.normal = haveGeometryHit ? dg.Ns : negate(geomRay.dir);
    }

    firstHit = false;

    niters++;
  }

  if (niters >= 1000) {
    sample.rgb   = make_vec3f(1.f, 0.f, 0.f);
    sample.alpha = 1.f;
  } else {
    sample.rgb   = make_vec3f(outputColor);
    sample.alpha = outputColor.w;
  }
}

// SciVis C++ interface /////////////////////////////////////////////////////

export void *uniform SciVis_create(void *uniform cppE)
{
  uniform SciVis *uniform self = uniform new uniform SciVis;
  Renderer_Constructor(&self->super, cppE, 1);
  self->super.renderSample = SciVis_renderSample;
  return self;
}

export void SciVis_set(void *uniform _self,
                       uniform int samplesPerFrame,
                       uniform float aoRadius,
                       uniform float aoIntensity)
{
  uniform SciVis *uniform self = (uniform SciVis * uniform) _self;

  self->samplesPerFrame = samplesPerFrame;
  self->aoRadius        = aoRadius;
  self->aoIntensity     = aoIntensity;
}