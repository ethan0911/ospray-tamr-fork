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

// ispc device
#include "math/random.ih"
#include "math/sampling.ih"
// SciVis renderer
#include "SciVisMaterial.ih"
#include "surfaces.ih"

inline vec4f getSurfaceColor(DifferentialGeometry dg,
                             const World *uniform world,
                             const varying Ray &ray)
{
  const SciVisMaterial *mat = (const SciVisMaterial *)dg.material;

  vec3f surfaceColor = make_vec3f(1.f);
  float opacity      = 1.f;
  if (mat) {
    foreach_unique(instID in ray.instID)
    {
      if (instID != RTC_INVALID_GEOMETRY_ID) {
        Instance *uniform instance = *(world->instances + instID);
        dg.P = xfmPoint(instance->xfm, dg.P);
      }
    }

    foreach_unique(m in mat)
    {
      surfaceColor = m->Kd;
      if (m->map_Kd) {
        vec4f Kd_from_map = get4f(m->map_Kd, dg);
        surfaceColor      = surfaceColor * make_vec3f(Kd_from_map);
      }
      opacity = m->d;
    }
  }

  return make_vec4f(surfaceColor * make_vec3f(dg.color), opacity);
}

inline float computeAO(const uniform SciVis *uniform self,
                       const FrameBuffer *uniform fb,
                       const World *uniform world,
                       const varying vec3i &sampleID,
                       const varying DifferentialGeometry &dg)
{
  const uniform int &sampleCnt = self->aoSamples;
  const uniform int accumID    = reduce_max(sampleID.z) * sampleCnt;

  // init TEA RNG //
  RandomTEA rng_state;
  varying RandomTEA *const uniform rng = &rng_state;
  RandomTEA__Constructor(rng, 0x290374, (fb->size.x * sampleID.y) + sampleID.x);
  const vec2f rot = RandomTEA__getFloats(rng);

  int hits = 0;

  const linear3f localToWorld = frame(dg.Ns);

  for (uniform int i = 0; i < sampleCnt; i++) {
    const vec2f halton       = HaltonSequence_get2D(sampleCnt * accumID + i);
    const vec2f r            = CranleyPattersonRotation(halton, rot);
    const vec3f local_ao_dir = cosineSampleHemisphere(r);
    const vec3f ao_dir       = localToWorld * local_ao_dir;

    if (dot(ao_dir, dg.Ns) < 0.05f) {  // check below surface
      hits++;
      continue;
    }

    Ray ao_ray;
    setRay(ao_ray, dg.P, ao_dir, dg.epsilon, self->aoRadius);
    if (isOccluded(world, ao_ray))
      hits++;
  }

  // the cosTheta of cosineSampleHemispherePDF and dot(shadingNormal, ao_dir)
  // cancel
  return 1.0f - (hits / (float)sampleCnt);
}

///////////////////////////////////////////////////////////////////////////////

SSI computeShading(const SciVis *uniform self,
                   const FrameBuffer *uniform fb,
                   const World *uniform world,
                   const DifferentialGeometry &dg,
                   ScreenSample &sample)
{
  SSI retval;

  const vec4f surfaceColor = getSurfaceColor(dg, world, sample.ray);

  const vec3f color   = make_vec3f(surfaceColor);
  const float opacity = surfaceColor.w;

  const float eyeLightIntensity = absf(dot(dg.Ns, sample.ray.dir)) * opacity;

  const uniform bool doAO = self->aoSamples > 0 && self->aoIntensity > 0.01f;

  const float ao = doAO ? computeAO(self, fb, world, sample.sampleID, dg) : 1.f;

  const float aoIntensity = doAO ? self->aoIntensity : 0.f;

  const float intensity =
      ((1 - aoIntensity) + (ao * aoIntensity)) * eyeLightIntensity;

  retval.shadedColor = make_vec4f(color * intensity, opacity);
  retval.albedo      = color;

  return retval;
}
