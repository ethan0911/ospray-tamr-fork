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

#include "volumes.ih"
// ispc device
#include "math/random.ih"
#include "math/sampling.ih"
#include "render/util.ih"

vec4f integrateVolumeInterval(const ScreenSample &sample,
                              const VolumeInterval &interval,
                              Ray &ray)
{
  VolumetricModel *varying model = interval.volumetricModel;

  vec3f color = make_vec3f(0.f);
  float alpha = 0.f;

  float pSample = -1.f;

  // Ray offset for this sample, as a fraction of the nominal step size.
  float jitter = precomputedHalton2(sample.sampleID.z);
  int ix       = sample.sampleID.x % 4;
  int iy       = sample.sampleID.y % 4;

  int patternID = ix + 4 * iy;
  jitter += precomputedHalton3(patternID);

  if (jitter > 1.f)
    jitter -= 1.f;

  //ray.t0 += jitter;

  foreach_unique(m in model)
  {
    Volume *uniform volume       = m->volume;
    TransferFunction *uniform tf = m->transferFunction;

    const uniform float baseStepSize = 0.1;//m->samplingStep / m->samplingRate;
    float lastSamplePos = ray.t0;

    while (ray.t0 <= ray.t && alpha < 0.99f) {
      const float sample = volume->sample(volume, ray.org + ray.t0 * ray.dir);

      if (!isnan(sample)) {
        vec3f sampleColor = tf->getIntegratedColorForValue(tf, pSample, sample);
        float sampleAlpha =
            tf->getIntegratedOpacityForValue(tf, pSample, sample);

        if (lastSamplePos != ray.t0) {
          // TODO: I'll send back the cell width in the "time" param, which will
          // be used as our sample distance. Then the TAMR volume will give us
          // back something like the "min cell width" which would be our base sampling rate.
          //sampleAlpha = 1.f - powf(1.f - sampleAlpha, (ray.t0 - lastSamplePos) / ray.time);
        }

        pSample = sample;

        const float clampedOpacity = clamp(sampleAlpha);// / m->samplingRate);

        sampleColor = sampleColor * clampedOpacity;
        sampleAlpha = clampedOpacity;

        color = color + ((1.f - alpha) * sampleColor);
        alpha = alpha + ((1.f - alpha) * sampleAlpha);
      }

      lastSamplePos = ray.t0;
      volume->stepRay(volume, tf, ray, m->samplingRate);
    }
  }

  return make_vec4f(color, alpha);
}
