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
                              const World *uniform world,
                              Ray &ray)
{
  VolumetricModel *varying model = interval.volumetricModel;

  vec3f color = make_vec3f(0.f);
  float alpha = 0.f;

  float pSample = -1.f;

  foreach_unique(m in model)
  {
    Volume *uniform volume       = m->volume;
    TransferFunction *uniform tf = m->transferFunction;

    if (volume->integrateVolumeInterval) {

      Ray vray = ray;
      vray.org = xfmPoint(interval.xfm, ray.org);
      vray.dir = xfmVector(interval.xfm.l, ray.dir);

      vec4f result =
          volume->integrateVolumeInterval(volume, tf, vray, interval.interval, sample);
      color = make_vec3f(result);
      alpha = result.w;

      // color = color + ((1.f - alpha) * sampleColor);
      // alpha = alpha + ((1.f - alpha) * sampleAlpha);
    } else {
      while (ray.t0 <= ray.t && alpha < 0.99f) {
        const float sample = volume->sample(volume, ray.org + ray.t0 * ray.dir);

        if (!isnan(sample)) {
          vec3f sampleColor =
              tf->getIntegratedColorForValue(tf, pSample, sample);
          float sampleAlpha =
              tf->getIntegratedOpacityForValue(tf, pSample, sample);

          pSample = sample;

          const float clampedOpacity = clamp(sampleAlpha);// / m->samplingRate);

          sampleColor = sampleColor * clampedOpacity;
          sampleAlpha = clampedOpacity;

          color = color + ((1.f - alpha) * sampleColor);
          alpha = alpha + ((1.f - alpha) * sampleAlpha);
        }
        volume->stepRay(volume, tf, ray, m->samplingRate);
      }
    }
  }

  return make_vec4f(color, alpha);
}
