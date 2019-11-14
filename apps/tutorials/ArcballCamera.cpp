// ======================================================================== //
// Copyright 2017-2019 Intel Corporation                                    //
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

#include "ArcballCamera.h"

ArcballCamera::ArcballCamera(const ospcommon::math::box3f &worldBounds,
                             const ospcommon::math::vec2i &windowSize)
    : worldDiag(1),
      invWindowSize(ospcommon::math::vec2f(1.0) /
                    ospcommon::math::vec2f(windowSize)),
      centerTranslation(ospcommon::math::one),
      translation(ospcommon::math::one),
      rotation(ospcommon::math::one)
{
  ospcommon::math::vec3f diag = worldBounds.size();
  worldDiag = ospcommon::math::max(length(diag), 1.f);
  diag =
      ospcommon::math::max(diag, ospcommon::math::vec3f(0.3f * length(diag)));

  centerTranslation =
      ospcommon::math::AffineSpace3f::translate(-worldBounds.center());
  translation = ospcommon::math::AffineSpace3f::translate(
      ospcommon::math::vec3f(0, 0, length(diag)));
  updateCamera();
}

void ArcballCamera::rotate(const ospcommon::math::vec2f &from,
                           const ospcommon::math::vec2f &to)
{
  rotation = screenToArcball(to) * screenToArcball(from) * rotation;
  updateCamera();
}

void ArcballCamera::zoom(float amount)
{
  amount *= (std::max(translation.p.z / worldDiag, 0.001f));
  translation = ospcommon::math::AffineSpace3f::translate(
                    ospcommon::math::vec3f(0, 0, amount)) *
                translation;
  translation.p.z = std::max(translation.p.z, 0.0001f);
  updateCamera();
}

void ArcballCamera::pan(const ospcommon::math::vec2f &delta)
{
  const ospcommon::math::vec3f t = ospcommon::math::vec3f(
      -delta.x * invWindowSize.x, delta.y * invWindowSize.y, 0);
  const ospcommon::math::vec3f worldt =
      translation.p.z * xfmVector(invCamera, t);
  centerTranslation =
      ospcommon::math::AffineSpace3f::translate(worldt) * centerTranslation;
  updateCamera();
}

ospcommon::math::vec3f ArcballCamera::eyePos() const
{
  return xfmPoint(invCamera, ospcommon::math::vec3f(0, 0, 1));
}

ospcommon::math::vec3f ArcballCamera::center() const
{
  return -centerTranslation.p;
}

ospcommon::math::vec3f ArcballCamera::lookDir() const
{
  return xfmVector(invCamera, ospcommon::math::vec3f(0, 0, 1));
}

ospcommon::math::vec3f ArcballCamera::upDir() const
{
  return xfmVector(invCamera, ospcommon::math::vec3f(0, 1, 0));
}

void ArcballCamera::updateCamera()
{
  const ospcommon::math::AffineSpace3f rot =
      ospcommon::math::LinearSpace3f(rotation);
  const ospcommon::math::AffineSpace3f camera =
      translation * rot * centerTranslation;
  invCamera = rcp(camera);
}

void ArcballCamera::setCenter(const ospcommon::math::vec3f &center)
{
  ospcommon::math::vec3f eye = eyePos();
  centerTranslation = ospcommon::math::AffineSpace3f::translate(-center);
  translation = ospcommon::math::AffineSpace3f::translate(
      ospcommon::math::vec3f(0, 0, length(center - eye)));
  updateCamera();
}

void ArcballCamera::setRotation(ospcommon::math::quaternionf q)
{
  rotation = q;
  updateCamera();
}

void ArcballCamera::updateWindowSize(const ospcommon::math::vec2i &windowSize)
{
  invWindowSize =
      ospcommon::math::vec2f(1) / ospcommon::math::vec2f(windowSize);
}

ospcommon::math::quaternionf ArcballCamera::screenToArcball(
    const ospcommon::math::vec2f &p)
{
  const float dist = dot(p, p);
  // If we're on/in the sphere return the point on it
  if (dist <= 1.f) {
    return ospcommon::math::quaternionf(0, p.x, p.y, std::sqrt(1.f - dist));
  } else {
    // otherwise we project the point onto the sphere
    const ospcommon::math::vec2f unitDir = normalize(p);
    return ospcommon::math::quaternionf(0, unitDir.x, unitDir.y, 0);
  }
}
