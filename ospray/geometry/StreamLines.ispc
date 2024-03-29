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
#include "common/Data.ih"
#include "common/Intersect.ih"
#include "common/Ray.ih"
#include "common/World.ih"
#include "geometry/Geometry.ih"
#include "math/box.ih"
#include "math/sampling.ih"
#include "math/vec.ih"

struct StreamLines
{
  Geometry super;
  Data1D index;
  Data1D vertex;
  Data1D color;
  float radius;
};

static void StreamLines_postIntersect(const Geometry *uniform self,
    varying DifferentialGeometry &dg,
    const varying Ray &ray,
    uniform int64 flags)
{
  StreamLines *uniform self = (StreamLines * uniform) self;

  dg.Ng = dg.Ns = ray.Ng;
  dg.epsilon = self->radius * ulpEpsilon;

  if (and(flags & DG_COLOR, valid(self->color))) {
    const uint32 index = get_int32(self->index, ray.primID);
    const vec4f col0 = get_vec4f(self->color, index);
    const vec4f col1 = get_vec4f(self->color, index + 1);
    dg.color = lerp(ray.u, col0, col1);
  }
}

unmasked void StreamLines_bounds(const RTCBoundsFunctionArguments *uniform args)
{
  StreamLines *uniform self = (StreamLines * uniform) args->geometryUserPtr;
  uniform int primID         = args->primID;
  const uniform uint32 index = get_int32(self->index, primID);
  const uniform vec3f A = get_vec3f(self->vertex, index);
  const uniform vec3f B = get_vec3f(self->vertex, index + 1);
  box3fa *uniform out        = (box3fa * uniform) args->bounds_o;
  *out = make_box3fa(min(A, B) - self->radius, max(A, B) + self->radius);
}

unmasked void StreamLines_intersect_kernel(
    const struct RTCIntersectFunctionNArguments *uniform args,
    const uniform bool occlusionTest)
{
  // make sure to set the mask
  if (!args->valid[programIndex])
    return;

  StreamLines *uniform self = (StreamLines * uniform) args->geometryUserPtr;
  uniform unsigned int primID = args->primID;

  // this assumes that the args->rayhit is actually a pointer toa varying ray!
  varying Ray *uniform ray = (varying Ray * uniform) args->rayhit;

  const uniform uint32 index = get_int32(self->index, primID);
  const uniform vec3f A = get_vec3f(self->vertex, index);
  const uniform vec3f B = get_vec3f(self->vertex, index + 1);

  IntersectionValues isect = intersectCapsule(ray->org, ray->dir, A, B, self->radius);

  bool hit = false;
  if(and(and(isect.t_in.t_value > ray->t0, isect.t_in.t_value < ray->t), isect.t_in.hit))
  {
    hit = true;
    ray->t = isect.t_in.t_value;
    ray->u = isect.t_in.u;
    ray->Ng = isect.t_in.N;
  } else if(and(and(isect.t_out.t_value > ray->t0, isect.t_out.t_value < ray->t), isect.t_out.hit))
  {
    hit = true;
    ray->t = isect.t_out.t_value;
    ray->u = isect.t_out.u;
    ray->Ng = isect.t_out.N;
  }


  if (hit) {
    if (occlusionTest) {
      ray->t = neg_inf;
    } else {
      ray->primID = primID;
      ray->geomID = self->super.geomID;
      ray->instID = args->context->instID[0];
    }
  }
}

unmasked void StreamLines_intersect(
    const struct RTCIntersectFunctionNArguments *uniform args)
{
  StreamLines_intersect_kernel(args, false);
}

unmasked void StreamLines_occluded(
    const struct RTCOccludedFunctionNArguments *uniform args)
{
  StreamLines_intersect_kernel(
      (RTCIntersectFunctionNArguments * uniform) args, true);
}

// For simplicity streamlines are modelled of segments, each consisting of one
// cylinder and two spheres. Sampling would put (too much) effort towards the
// vertices, where often two spheres and two cylinders overlap. Furthermore,
// sampling such a mixed "primitive" is more complex. Thus, choose a different
// representation for area sampling, i.e. one cylinder per segment and one
// sphere per vertex.
SampleAreaRes StreamLines_sampleArea(const Geometry *uniform const _self,
                                     const int32 areaPrimID,
                                     const uniform affine3f &xfm,
                                     const uniform affine3f &rcp_xfm,
                                     const vec2f &s)
{
  const StreamLines *const uniform self = (const StreamLines *uniform)_self;
  const uniform float radius            = self->radius;

  vec3f localNormal;
  vec3f localPos;
  const uniform int32 spheres = self->vertex.numItems;
  if (areaPrimID < spheres) {
    localNormal        = uniformSampleSphere(radius, s);
    const vec3f center = get_vec3f(self->vertex, areaPrimID);
    localPos           = center + localNormal;
  } else { // cylinder
    const uint32 idx = get_uint32(self->index, areaPrimID - spheres);
    const vec3f v0 = get_vec3f(self->vertex, idx);
    const vec3f v1 = get_vec3f(self->vertex, idx + 1);

    linear3f frame  = frame(v1 - v0);
    frame.vy        = normalize(frame.vy);
    const float phi = two_pi * s.x;
    float sinPhi, cosPhi;
    sincos(phi, &sinPhi, &cosPhi);
    localNormal = (cosPhi * radius) * frame.vx + (sinPhi * radius) * frame.vy;
    localPos    = v0 + localNormal + s.y * frame.vz;
  }

  SampleAreaRes res;
  res.pos    = xfmPoint(xfm, localPos);
  res.normal = normalize(xfmVector(transposed(rcp_xfm.l), localNormal));

  return res;
}

void StreamLines_getAreas(const Geometry *const uniform _self,
                          const int32 *const uniform primIDs,
                          const uniform int32 numPrims,
                          const uniform affine3f &xfm,
                          float *const uniform areas)
{
  StreamLines *uniform self = (StreamLines * uniform) _self;

  // detect (non-uniform) scaling; get length of transformed unit-vectors
  const uniform vec3f scaling3 =
      make_vec3f(length(xfm.l.vx), length(xfm.l.vy), length(xfm.l.vz));
  const uniform float min_scaling = reduce_min(scaling3);
  const uniform float max_scaling = reduce_max(scaling3);
  if ((max_scaling - min_scaling) > 1e-4f * min_scaling)
    print(
        "#osp:StreamLines_getAreas: Non-uniform scaling in instance "
        "transformation detected! Importance sampling for emissive "
        "materials and thus resulting image may be wrong.\n");
  // use average as approximation to closest uniform scaling
  const uniform float scaling = reduce_add(scaling3) / 3.f;

  // spheres at vertices
  const uniform float sphereArea = four_pi * sqr(self->radius * scaling);
  const uniform int32 spheres = self->vertex.numItems;

  // cylinders for segments; area of a cylinder with unit-length
  const uniform float cyl1Area = two_pi * self->radius * sqr(scaling);
  // TODO vectorize this loop, with foreach or ProgramCount & ProgramIndex
  for (uniform int32 i = 0; i < numPrims; i++) {
    const uniform int32 primID = primIDs[i];
    if (primID < spheres) {  // sphere
      areas[i] = sphereArea;
    } else {  // cylinder
      const uniform uint32 idx = get_uint32(self->index, primID - spheres);
      const uniform vec3f v0 = get_vec3f(self->vertex, idx);
      const uniform vec3f v1 = get_vec3f(self->vertex, idx + 1);
      areas[i]                 = cyl1Area * length(v0 - v1);
    }
  }
}

void StreamLines_Constructor(
    StreamLines *uniform self, void *uniform cppEquivalent)
{
  Geometry_Constructor(&self->super, cppEquivalent, StreamLines_postIntersect);
  Data1D_Constructor(&self->index);
  Data1D_Constructor(&self->vertex);
  Data1D_Constructor(&self->color);
}

export void *uniform StreamLines_create(void *uniform cppEquivalent)
{
  StreamLines *uniform self = uniform new StreamLines;
  StreamLines_Constructor(self, cppEquivalent);
  return self;
}

export void *uniform StreamLines_setCurve(void *uniform _self,
    const Data1D *uniform index,
    const Data1D *uniform color)
{
  StreamLines *uniform self = (StreamLines * uniform) _self;

  self->index = *index;
  Data1D_Constructor(&self->vertex); // not used by curve
  self->color = *color;
  self->super.numPrimitives = index->numItems;

  // TODO implement area sampling of curves for geometry lights
  self->super.getAreas   = NULL;
  self->super.sampleArea = NULL;
}

export void *uniform StreamLines_set(void *uniform _self,
    void *uniform _embreeGeometry,
    float uniform radius,
    const Data1D *uniform index,
    const Data1D *uniform vertex,
    const Data1D *uniform color)
{
  StreamLines *uniform self = (StreamLines * uniform) _self;

  self->radius = radius;
  self->index = *index;
  self->vertex = *vertex;
  self->color = *color;

  self->super.getAreas      = StreamLines_getAreas;
  self->super.sampleArea    = StreamLines_sampleArea;
  self->super.numPrimitives = index->numItems;

  Geometry_setEmbreeUserGeometry(&self->super,
      (RTCGeometry)_embreeGeometry,
      StreamLines_bounds,
      StreamLines_intersect,
      StreamLines_occluded);

  // XXX different representation for area sampling, see above
  self->super.numPrimitives += vertex->numItems;
}
