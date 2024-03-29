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

#pragma once

#include <ospray/ospray.h>

#ifdef _WIN32
#ifdef ospray_testing_EXPORTS
#define OSPRAY_TESTING_INTERFACE __declspec(dllexport)
#else
#define OSPRAY_TESTING_INTERFACE __declspec(dllimport)
#endif
#define OSPRAY_TESTING_DLLEXPORT __declspec(dllexport)
#else
#define OSPRAY_TESTING_INTERFACE
#define OSPRAY_TESTING_DLLEXPORT
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct { float x, y; }             osp_vec2f;
typedef struct { float x, y, z; }          osp_vec3f;
typedef struct { osp_vec3f lower, upper; } osp_box3f;

typedef struct
{
  OSPGeometry geometry;
  OSPGeometricModel model;
  OSPGroup group;
  OSPInstance instance;
  osp_box3f bounds;
} OSPTestingGeometry;

typedef struct
{
  OSPVolume volume;
  osp_box3f bounds;
  osp_vec2f voxelRange;
} OSPTestingVolume;

/* Create an OSPRay renderer with sensible defaults for testing */
OSPRAY_TESTING_INTERFACE
OSPRenderer ospTestingNewRenderer(const char *type OSP_DEFAULT_VAL("scivis"));

/* Create an OSPRay geometry (from a registered name), with the given renderer
 * type to create materials */
OSPRAY_TESTING_INTERFACE
OSPTestingGeometry ospTestingNewGeometry(const char *geom_type,
                                         const char *renderer_type
                                             OSP_DEFAULT_VAL("scivis"));

/* Create an OSPRay geometry (from a registered name) */
OSPRAY_TESTING_INTERFACE
OSPTestingVolume ospTestingNewVolume(const char *volume_type);

/* Create an OSPRay geometry (from a registered name) */
OSPRAY_TESTING_INTERFACE
OSPTransferFunction ospTestingNewTransferFunction(
    osp_vec2f voxelRange,
    const char *tf_name OSP_DEFAULT_VAL("grayscale"));

/* Create an OSPRay perspective camera which looks at the center of the given
 * bounding box
 *
 * NOTE: this only sets 'dir', 'pos', and 'up'
 */
OSPRAY_TESTING_INTERFACE
OSPCamera ospTestingNewDefaultCamera(osp_box3f bounds);

/* Create a list of lights, using a given preset name */
OSPRAY_TESTING_INTERFACE
OSPData ospTestingNewLights(
    const char *lighting_set_name OSP_DEFAULT_VAL("ambient_only"));

#ifdef __cplusplus
}  // extern "C"
#endif
