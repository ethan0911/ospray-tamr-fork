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

#include "Geometry.h"

namespace ospray {

  struct OSPRAY_SDK_INTERFACE StreamLines : public Geometry
  {
    StreamLines() = default;
    virtual ~StreamLines() override = default;

    virtual std::string toString() const override;

    virtual void commit() override;

    virtual size_t numPrimitives() const override;

    LiveGeometry createEmbreeGeometry() override;

   protected:
    Ref<const DataT<uint32_t>> indexData;
    Ref<const DataT<vec3f>> vertexData;
    Ref<const DataT<vec4f>> colorData;
    Ref<const DataT<float>> radiusData;

    float radius{0.01}; // default radius, if no per-vertex radius
    bool useCurve{false};

    std::vector<vec4f> vertexCurve;
    std::vector<uint32_t> indexCurve;
  };

}  // namespace ospray
