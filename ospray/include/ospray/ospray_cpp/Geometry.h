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

#include "ManagedObject.h"

namespace ospray {
  namespace cpp    {

    class Geometry : public ManagedObject_T<OSPGeometry>
    {
    public:

      Geometry(const std::string &type);
      Geometry(const Geometry &copy);
      Geometry(OSPGeometry existing);
    };

    // Inlined function definitions ///////////////////////////////////////////

    inline Geometry::Geometry(const std::string &type)
    {
      ospObject = ospNewGeometry(type.c_str());
    }

    inline Geometry::Geometry(const Geometry &copy) :
      ManagedObject_T<OSPGeometry>(copy.handle())
    {
    }

    inline Geometry::Geometry(OSPGeometry existing) :
      ManagedObject_T<OSPGeometry>(existing)
    {
    }

  }// namespace cpp
}// namespace ospray
