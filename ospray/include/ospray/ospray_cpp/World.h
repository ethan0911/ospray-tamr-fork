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

#include "Instance.h"
#include "ManagedObject.h"

namespace ospray {
  namespace cpp {

    class World : public ManagedObject_T<OSPWorld>
    {
     public:
      World();
      World(const World &copy);
      World(OSPWorld existing);
    };

    // Inlined function definitions ///////////////////////////////////////////

    inline World::World()
    {
      ospObject = ospNewWorld();
    }

    inline World::World(const World &copy)
        : ManagedObject_T<OSPWorld>(copy.handle())
    {
    }

    inline World::World(OSPWorld existing) : ManagedObject_T<OSPWorld>(existing)
    {
    }

  }  // namespace cpp
}  // namespace ospray
