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

// stl
#include <map>
// ospcommon
#include "ospcommon/os/library.h"

namespace ospray {
  namespace testing {

    template <typename T>
    inline T *objectFactory(const std::string &categoryName,
                            const std::string &type)
    {
      using namespace ospcommon;
      // Function pointer type for creating a concrete instance of a subtype of
      // this class.
      using creationFunctionPointer = T *(*)();

      // Function pointers corresponding to each subtype.
      static std::map<std::string, creationFunctionPointer> symbolRegistry;

      // Find the creation function for the subtype if not already known.
      if (symbolRegistry.count(type) == 0) {
        // Construct the name of the creation function to look for.
        std::string creationFunctionName =
            "ospray_create_" + categoryName + "__" + type;

        loadLibrary("ospray_testing", false);

        // Look for the named function.
        symbolRegistry[type] =
            (creationFunctionPointer)getSymbol(creationFunctionName);
      }

      // Create a concrete instance of the requested subtype.
      auto *object = symbolRegistry[type] ? (*symbolRegistry[type])() : nullptr;

      if (object == nullptr) {
        symbolRegistry.erase(type);
        throw std::runtime_error(
            "Could not find " + categoryName + " of type: " + type +
            ".  Make sure you have the correct OSPRay libraries linked.");
      }

      return object;
    }

  }  // namespace testing
}  // namespace ospray
