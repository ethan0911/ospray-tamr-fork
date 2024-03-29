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

#include "common/Managed.h"

namespace ospray {

  /*! \brief A TransferFunction is an abstraction that maps a value to
    a color and opacity for rendering.

    The actual mapping is unknown to this class, and is implemented
    in subclasses.  A type string specifies a particular concrete
    implementation to createInstance().  This type string must be
    registered in OSPRay proper, or in a loaded module using
    OSP_REGISTER_TRANSFER_FUNCTION.
  */
  struct OSPRAY_SDK_INTERFACE TransferFunction : public ManagedObject
  {
    TransferFunction() = default;
    virtual ~TransferFunction() override = default;
    virtual void commit() override;
    virtual std::string toString() const override;

    //! Create a transfer function of the given type.
    static TransferFunction *createInstance(const std::string &type);
  };

  OSPTYPEFOR_SPECIALIZATION(TransferFunction *, OSP_TRANSFER_FUNCTION);

/*! \brief Define a function to create an instance of the InternalClass
  associated with external_name.

 \internal The function generated by this macro is used to create an
  instance of a concrete subtype of an abstract base class.  This
  macro is needed since the subclass type may not be known to OSPRay
  at build time.  Rather, the subclass can be defined in an external
  module and registered with OSPRay using this macro.
*/
#define OSP_REGISTER_TRANSFER_FUNCTION(InternalClass, external_name) \
  OSP_REGISTER_OBJECT(::ospray::TransferFunction, transfer_function, \
                      InternalClass, external_name)

} // ::ospray
