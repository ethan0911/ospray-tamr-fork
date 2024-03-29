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

#include "api/ISPCDevice.h"
#include "common/Data.h"
#include "common/Managed.h"
#include "common/Material.h"
#include "common/OSPCommon.h"
// embree
#include "embree3/rtcore.h"

namespace ospray {

  struct OSPRAY_SDK_INTERFACE LiveGeometry
  {
    void *ispcEquivalent{nullptr};
    RTCGeometry embreeGeometry{nullptr};
  };

  struct OSPRAY_SDK_INTERFACE Geometry : public ManagedObject
  {
    Geometry();
    virtual ~Geometry() override = default;

    virtual std::string toString() const override;

    virtual size_t numPrimitives() const = 0;

    virtual LiveGeometry createEmbreeGeometry() = 0;

    void postCreationInfo(size_t numVerts = 0) const;

    // Object factory //

    static Geometry *createInstance(const char *type);
  };

  OSPTYPEFOR_SPECIALIZATION(Geometry *, OSP_GEOMETRY);

  // convenience wrappers to set Embree buffer
  // /////////////////////////////////////////

  template <typename T>
  struct RTCFormatFor
  {
    static constexpr RTCFormat value = RTC_FORMAT_UNDEFINED;
  };

#define RTCFORMATFOR_SPECIALIZATION(type, rtc_format)                          \
  template <>                                                                  \
  struct RTCFormatFor<type>                                                    \
  {                                                                            \
    static constexpr RTCFormat value = rtc_format;                             \
  };

  RTCFORMATFOR_SPECIALIZATION(char, RTC_FORMAT_CHAR);
  RTCFORMATFOR_SPECIALIZATION(unsigned char, RTC_FORMAT_UCHAR);
  RTCFORMATFOR_SPECIALIZATION(short, RTC_FORMAT_SHORT);
  RTCFORMATFOR_SPECIALIZATION(unsigned short, RTC_FORMAT_USHORT);
  RTCFORMATFOR_SPECIALIZATION(int32_t, RTC_FORMAT_INT);
  RTCFORMATFOR_SPECIALIZATION(vec2i, RTC_FORMAT_INT2);
  RTCFORMATFOR_SPECIALIZATION(vec3i, RTC_FORMAT_INT3);
  RTCFORMATFOR_SPECIALIZATION(vec4i, RTC_FORMAT_INT4);
  RTCFORMATFOR_SPECIALIZATION(uint32_t, RTC_FORMAT_UINT);
  RTCFORMATFOR_SPECIALIZATION(vec2ui, RTC_FORMAT_UINT2);
  RTCFORMATFOR_SPECIALIZATION(vec3ui, RTC_FORMAT_UINT3);
  RTCFORMATFOR_SPECIALIZATION(vec4ui, RTC_FORMAT_UINT4);
  RTCFORMATFOR_SPECIALIZATION(int64_t, RTC_FORMAT_LLONG);
  RTCFORMATFOR_SPECIALIZATION(vec2l, RTC_FORMAT_LLONG2);
  RTCFORMATFOR_SPECIALIZATION(vec3l, RTC_FORMAT_LLONG3);
  RTCFORMATFOR_SPECIALIZATION(vec4l, RTC_FORMAT_LLONG4);
  RTCFORMATFOR_SPECIALIZATION(uint64_t, RTC_FORMAT_ULLONG);
  RTCFORMATFOR_SPECIALIZATION(vec2ul, RTC_FORMAT_ULLONG2);
  RTCFORMATFOR_SPECIALIZATION(vec3ul, RTC_FORMAT_ULLONG3);
  RTCFORMATFOR_SPECIALIZATION(vec4ul, RTC_FORMAT_ULLONG4);
  RTCFORMATFOR_SPECIALIZATION(float, RTC_FORMAT_FLOAT);
  RTCFORMATFOR_SPECIALIZATION(vec2f, RTC_FORMAT_FLOAT2);
  RTCFORMATFOR_SPECIALIZATION(vec3f, RTC_FORMAT_FLOAT3);
  RTCFORMATFOR_SPECIALIZATION(vec4f, RTC_FORMAT_FLOAT4);
  RTCFORMATFOR_SPECIALIZATION(linear2f, RTC_FORMAT_FLOAT2X2_COLUMN_MAJOR);
  RTCFORMATFOR_SPECIALIZATION(linear3f, RTC_FORMAT_FLOAT3X3_COLUMN_MAJOR);
  RTCFORMATFOR_SPECIALIZATION(affine2f, RTC_FORMAT_FLOAT2X3_COLUMN_MAJOR);
  RTCFORMATFOR_SPECIALIZATION(affine3f, RTC_FORMAT_FLOAT3X4_COLUMN_MAJOR);
#undef RTCFORMATFOR_SPECIALIZATION

  // ... via ospData, NoOp when data is invalid
  template <typename T>
  void setEmbreeGeometryBuffer(RTCGeometry geom,
      enum RTCBufferType type,
      Ref<const DataT<T, 1>> &dataRef,
      unsigned int slot = 0)
  {
    if (!dataRef)
      return;
    rtcSetSharedGeometryBuffer(geom,
        type,
        slot,
        RTCFormatFor<T>::value,
        dataRef->data(),
        0,
        dataRef->stride(),
        dataRef->size());
  }
  // ... via an std::vector
  template <typename T>
  void setEmbreeGeometryBuffer(RTCGeometry geom,
      enum RTCBufferType type,
      std::vector<T> &data,
      unsigned int slot = 0)
  {
    rtcSetSharedGeometryBuffer(geom,
        type,
        slot,
        RTCFormatFor<T>::value,
        data.data(),
        0,
        sizeof(T),
        data.size());
  }

#define OSP_REGISTER_GEOMETRY(InternalClass, external_name) \
  OSP_REGISTER_OBJECT(                                      \
      ::ospray::Geometry, geometry, InternalClass, external_name)

}  // namespace ospray
