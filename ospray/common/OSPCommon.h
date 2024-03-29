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

#include "OSPConfig.h"

#ifdef _WIN32
// ----------- windows only -----------
typedef unsigned long long id_t;
# ifndef _USE_MATH_DEFINES
#   define _USE_MATH_DEFINES
# endif
# include <cmath>
# include <math.h>
# ifdef _M_X64
typedef long long ssize_t;
# else
typedef int ssize_t;
# endif
#else
// ----------- NOT windows -----------
# include "unistd.h"
#endif

// ospcommon
#include "ospcommon/math/AffineSpace.h"
#include "ospcommon/memory/RefCount.h"
#include "ospcommon/memory/malloc.h"

// ospray
#include "ospray/ospray.h"

// std
#include <cstdint> // for int64_t etc
#include <sstream>

// NOTE(jda) - This one file is shared between the core OSPRay library and the
//             ispc module...thus we have to define 2 different EXPORTS macros
//             here. This needs to be split between the libraries!
#ifdef _WIN32
#  ifdef ospray_EXPORTS
#    define OSPRAY_INTERFACE __declspec(dllexport)
#  else
#    define OSPRAY_INTERFACE __declspec(dllimport)
#  endif
#  define OSPRAY_DLLEXPORT __declspec(dllexport)
#else
#  define OSPRAY_INTERFACE
#  define OSPRAY_DLLEXPORT
#endif
#define OSPRAY_CORE_INTERFACE OSPRAY_INTERFACE

#ifdef _WIN32
#  ifdef ospray_module_ispc_EXPORTS
#    define OSPRAY_MODULE_ISPC_INTERFACE __declspec(dllexport)
#  else
#    define OSPRAY_MODULE_ISPC_INTERFACE __declspec(dllimport)
#  endif
#  define OSPRAY_MODULE_ISPC_DLLEXPORT __declspec(dllexport)
#else
#  define OSPRAY_MODULE_ISPC_INTERFACE
#  define OSPRAY_MODULE_ISPC_DLLEXPORT
#endif
#define OSPRAY_SDK_INTERFACE OSPRAY_MODULE_ISPC_INTERFACE

#define OSP_REGISTER_OBJECT(Object, object_name, InternalClass, external_name) \
  extern "C" OSPRAY_DLLEXPORT                                                  \
      Object *ospray_create_##object_name##__##external_name()                 \
  {                                                                            \
    return new InternalClass;                                                  \
  }                                                                            \
  /* additional declaration to avoid "extra ;" -Wpedantic warnings */          \
  Object *ospray_create_##object_name##__##external_name()

//! main namespace for all things ospray (for internal code)
namespace ospray {

  using namespace ospcommon;
  using namespace ospcommon::math;
  using namespace ospcommon::memory;

  /*! basic types */
  using int64  = std::int64_t;
  using uint64 = std::uint64_t;

  using int32  = std::int32_t;
  using uint32 = std::uint32_t;

  using int16  = std::int16_t;
  using uint16 = std::uint16_t;

  using int8  = std::int8_t;
  using uint8 = std::uint8_t;

  using index_t = std::int64_t;

  void initFromCommandLine(int *ac = nullptr, const char ***av = nullptr);

  extern "C" {
    /*! 64-bit malloc. allows for alloc'ing memory larger than 4GB */
    OSPRAY_CORE_INTERFACE void *malloc64(size_t size);
    /*! 64-bit malloc. allows for alloc'ing memory larger than 4GB */
    OSPRAY_CORE_INTERFACE void free64(void *ptr);
  }

  OSPRAY_CORE_INTERFACE OSPDataType typeOf(const char *string);
  inline OSPDataType typeOf(const std::string &s)
  {
    return typeOf(s.c_str());
  }

  OSPRAY_CORE_INTERFACE std::string stringFor(OSPDataType);
  OSPRAY_CORE_INTERFACE std::string stringFor(OSPTextureFormat);

  inline bool isObjectType(OSPDataType type)
  {
    return type & OSP_OBJECT;
  }

  OSPRAY_CORE_INTERFACE size_t sizeOf(OSPDataType);
  OSPRAY_CORE_INTERFACE size_t sizeOf(OSPTextureFormat);

  OSPRAY_CORE_INTERFACE OSPError loadLocalModule(const std::string &name);

  /*! little helper class that prints out a warning string upon the
    first time it is encountered.

    Usage:

    if (someThisBadHappens) {
       static WarnOnce warning("something bad happened, at least once!");
       ...
    }
  */
  struct OSPRAY_CORE_INTERFACE WarnOnce
  {
    WarnOnce(const std::string &message, uint32_t postAtLogLevel = 0);
  private:
    const std::string s;
  };

  OSPRAY_CORE_INTERFACE uint32_t logLevel();

  OSPRAY_CORE_INTERFACE void postStatusMsg(const std::stringstream &msg,
                                          uint32_t postAtLogLevel = 0);

  OSPRAY_CORE_INTERFACE void postStatusMsg(const std::string &msg,
                                          uint32_t postAtLogLevel = 0);

  // use postStatusMsg to output any information, which will use OSPRay's
  // internal infrastructure, optionally at a given loglevel
  // a newline is added implicitly
  //////////////////////////////////////////////////////////////////////////////

  struct StatusMsgStream : public std::stringstream
  {
    StatusMsgStream(uint32_t postAtLogLevel = 0);
    StatusMsgStream(StatusMsgStream &&other);
    ~StatusMsgStream();

  private:

    uint32_t logLevel {0};
  };

  inline StatusMsgStream::StatusMsgStream(uint32_t postAtLogLevel)
    : logLevel(postAtLogLevel)
  {
  }

  inline StatusMsgStream::~StatusMsgStream()
  {
    auto msg = str();
    if (!msg.empty())
      postStatusMsg(msg, logLevel);
  }

  inline StatusMsgStream::StatusMsgStream(StatusMsgStream &&other)
  {
    this->str(other.str());
  }

  OSPRAY_CORE_INTERFACE StatusMsgStream postStatusMsg(uint32_t postAtLogLevel = 0);

  /////////////////////////////////////////////////////////////////////////////

  OSPRAY_CORE_INTERFACE void handleError(OSPError e, const std::string &message);

  OSPRAY_CORE_INTERFACE void postTraceMsg(const std::string &message);

  //! log status message at loglevel x
  #define ospLog(x) StatusMsgStream(x) << "(" << x << "): "
  //! log status message at loglevel x with function name
  #define ospLogF(x) StatusMsgStream(x) << __FUNCTION__ << ": "
  //! log status message at loglevel x with function, file, and line number
  #define ospLogL(x) StatusMsgStream(x) << __FUNCTION__ << "(" << __FILE__ << ":" << __LINE__ << "): "

  // RTTI hash ID lookup helper functions ///////////////////////////////////

  OSPRAY_CORE_INTERFACE size_t translatedHash(size_t v);

  template <typename T>
  inline size_t typeIdOf()
  {
    return translatedHash(typeid(T).hash_code());
  }

  template <typename T>
  inline size_t typeIdOf(T *v)
  {
    return translatedHash(typeid(*v).hash_code());
  }

  template <typename T>
  inline size_t typeIdOf(const T &v)
  {
    return translatedHash(typeid(v).hash_code());
  }

  template <typename T>
  inline size_t typeIdOf(const std::unique_ptr<T> &v)
  {
    return translatedHash(typeid(*v).hash_code());
  }

  template <typename T>
  inline size_t typeIdOf(const std::shared_ptr<T> &v)
  {
    return translatedHash(typeid(*v).hash_code());
  }

  // RTTI string name lookup helper functions ///////////////////////////////

  template <typename T>
  inline std::string typeString()
  {
    return typeid(T).name();
  }

  template <typename T>
  inline std::string typeString(T *v)
  {
    return typeid(*v).name();
  }

  template <typename T>
  inline std::string typeString(const T &v)
  {
    return typeid(v).name();
  }

  template <typename T>
  inline std::string typeString(const std::unique_ptr<T> &v)
  {
    return typeid(*v).name();
  }

  template <typename T>
  inline std::string typeString(const std::shared_ptr<T> &v)
  {
    return typeid(*v).name();
  }

  // Infer (compile time) OSP_DATA_TYPE from input type ///////////////////////

  template <typename T>
  struct OSPTypeFor
  {
    static constexpr OSPDataType value = OSP_UNKNOWN;
  };

#define OSPTYPEFOR_SPECIALIZATION(type, osp_type)                              \
  template <>                                                                  \
  struct OSPTypeFor<type>                                                      \
  {                                                                            \
    static constexpr OSPDataType value = osp_type;                             \
  };

  OSPTYPEFOR_SPECIALIZATION(const char *, OSP_STRING);
  OSPTYPEFOR_SPECIALIZATION(const char [], OSP_STRING);
  OSPTYPEFOR_SPECIALIZATION(char, OSP_CHAR);
  OSPTYPEFOR_SPECIALIZATION(unsigned char, OSP_UCHAR);
  OSPTYPEFOR_SPECIALIZATION(short, OSP_SHORT);
  OSPTYPEFOR_SPECIALIZATION(unsigned short, OSP_USHORT);
  OSPTYPEFOR_SPECIALIZATION(int32_t, OSP_INT);
  OSPTYPEFOR_SPECIALIZATION(vec2i, OSP_VEC2I);
  OSPTYPEFOR_SPECIALIZATION(vec3i, OSP_VEC3I);
  OSPTYPEFOR_SPECIALIZATION(vec4i, OSP_VEC4I);
  OSPTYPEFOR_SPECIALIZATION(uint32_t, OSP_UINT);
  OSPTYPEFOR_SPECIALIZATION(vec2ui, OSP_VEC2UI);
  OSPTYPEFOR_SPECIALIZATION(vec3ui, OSP_VEC3UI);
  OSPTYPEFOR_SPECIALIZATION(vec4ui, OSP_VEC4UI);
  OSPTYPEFOR_SPECIALIZATION(int64_t, OSP_LONG);
  OSPTYPEFOR_SPECIALIZATION(vec2l, OSP_VEC2L);
  OSPTYPEFOR_SPECIALIZATION(vec3l, OSP_VEC3L);
  OSPTYPEFOR_SPECIALIZATION(vec4l, OSP_VEC4L);
  OSPTYPEFOR_SPECIALIZATION(uint64_t, OSP_ULONG);
  OSPTYPEFOR_SPECIALIZATION(vec2ul, OSP_VEC2UL);
  OSPTYPEFOR_SPECIALIZATION(vec3ul, OSP_VEC3UL);
  OSPTYPEFOR_SPECIALIZATION(vec4ul, OSP_VEC4UL);
  OSPTYPEFOR_SPECIALIZATION(float, OSP_FLOAT);
  OSPTYPEFOR_SPECIALIZATION(vec2f, OSP_VEC2F);
  OSPTYPEFOR_SPECIALIZATION(vec3f, OSP_VEC3F);
  OSPTYPEFOR_SPECIALIZATION(vec4f, OSP_VEC4F);
  OSPTYPEFOR_SPECIALIZATION(double, OSP_DOUBLE);
  OSPTYPEFOR_SPECIALIZATION(box1i, OSP_BOX1I);
  OSPTYPEFOR_SPECIALIZATION(box2i, OSP_BOX2I);
  OSPTYPEFOR_SPECIALIZATION(box3i, OSP_BOX3I);
  OSPTYPEFOR_SPECIALIZATION(box4i, OSP_BOX4I);
  OSPTYPEFOR_SPECIALIZATION(box1f, OSP_BOX1F);
  OSPTYPEFOR_SPECIALIZATION(box2f, OSP_BOX2F);
  OSPTYPEFOR_SPECIALIZATION(box3f, OSP_BOX3F);
  OSPTYPEFOR_SPECIALIZATION(box4f, OSP_BOX4F);
  OSPTYPEFOR_SPECIALIZATION(linear2f, OSP_LINEAR2F);
  OSPTYPEFOR_SPECIALIZATION(linear3f, OSP_LINEAR3F);
  OSPTYPEFOR_SPECIALIZATION(affine2f, OSP_AFFINE2F);
  OSPTYPEFOR_SPECIALIZATION(affine3f, OSP_AFFINE3F);

  OSPTYPEFOR_SPECIALIZATION(OSPObject, OSP_OBJECT);
  OSPTYPEFOR_SPECIALIZATION(OSPCamera, OSP_CAMERA);
  OSPTYPEFOR_SPECIALIZATION(OSPData, OSP_DATA);
  OSPTYPEFOR_SPECIALIZATION(OSPFrameBuffer, OSP_FRAMEBUFFER);
  OSPTYPEFOR_SPECIALIZATION(OSPFuture, OSP_FUTURE);
  OSPTYPEFOR_SPECIALIZATION(OSPGeometricModel, OSP_GEOMETRIC_MODEL);
  OSPTYPEFOR_SPECIALIZATION(OSPGeometry, OSP_GEOMETRY);
  OSPTYPEFOR_SPECIALIZATION(OSPGroup, OSP_GROUP);
  OSPTYPEFOR_SPECIALIZATION(OSPImageOp, OSP_IMAGE_OP);
  OSPTYPEFOR_SPECIALIZATION(OSPInstance, OSP_INSTANCE);
  OSPTYPEFOR_SPECIALIZATION(OSPLight, OSP_LIGHT);
  OSPTYPEFOR_SPECIALIZATION(OSPMaterial, OSP_MATERIAL);
  OSPTYPEFOR_SPECIALIZATION(OSPRenderer, OSP_RENDERER);
  OSPTYPEFOR_SPECIALIZATION(OSPTexture, OSP_TEXTURE);
  OSPTYPEFOR_SPECIALIZATION(OSPTransferFunction, OSP_TRANSFER_FUNCTION);
  OSPTYPEFOR_SPECIALIZATION(OSPVolume, OSP_VOLUME);
  OSPTYPEFOR_SPECIALIZATION(OSPVolumetricModel, OSP_VOLUMETRIC_MODEL);
  OSPTYPEFOR_SPECIALIZATION(OSPWorld, OSP_WORLD);

} // ::ospray
