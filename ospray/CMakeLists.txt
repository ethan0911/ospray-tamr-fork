## ======================================================================== ##
## Copyright 2009-2019 Intel Corporation                                    ##
##                                                                          ##
## Licensed under the Apache License, Version 2.0 (the "License");          ##
## you may not use this file except in compliance with the License.         ##
## You may obtain a copy of the License at                                  ##
##                                                                          ##
##     http://www.apache.org/licenses/LICENSE-2.0                           ##
##                                                                          ##
## Unless required by applicable law or agreed to in writing, software      ##
## distributed under the License is distributed on an "AS IS" BASIS,        ##
## WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. ##
## See the License for the specific language governing permissions and      ##
## limitations under the License.                                           ##
## ======================================================================== ##

# -------------------------------------------------------
# Setup ospray include directories and source files
# -------------------------------------------------------

set(OSPRAY_SOURCES
  api/API.cpp
  api/Device.cpp

  common/ospray.rc
  common/OSPCommon.cpp

  include/ospray/ospray.h
  include/ospray/OSPEnums.h

  include/ospray/ospray_cpp.h
  include/ospray/ospray_cpp/Camera.h
  include/ospray/ospray_cpp/Data.h
  include/ospray/ospray_cpp/FrameBuffer.h
  include/ospray/ospray_cpp/Geometry.h
  include/ospray/ospray_cpp/Light.h
  include/ospray/ospray_cpp/ManagedObject.h
  include/ospray/ospray_cpp/Material.h
  include/ospray/ospray_cpp/ImageOp.h
  include/ospray/ospray_cpp/Renderer.h
  include/ospray/ospray_cpp/Texture.h
  include/ospray/ospray_cpp/TransferFunction.h
  include/ospray/ospray_cpp/Volume.h
  include/ospray/ospray_cpp/World.h
)


set(OSPRAY_ISPC_DEVICE_SOURCES
  ispc_tasksys.cpp

  api/ISPCDevice.h
  api/ISPCDevice.cpp

  common/ObjectHandle.cpp
  common/Data.ih
  common/Data.cpp
  common/Group.cpp
  common/Group.ispc
  common/Instance.cpp
  common/Instance.ispc
  common/Managed.cpp
  common/OSPCommon.ispc
  common/Material.cpp
  common/Util.h
  common/World.ispc
  common/World.cpp

  fb/FrameBuffer.ispc
  fb/FrameBuffer.cpp
  fb/FrameBufferView.cpp
  fb/LocalFB.ispc
  fb/LocalFB.cpp
  fb/ImageOp.cpp
  fb/Tile.h
  fb/TileError.cpp

  fb/frame_ops/Blur.cpp
  fb/frame_ops/Debug.cpp
  fb/frame_ops/Depth.cpp
  fb/frame_ops/SSAO.cpp
  fb/frame_ops/SSAO.ispc

  fb/tile_ops/SaveTiles.cpp
  fb/tile_ops/ToneMapper.cpp
  fb/tile_ops/ToneMapper.ispc

  camera/Camera.cpp
  camera/Camera.ispc
  camera/PerspectiveCamera.ispc
  camera/PerspectiveCamera.cpp
  camera/OrthographicCamera.ispc
  camera/OrthographicCamera.cpp
  camera/PanoramicCamera.ispc
  camera/PanoramicCamera.cpp

  geometry/Geometry.ispc
  geometry/Geometry.cpp
  geometry/GeometricModel.ispc
  geometry/GeometricModel.cpp
  geometry/Boxes.ispc
  geometry/Boxes.cpp
  geometry/TriangleMesh.ispc
  geometry/TriangleMesh.cpp
  geometry/Subdivision.ispc
  geometry/Subdivision.cpp
  geometry/QuadMesh.ispc
  geometry/QuadMesh.cpp
  geometry/Curves.cpp
  geometry/Curves.ispc
  geometry/StreamLines.cpp
  geometry/StreamLines.ispc
  geometry/Spheres.cpp
  geometry/Spheres.ispc
  geometry/Cylinders.cpp
  geometry/Cylinders.ispc
  geometry/Isosurfaces.ispc
  geometry/Isosurfaces.cpp

  lights/Light.ispc
  lights/Light.cpp
  lights/AmbientLight.ispc
  lights/AmbientLight.cpp
  lights/DirectionalLight.cpp
  lights/DirectionalLight.ispc
  lights/PointLight.cpp
  lights/PointLight.ispc
  lights/SpotLight.cpp
  lights/SpotLight.ispc
  lights/QuadLight.cpp
  lights/QuadLight.ispc
  lights/HDRILight.cpp
  lights/HDRILight.ispc

  math/box.ispc
  math/Distribution1D.ispc
  math/Distribution2D.ispc
  math/sobol.ih
  math/sobol.ispc
  math/spectrum.h
  math/spectrum.ih
  math/spectrum.ispc

  render/LoadBalancer.cpp
  render/Renderer.ispc
  render/Renderer.cpp
  render/util.ispc
  render/raycast/RaycastRenderer.cpp
  render/raycast/RaycastRenderer.ispc
  render/scivis/SciVis.cpp
  render/scivis/SciVis.ispc
  render/scivis/SciVisMaterial.cpp
  render/scivis/SciVisMaterial.ispc
  render/scivis/surfaces.ispc
  render/scivis/volumes.ispc

  render/pathtracer/PathTracer.ispc
  render/pathtracer/PathTracer.cpp
  render/pathtracer/PathSampler.ispc
  render/pathtracer/GeometryLight.ispc
  render/pathtracer/VirtualLight.ispc
  render/pathtracer/TransparentShadow.ispc
  render/pathtracer/ShadowCatcher.ispc
  render/pathtracer/NextEventEstimation.ispc
  render/pathtracer/bsdfs/MicrofacetAlbedoTables.ispc
  render/pathtracer/volumes/VolumeSampler.ispc
  render/pathtracer/materials/Material.ispc
  render/pathtracer/materials/OBJ.ispc
  render/pathtracer/materials/OBJ.cpp
  render/pathtracer/materials/Principled.ispc
  render/pathtracer/materials/Principled.cpp
  render/pathtracer/materials/CarPaint.ispc
  render/pathtracer/materials/CarPaint.cpp
  render/pathtracer/materials/Velvet.ispc
  render/pathtracer/materials/Velvet.cpp
  render/pathtracer/materials/Metal.ispc
  render/pathtracer/materials/Metal.cpp
  render/pathtracer/materials/Alloy.ispc
  render/pathtracer/materials/Alloy.cpp
  render/pathtracer/materials/ThinGlass.ispc
  render/pathtracer/materials/ThinGlass.cpp
  render/pathtracer/materials/Glass.ispc
  render/pathtracer/materials/Glass.cpp
  render/pathtracer/materials/MetallicPaint.ispc
  render/pathtracer/materials/MetallicPaint.cpp
  render/pathtracer/materials/Plastic.ispc
  render/pathtracer/materials/Plastic.cpp
  render/pathtracer/materials/Luminous.ispc
  render/pathtracer/materials/Luminous.cpp
  render/pathtracer/materials/Mix.ispc
  render/pathtracer/materials/Mix.cpp

  texture/Texture.cpp
  texture/Texture2D.cpp
  texture/Texture2D.ispc
  texture/TextureVolume.cpp
  texture/TextureVolume.ispc

  transferFunction/LinearTransferFunction.ispc
  transferFunction/LinearTransferFunction.cpp
  transferFunction/TransferFunction.ispc
  transferFunction/TransferFunction.cpp

  volume/Volume.ispc
  volume/Volume.cpp
  volume/VolumetricModel.ispc
  volume/VolumetricModel.cpp

  volume/structured/GridAccelerator.ispc
  volume/structured/StructuredVolume.ispc
  volume/structured/StructuredVolume.cpp

  volume/structured/shared/SharedStructuredVolume.ispc
  volume/structured/shared/SharedStructuredVolume.cpp

  volume/unstructured/MinMaxBVH2.cpp
  volume/unstructured/MinMaxBVH2.ispc
  volume/unstructured/UnstructuredVolume.cpp
  volume/unstructured/UnstructuredVolume.ispc

  volume/amr/AMRAccel.cpp
  volume/amr/AMRData.cpp
  volume/amr/AMRVolume.ispc
  volume/amr/AMRVolume.cpp
  volume/amr/CellRef.ispc
  volume/amr/DualCell.ispc
  volume/amr/method_current.ispc
  volume/amr/method_finest.ispc
  volume/amr/method_octant.ispc
)

# -------------------------------------------------------
# Install SDK headers
# -------------------------------------------------------

get_subdirectories(SUBDIRS ${CMAKE_CURRENT_LIST_DIR})

foreach(dir ${SUBDIRS})
  if (NOT "${dir}" STREQUAL include)
    install(DIRECTORY ${dir}
      DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ospray/SDK
      COMPONENT devel
      FILES_MATCHING
      PATTERN *.h
      PATTERN *.ih
    )
  endif()
endforeach()

##############################################################
# Build the ospray library
##############################################################

if (WIN32)
  list(APPEND OSPRAY_LIBS ws2_32)

  # -------------------------------------------------------
  # export ISPC SDK symbols on Windows ##
  # -------------------------------------------------------

  file(READ def_header.txt OSPRAY_DEF)
  file(READ ispc_symbols.txt OSPRAY_ISPC_SYMBOLS_IN)

  foreach(isa ${OSPRAY_ISPC_TARGET_LIST})
    string(REPLACE "-i32x16" "" isa ${isa}) # strip avx512(knl|skx)-i32x16
    # add isa suffix
    string(REPLACE "," ${isa} OSPRAY_ISPC_SYMBOLS ${OSPRAY_ISPC_SYMBOLS_IN})
    string(APPEND OSPRAY_DEF ${OSPRAY_ISPC_SYMBOLS})
  endforeach()

  file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/ospray_module_ispc.def.in "${OSPRAY_DEF}")

  # changes ospray.def only if content changed, avoids unnecessary re-linking
  configure_file(${CMAKE_CURRENT_BINARY_DIR}/ospray_module_ispc.def.in ospray_module_ispc.def COPYONLY)

  list(APPEND OSPRAY_ISPC_DEVICE_SOURCES ospray_module_ispc.def)
endif()

add_library(ospray SHARED ${OSPRAY_SOURCES})
target_link_libraries(ospray PRIVATE ospcommon::ospcommon)
ospray_install_library(ospray COMPONENT lib)
set_target_properties(ospray
  PROPERTIES VERSION ${OSPRAY_VERSION} SOVERSION ${OSPRAY_SOVERSION})

target_include_directories(ospray
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/ospray/include>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/ospray>
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

add_library(ospray_sdk INTERFACE)
target_link_libraries(ospray_sdk INTERFACE ospray ospcommon::ospcommon)
ospray_install_library(ospray_sdk COMPONENT devel)

##############################################################
# Build ISPC device
##############################################################

option(OSPRAY_BUILD_ISPC_DEVICE "Build ISPC Device" ON)
mark_as_advanced(OSPRAY_BUILD_ISPC_DEVICE)

if(OSPRAY_BUILD_ISPC_DEVICE)

  ## ISPC Device-specific options ##

  set(OSPRAY_TILE_SIZE 64 CACHE STRING "Tile size (x,y dimensions)")
  set_property(CACHE OSPRAY_TILE_SIZE PROPERTY STRINGS 8 16 32 64 128 256 512)
  mark_as_advanced(OSPRAY_TILE_SIZE)

  if (WIN32)
    set(TILE_STACK_DEFAULT_SIZE 64)
  elseif (APPLE)
    set(TILE_STACK_DEFAULT_SIZE 32)
  else ()
    set(TILE_STACK_DEFAULT_SIZE 128)
  endif()

  set(OSPRAY_MAX_STACK_TILE_SIZE ${TILE_STACK_DEFAULT_SIZE} CACHE STRING
      "Max size for tile to remain allocated on the stack")
  set_property(CACHE OSPRAY_MAX_STACK_TILE_SIZE PROPERTY STRINGS 8 16 32 64 128 256 512)
  mark_as_advanced(OSPRAY_MAX_STACK_TILE_SIZE)

  set(OSPRAY_PIXELS_PER_JOB 64 CACHE STRING
      "Must be multiple of largest vector width *and* <= OSPRAY_TILE_SIZE")
  mark_as_advanced(OSPRAY_PIXELS_PER_JOB)

  configure_file(common/OSPConfig.h.in ../OSPConfig.h)
  install(FILES ${CMAKE_BINARY_DIR}/OSPConfig.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ospray/SDK
    COMPONENT devel
  )

  ## Build the module ##

  ispc_include_directories(
    ${CMAKE_SOURCE_DIR}/ospray/include
    ${CMAKE_SOURCE_DIR}/ospray
    ${CMAKE_SOURCE_DIR}
    ${PROJECT_BINARY_DIR}
    ${EMBREE_INCLUDE_DIRS}
  )

  add_library(ospray_module_ispc SHARED)

  ispc_target_add_sources(ospray_module_ispc ${OSPRAY_ISPC_DEVICE_SOURCES})

  ospray_install_library(ospray_module_ispc COMPONENT lib)

  target_link_libraries(ospray_module_ispc
  PUBLIC
    ospray_sdk
    $<BUILD_INTERFACE:embree>
  )

  target_include_directories(ospray_module_ispc
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/ospray>
      $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
      $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/ospray/SDK>
      $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  )

  set_target_properties(ospray_module_ispc
    PROPERTIES VERSION ${OSPRAY_VERSION} SOVERSION ${OSPRAY_SOVERSION})

endif()

##############################################################
# Configure find_package files
##############################################################

include(${CMAKE_SOURCE_DIR}/cmake/ospray_cmake_config.cmake)
