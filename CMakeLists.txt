cmake_minimum_required ( VERSION 3.1.0 )

project ( RTWeekend
  VERSION 3.0.0
  LANGUAGES CXX
)

# Set to c++11
set ( CMAKE_CXX_STANDARD 17 )

# Source
set ( COMMON_ALL
  src/common/rtweekend.h
  src/common/camera.h
  src/common/ray.h
  src/common/vec3.h
  src/common/OBJ_Loader.hpp
)

FIND_PACKAGE(OpenCV REQUIRED)

FIND_PACKAGE(OpenMP REQUIRED)
if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()


set ( TOY_RAY_TRACER
  ${COMMON_ALL}
  src/common/aabb.h
  src/common/external/stb_image.h
  src/common/perlin.h
  src/common/rtw_stb_image.h
  src/common/texture.h
  src/Main/aarect.h
  src/Main/box.h
  src/Main/bvh.h
  src/Main/constant_medium.h
  src/Main/hittable.h
  src/Main/hittable_list.h
  src/Main/material.h
  src/Main/moving_sphere.h
  src/Main/sphere.h
  src/Main/main.cc
  src/Main/mesh_triangle.h
)


link_directories(${OpenCV_LIBRARY_DIRS})
# Executables
add_executable(toy_ray_tracer       ${TOY_RAY_TRACER})
target_link_libraries(toy_ray_tracer ${OpenCV_LIBS})
include_directories(src/common)
