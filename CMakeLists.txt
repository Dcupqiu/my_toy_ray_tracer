#---------------------------------------------------------------------------------------------------
# CMake Build Configuration for the Ray Tracing Weekend Series
#
# See README.md for guidance.
#---------------------------------------------------------------------------------------------------

cmake_minimum_required ( VERSION 3.1.0 )

project ( RTWeekend
  VERSION 3.0.0
  LANGUAGES CXX
)

# Set to c++11
set ( CMAKE_CXX_STANDARD 11 )

# Source
set ( COMMON_ALL
  src/common/rtweekend.h
  src/common/camera.h
  src/common/ray.h
  src/common/vec3.h
)

FIND_PACKAGE(OpenCV REQUIRED)

FIND_PACKAGE(OpenMP REQUIRED)
if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()




set ( SOURCE_NEXT_WEEK
  ${COMMON_ALL}
  src/common/aabb.h
  src/common/external/stb_image.h
  src/common/perlin.h
  src/common/rtw_stb_image.h
  src/common/texture.h
  src/TheNextWeek/aarect.h
  src/TheNextWeek/box.h
  src/TheNextWeek/bvh.h
  src/TheNextWeek/constant_medium.h
  src/TheNextWeek/hittable.h
  src/TheNextWeek/hittable_list.h
  src/TheNextWeek/material.h
  src/TheNextWeek/moving_sphere.h
  src/TheNextWeek/sphere.h
  src/TheNextWeek/main.cc
)

set ( SOURCE_REST_OF_YOUR_LIFE
  ${COMMON_ALL}
  src/common/aabb.h
  src/common/external/stb_image.h
  src/common/perlin.h
  src/common/rtw_stb_image.h
  src/common/texture.h
  src/TheRestOfYourLife/aarect.h
  src/TheRestOfYourLife/box.h
  src/TheRestOfYourLife/bvh.h
  src/TheRestOfYourLife/hittable.h
  src/TheRestOfYourLife/hittable_list.h
  src/TheRestOfYourLife/material.h
  src/TheRestOfYourLife/onb.h
  src/TheRestOfYourLife/pdf.h
  src/TheRestOfYourLife/sphere.h
  src/TheRestOfYourLife/main.cc
)
link_directories(${OpenCV_LIBRARY_DIRS})
# Executables
add_executable(theNextWeek       ${SOURCE_NEXT_WEEK})
add_executable(theRestOfYourLife ${SOURCE_REST_OF_YOUR_LIFE})
add_executable(cos_cubed         src/TheRestOfYourLife/cos_cubed.cc         ${COMMON_ALL})
add_executable(cos_density       src/TheRestOfYourLife/cos_density.cc       ${COMMON_ALL})
add_executable(integrate_x_sq    src/TheRestOfYourLife/integrate_x_sq.cc    ${COMMON_ALL})
add_executable(pi                src/TheRestOfYourLife/pi.cc                ${COMMON_ALL})
add_executable(sphere_importance src/TheRestOfYourLife/sphere_importance.cc ${COMMON_ALL})
add_executable(sphere_plot       src/TheRestOfYourLife/sphere_plot.cc       ${COMMON_ALL})
target_link_libraries(theNextWeek ${OpenCV_LIBS})
include_directories(src/common)
