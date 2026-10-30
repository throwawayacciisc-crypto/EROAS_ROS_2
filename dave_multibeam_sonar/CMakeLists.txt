cmake_minimum_required(VERSION 3.16)
project(dave_multibeam_sonar LANGUAGES CXX)

# DAVE multibeam sonar (IOES-Lab/dave, gazebo/dave_gz_multibeam_sonar),
# built against Gazebo Harmonic / ROS 2 Jazzy instead of Jetty / Lyrical.
# Produces the same two plugins as upstream:
#   libmultibeam_sonar.so         custom gz-sensors RenderingSensor
#   libmultibeam_sonar_system.so  gz-sim system that creates/updates it

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

include(CheckLanguage)
check_language(CUDA)
if(NOT CMAKE_CUDA_COMPILER)
  message(WARNING "nvcc not found: skipping DAVE multibeam sonar plugins")
  ament_package()
  return()
endif()

enable_language(CUDA)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
# Pascal (GTX 10xx) and newer; override with -DCMAKE_CUDA_ARCHITECTURES=...
if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
  set(CMAKE_CUDA_ARCHITECTURES 60)
endif()

find_package(CUDAToolkit REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(gz-common5 REQUIRED COMPONENTS profiler)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-math7 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-rendering8 REQUIRED)
find_package(gz-sensors8 REQUIRED COMPONENTS rendering)
find_package(gz-sim8 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(marine_acoustic_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

add_library(multibeam_sonar SHARED
  src/MultibeamSonarSensor.cc
  src/sonar_calculation_cuda.cu
)
set_target_properties(multibeam_sonar PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
target_include_directories(multibeam_sonar PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(multibeam_sonar
  gz-sim8::gz-sim8
  gz-sensors8::gz-sensors8
  gz-sensors8::gz-sensors8-rendering
  gz-rendering8::gz-rendering8
  gz-msgs10::gz-msgs10
  gz-transport13::gz-transport13
  gz-math7::gz-math7
  gz-common5::gz-common5
  CUDA::cudart
  CUDA::cufft
  CUDA::cublas
  ${OpenCV_LIBS}
)
ament_target_dependencies(multibeam_sonar
  rclcpp sensor_msgs geometry_msgs marine_acoustic_msgs cv_bridge
)

add_library(multibeam_sonar_system SHARED src/MultibeamSonarSystem.cc)
target_link_libraries(multibeam_sonar_system
  multibeam_sonar
  gz-common5::gz-common5
  gz-plugin2::register
  gz-sim8::gz-sim8
  gz-sensors8::gz-sensors8
)

install(TARGETS multibeam_sonar multibeam_sonar_system
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
