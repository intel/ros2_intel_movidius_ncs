# Copyright (c) 2017 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)
project(movidius_ncs_stream)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(class_loader REQUIRED)
find_package(object_msgs REQUIRED)
find_package(movidius_ncs_lib REQUIRED)
find_package(rviz_yaml_cpp_vendor REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(include
  ${rviz_yaml_cpp_vendor_DIR}/../../../opt/rviz_yaml_cpp_vendor/include)

add_library(${PROJECT_NAME} SHARED
  src/ncs_composition.cpp)
ament_target_dependencies(${PROJECT_NAME}
  "ament_index_cpp"
  "class_loader"
  "rclcpp"
  "rviz_yaml_cpp_vendor"
  "sensor_msgs"
  "movidius_ncs_lib"
  "object_msgs"
  "OpenCV"
  "cv_bridge")
rclcpp_register_node_plugins(${PROJECT_NAME} "movidius_ncs_stream::NCSComposition")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Flags
if(UNIX OR APPLE)
  # Linker flags.
  if(${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU" OR ${CMAKE_CXX_COMPILER_ID} STREQUAL "Intel")
    # GCC specific flags. ICC is compatible with them.
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -z noexecstack -z relro -z now")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -z noexecstack -z relro -z now")
  elseif(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    # In Clang, -z flags are not compatible, they need to be passed to linker via -Wl.
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-z,noexecstack -Wl,-z,relro -Wl,-z,now")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-z,noexecstack -Wl,-z,relro -Wl,-z,now")
  endif()

  # Compiler flags.
  if(${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
    # GCC specific flags.
    if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 4.9 OR CMAKE_CXX_COMPILER_VERSION VERSION_EQUAL 4.9)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIE -fstack-protector-strong")
    else()
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIE -fstack-protector")
    endif()
  elseif(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    # Clang is compatbile with some of the flags.
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIE -fstack-protector")
  elseif(${CMAKE_CXX_COMPILER_ID} STREQUAL "Intel")
    # Same as above, with exception that ICC compilation crashes with -fPIE option, even
    # though it uses -pie linker option that require -fPIE during compilation. Checksec
    # shows that it generates correct PIE anyway if only -pie is provided.
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fstack-protector")
  endif()

  # Generic flags.
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -fno-operator-names -Wformat -Wformat-security -Wall")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
  # Dot not forward c++14 flag to GPU beucause it is not supported
  set(CUDA_PROPAGATE_HOST_FLAGS OFF)
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -D_FORTIFY_SOURCE=2")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pie")
endif()

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
