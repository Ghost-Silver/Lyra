cmake_minimum_required(VERSION 3.16)
project(desk_6axis_arm CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra -O2)
if(APPLE)
  # enable native vectorization (NEON/AMX) — irrelevant on x86 but harmless guard
  set(CMAKE_OSX_DEPLOYMENT_TARGET "11.0" CACHE STRING "" FORCE)
endif()

add_executable(arm_sim
  app/main.cpp
)
target_include_directories(arm_sim PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# reasonable; no external deps needed (self-contained socket + json)