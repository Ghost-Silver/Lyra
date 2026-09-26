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

# ---- Layer 0 实时内核 + 主程序（0 依赖，自包含 socket + json）----
add_executable(arm_sim
  app/main.cpp
)
target_include_directories(arm_sim PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/lib
  ${CMAKE_CURRENT_SOURCE_DIR}
)
if(UNIX AND NOT APPLE)
  target_link_libraries(arm_sim PRIVATE pthread)
endif()

# ---- Layer 1 学习层（可选，CTorch）----
option(ARM_ENABLE_CTORCH "启用 CTorch 学习层 (learning/)" OFF)
set(CTORCH_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/CTorch" CACHE PATH "CTorch 仓库根")

if(ARM_ENABLE_CTORCH)
  if(NOT EXISTS "${CTORCH_ROOT}/CMakeLists.txt")
    message(FATAL_ERROR "CTORCH_ROOT 无效: ${CTORCH_ROOT}（先 git submodule update --init --recursive）")
  endif()
  # CTorch 子工程开关：关 MLIR(免 LLVM) 与 ThinLTO(GCC 不识别)，跳过其测试
  set(CT_ENABLE_MLIR OFF CACHE BOOL "" FORCE)
  set(CT_ENABLE_LTO OFF CACHE BOOL "" FORCE)
  set(FETCH_GOOGLETEST OFF CACHE BOOL "" FORCE)
  # MLIR=OFF 时 C3 的图融合/JIT 管线无可用后端（每次尝试都失败并回落 eager，
  # 刷 ERROR 日志且白耗时间）；学习层用 eager+autograd，数值等价，编译期关掉
  set(CT_C3_DISABLE_SINGLE_KERNEL ON CACHE BOOL "" FORCE)
  set(CT_C3_DISABLE_REGION_FUSION ON CACHE BOOL "" FORCE)
  set(CT_C3_DISABLE_BACKWARD ON CACHE BOOL "" FORCE)
  set(CT_C3_DISABLE_HOTPATH ON CACHE BOOL "" FORCE)
  # GCC 12 无 __bf16 内建类型（CTorch 的 CoreDefs.h 需要），强制包含兼容垫片
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 13)
    add_compile_options(-include ${CMAKE_CURRENT_SOURCE_DIR}/cmake/compat_bf16.h)
    message(STATUS "GCC ${CMAKE_CXX_COMPILER_VERSION}: 启用 __bf16 兼容垫片")
  endif()
  add_subdirectory(${CTORCH_ROOT} ${CMAKE_BINARY_DIR}/ctorch EXCLUDE_FROM_ALL)
  add_subdirectory(learning)
endif()

# ---- 测试 ----
include(CTest)
if(BUILD_TESTING)
  add_subdirectory(tests)
endif()
