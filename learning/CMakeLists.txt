# learning/ — Layer 1 CTorch 学习层（ARM_ENABLE_CTORCH=ON 时构建）
add_executable(arm_train
  trainer_main.cpp
)
target_include_directories(arm_train PRIVATE
  ${CMAKE_SOURCE_DIR}/lib
  ${CMAKE_SOURCE_DIR}
  ${CMAKE_SOURCE_DIR}/ctorch_ext
  ${CTORCH_ROOT}/include
  ${CTORCH_ROOT}/src
  ${CTORCH_ROOT}/c3/include
)
target_compile_features(arm_train PRIVATE cxx_std_20)
target_link_libraries(arm_train PRIVATE CTorch)
if(UNIX AND NOT APPLE)
  target_link_libraries(arm_train PRIVATE pthread)
endif()
