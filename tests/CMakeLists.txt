# Layer 0 单元测试（CTest）。也可用 tests/run_tests.sh 直接 g++ 构建。
foreach(t test_kinematics test_planning test_sim test_json_ws test_serial test_safety)
  add_executable(${t} ${t}.cpp)
  target_include_directories(${t} PRIVATE ${CMAKE_SOURCE_DIR}/lib ${CMAKE_SOURCE_DIR})
  if(UNIX AND NOT APPLE)
    target_link_libraries(${t} PRIVATE pthread)
  endif()
  add_test(NAME ${t} COMMAND ${t})
endforeach()

if(ARM_ENABLE_CTORCH)
  # 学习层测试：test_learn（学习层单元）、test_ctorch_transpose_grad（A1 补丁回归）、
  # test_train_regression（A3 训练可复现性基准）
  foreach(t test_learn test_ctorch_transpose_grad test_train_regression)
    add_executable(${t} ${t}.cpp)
    target_include_directories(${t} PRIVATE
      ${CMAKE_SOURCE_DIR}/lib
      ${CMAKE_SOURCE_DIR}
      ${CMAKE_SOURCE_DIR}/ctorch_ext
      ${CTORCH_ROOT}/include
      ${CTORCH_ROOT}/src
      ${CTORCH_ROOT}/c3/include
    )
    target_compile_features(${t} PRIVATE cxx_std_20)
    target_link_libraries(${t} PRIVATE CTorch)
    if(UNIX AND NOT APPLE)
      target_link_libraries(${t} PRIVATE pthread)
    endif()
    add_test(NAME ${t} COMMAND ${t})
  endforeach()
endif()
