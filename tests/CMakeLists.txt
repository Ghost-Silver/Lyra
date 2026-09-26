# Layer 0 单元测试（CTest）。也可用 tests/run_tests.sh 直接 g++ 构建。
foreach(t test_kinematics test_planning test_sim test_json_ws test_serial)
  add_executable(${t} ${t}.cpp)
  target_include_directories(${t} PRIVATE ${CMAKE_SOURCE_DIR}/lib ${CMAKE_SOURCE_DIR})
  if(UNIX AND NOT APPLE)
    target_link_libraries(${t} PRIVATE pthread)
  endif()
  add_test(NAME ${t} COMMAND ${t})
endforeach()

if(ARM_ENABLE_CTORCH)
  add_executable(test_learn test_learn.cpp)
  target_include_directories(test_learn PRIVATE
    ${CMAKE_SOURCE_DIR}/lib
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/ctorch_ext
    ${CTORCH_ROOT}/include
    ${CTORCH_ROOT}/src
    ${CTORCH_ROOT}/c3/include
  )
  target_compile_features(test_learn PRIVATE cxx_std_20)
  target_link_libraries(test_learn PRIVATE CTorch)
  if(UNIX AND NOT APPLE)
    target_link_libraries(test_learn PRIVATE pthread)
  endif()
  add_test(NAME test_learn COMMAND test_learn)
endif()
