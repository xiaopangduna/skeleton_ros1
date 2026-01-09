# # 创建测试辅助库
# add_library(test_utils STATIC
#   utils.cpp)

# # 设置测试辅助库的包含目录
# target_include_directories(test_utils PUBLIC
#     ${CMAKE_CURRENT_SOURCE_DIR}
# )

# 定义创建测试的函数
function(create_test TEST_NAME)
  # 创建测试可执行文件
  add_executable(${TEST_NAME} ${ARGN})
  
  # 设置包含目录
  target_include_directories(${TEST_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

  target_link_libraries(${TEST_NAME} PRIVATE
      skeleton_cpp
      # test_utils
      GTest::gtest
      GTest::gtest_main)

  # 添加到测试列表
  add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
  # 设置测试工作目录
  set_tests_properties(${TEST_NAME} PROPERTIES 
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/.."
  )
endfunction()

# 使用函数创建测试
create_test(calculator_test calculator_test.cpp)
create_test(subtract_test subtract_test.cpp)

