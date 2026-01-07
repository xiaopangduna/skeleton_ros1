# FindThirdParty.cmake - 第三方库路径设置模块

#.rst:
# 设置THIRD_PARTY_DIR目录为第三方库的路径
message(STATUS "==============================================================================")
message(STATUS "CMAKE_VERSION: ${CMAKE_VERSION}")
message(STATUS "CMAKE_SYSTEM_NAME: ${CMAKE_SYSTEM_NAME}")
message(STATUS "CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "==============================================================================")

# 全局设置：优先 Config 模式（存入缓存，全局生效）CMake 3.15+才支持
set(CMAKE_FIND_PACKAGE_PREFER_CONFIG TRUE CACHE BOOL 
    "Prefer Config mode over Module mode for find_package")

# 设置第三方库目录
set(THIRD_PARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party")
message(STATUS "THIRD_PARTY_DIR: ${THIRD_PARTY_DIR}")
# 导入自定义的查找模块
include(FindOpenCVCustom)
include(FindSpdlogCustom)
include(FindYamlCppCustom)

message(STATUS "已完成第三方库配置")