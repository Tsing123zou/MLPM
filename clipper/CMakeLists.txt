cmake_minimum_required(VERSION 3.30)

project(Clipper2)

# 强制所有目标文件支持 -fPIC
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 指定源代码目录
set(SRC_DIR ${CMAKE_SOURCE_DIR}/lib)

# 包含头文件
include_directories(${CMAKE_SOURCE_DIR}/include)

# 设置源文件
file(GLOB_RECURSE SOURCES ${SRC_DIR}/*.cpp)

# 创建静态库
add_library(Clipper2 STATIC ${SOURCES})

# 设置输出目录
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)

# 链接依赖的其他库（如果需要）
# target_link_libraries(Clipper2 <other-libraries-if-needed>)
