#cmake_minimum_required(VERSION 3.16)
cmake_minimum_required(VERSION 3.30)

set(CMAKE_CXX_COMPILER "/home/gbzou/gcc-1320/bin/g++")
set(CMAKE_C_COMPILER "/home/gbzou/gcc-1320/bin/gcc")
project(pm)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#set(CMAKE_CXX_EXTENSIONS ON)

set(TBB_INCLUDE_DIR "/home/gbzou/oneapi-tbb-2021.11.0/include")
set(TBB_LIB_DIR "/home/gbzou/oneapi-tbb-2021.11.0/lib/intel64/gcc4.8")

add_compile_options(-O2)

add_executable(pm
        main.cpp
        src/layout.cpp
        src/read.cpp
        src/check.cpp
        src/test.cpp
        src/qtreebase.cpp
        src/msqtree.cpp
        src/config.cpp
)

target_include_directories(pm BEFORE PRIVATE
        include
        3rd/mio/include
        ${TBB_INCLUDE_DIR}
        #3rd/tbb/include/oneapi
        #3rd/tbb/include
        #3rd/gperftools/include
        #3rd/jemalloc/include
        3rd/clipper/include/clipper2
)

# 链接库的搜索路径
target_link_directories(pm PRIVATE
        ${TBB_LIB_DIR}
        #3rd/tbb/lib
        #3rd/gperftools/lib
        #3rd/jemalloc/lib
        3rd/clipper/lib
)

# 链接库名称
target_link_libraries(pm
        #libtbb.a
        libtbb.so
        #TBB::tbb
        pthread
        libClipper2.a
        dl
)
