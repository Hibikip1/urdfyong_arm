cmake_minimum_required(VERSION 3.10)

project (dm_Linux_Drive)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)


add_executable(test_damiao src/test_damiao.cpp)
add_executable(test_3rads src/test_3rads.cpp)
add_executable(set_zero_test src/set_zero_test.cpp)


include_directories(${CMAKE_SOURCE_DIR}/include)