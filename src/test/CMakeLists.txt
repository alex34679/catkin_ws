cmake_minimum_required(VERSION 3.10)

# 项目名称
project(QuaternionYawCalculator)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 查找Eigen库
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# 添加可执行文件
add_executable(QuaternionYawCalculator test.cpp)

# 链接Eigen库
target_link_libraries(QuaternionYawCalculator Eigen3::Eigen)
