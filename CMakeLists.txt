cmake_minimum_required(VERSION 3.10)
project(BronKerboshMatch)
add_compile_definitions(GLOG_USE_GLOG_EXPORT)
# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)


option(BUILD_apps "Build application programs" ON)
option(BUILD_test "Build test programs" OFF)
option(BUILD_PYTHON_BINDINGS "Build python bindings" OFF)

#set(CMAKE_BUILD_TYPE "Release")

# 查找 Eigen 库
find_package(Eigen3 REQUIRED)
set(PCL_DIR "/home/ecarx/pcl/install/share/pcl-1.13")
find_package(PCL REQUIRED)
# 添加 GLOG 库
find_package(glog REQUIRED)
# 添加 Boost 库
find_package(Boost REQUIRED)

# 添加 include 目录
include_directories(include ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} ${GLOG_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

# 添加源文件
file(GLOB SOURCES "src/*.cpp" "src/recognizers/*.cpp")

# 创建库
add_library(${PROJECT_NAME}_Lib STATIC ${SOURCES})
target_link_libraries(${PROJECT_NAME}_Lib glog::glog ${PCL_LIBRARIES} ${Boost_LIBRARIES})

# 创建可执行文件
# add_executable(${PROJECT_NAME} src/main.cpp)  # 假设你有一个 main.cpp 文件
# target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_Lib)

# 添加 Google Test
enable_testing()
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})


# 创建测试可执行文件
add_executable(runTests test/bron_kerbosch_gtest.cpp)
target_link_libraries(runTests ${PROJECT_NAME}_Lib ${GTEST_LIBRARIES} ${PCL_LIBRARIES} ${GLOG_LIBRARIES} ${Boost_LIBRARIES} pthread)

# 添加测试
add_test(NAME GeometricConsistencyRecognizer COMMAND runTests)
