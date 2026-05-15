cmake_minimum_required(VERSION 3.14)
project(sophus_deskewer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(pybind11 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(TBB REQUIRED)

pybind11_add_module(sophus_deskewer deskewer.cpp)

target_include_directories(sophus_deskewer
    PRIVATE
        ${EIGEN3_INCLUDE_DIRS}
        /usr/local/include   # this contains sophus/
)

target_link_libraries(sophus_deskewer
    PRIVATE Eigen3::Eigen
    TBB::tbb
)