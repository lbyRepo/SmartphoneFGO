cmake_minimum_required(VERSION 3.22.1)
project(navgraph_estimator LANGUAGES CXX)

add_library(navgraph_estimator SHARED
    core/frame.cpp
    ekf/ekf.cpp
    fgo/fgo.cpp
    gnss/gnss.cpp
    ins/ins.cpp
    jni/navgraph_jni.cpp
)

target_include_directories(navgraph_estimator PRIVATE include)
target_compile_features(navgraph_estimator PRIVATE cxx_std_17)
target_compile_options(navgraph_estimator PRIVATE -Wall -Wextra -Werror -pedantic)
