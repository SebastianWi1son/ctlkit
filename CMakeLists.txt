cmake_minimum_required(VERSION 3.16)
project(ctlkit VERSION 0.1.0 LANGUAGES CXX)

# ctlkit —— 嵌入式实时控制原语（上游库）
# 本 CMake 面向主机端：构建静态库 + 跑单测/示例。
# 嵌入式下游消费方式见 README「下游消费方式」（拷贝 include/ctl + src/*.cpp）。

add_library(ctl STATIC
    src/pid.cpp
    src/lpf.cpp
    src/ramp.cpp
    src/smooth_planner.cpp
    src/deadzone.cpp
)
add_library(ctlkit::ctl ALIAS ctl)

target_include_directories(ctl PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/inc)
target_compile_features(ctl PUBLIC cxx_std_11)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(ctl PRIVATE -Wall -Wextra)
endif()

# 仅在作为顶层项目时构建测试/示例（被 add_subdirectory 引入时安静）
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    option(CTLKIT_BUILD_TESTS "Build host tests" ON)
    option(CTLKIT_BUILD_EXAMPLES "Build examples" ON)

    if(CTLKIT_BUILD_TESTS)
        enable_testing()
        add_subdirectory(tests)
    endif()

    if(CTLKIT_BUILD_EXAMPLES)
        add_subdirectory(examples)
    endif()
endif()
