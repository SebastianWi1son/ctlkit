add_executable(smoke_test smoke_test.cpp)
target_link_libraries(smoke_test PRIVATE ctlkit::ctl)
add_test(NAME smoke COMMAND smoke_test)

# 黄金向量回归：期望值来自 oracle（tests/golden/*.csv），容差记录在 CSV 头部。
# 用 CONFIGURE_DEPENDS 让新增 csv 自动进构建；文件列表由 CMake 传给测试可执行文件。
file(GLOB CTLKIT_GOLDEN_CSV CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/golden/*.csv)
add_executable(golden_test golden_test.cpp)
target_link_libraries(golden_test PRIVATE ctlkit::ctl)
add_test(NAME golden COMMAND golden_test ${CTLKIT_GOLDEN_CSV})
