add_executable(smoke_test smoke_test.cpp)
target_link_libraries(smoke_test PRIVATE ctlkit::ctl)

add_test(NAME smoke COMMAND smoke_test)
