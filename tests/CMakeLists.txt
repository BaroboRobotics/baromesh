add_executable(lavalamp lavalamp.cpp)
target_link_libraries(lavalamp baromesh)
target_compile_options(lavalamp PRIVATE "-std=c++11")
add_test(NAME lavalamp COMMAND lavalamp)