#add_executable(lavalamp lavalamp.cpp)
#target_link_libraries(lavalamp baromesh)
#target_compile_options(lavalamp PRIVATE "-std=c++11")
#add_test(NAME lavalamp COMMAND lavalamp)

add_executable(testmovement testmovement.cpp)
target_link_libraries(testmovement baromesh)
target_compile_options(testmovement PRIVATE "-std=c++11")
add_test(NAME testmovement COMMAND testmovement)
