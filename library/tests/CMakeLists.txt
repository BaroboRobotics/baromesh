add_executable(lavalamp lavalamp.cpp)
target_link_libraries(lavalamp baromesh)
target_compile_options(lavalamp PRIVATE "-std=c++11")
add_test(NAME lavalamp COMMAND lavalamp)

add_executable(testmovement testmovement.cpp)
target_link_libraries(testmovement baromesh)
target_compile_options(testmovement PRIVATE "-std=c++11")
add_test(NAME testmovement COMMAND testmovement)

add_executable(speeddial speeddial.cpp)
target_link_libraries(speeddial baromesh)
target_compile_options(speeddial PRIVATE "-std=c++11")
add_test(NAME speeddial COMMAND speeddial)

add_executable(test_c test_c.cpp)
target_link_libraries(test_c baromesh)
add_test(NAME test_c COMMAND test_c)
