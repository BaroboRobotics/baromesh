add_executable(lavalamp lavalamp.cpp)
target_link_libraries(lavalamp baromesh)
target_compile_options(lavalamp PRIVATE "-std=c++11")
add_test(NAME lavalamp COMMAND lavalamp)

add_executable(lavalamp-linkbot-api lavalamp-linkbot-api.cpp)
target_link_libraries(lavalamp-linkbot-api baromesh)
target_compile_options(lavalamp-linkbot-api PRIVATE "-std=c++11")
add_test(NAME lavalamp-linkbot-api COMMAND lavalamp-linkbot-api)
