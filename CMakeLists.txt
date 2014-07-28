cmake_minimum_required(VERSION 2.8.12) 
project(daemon)

add_subdirectory(../libsfp libsfp)
add_subdirectory(../ribbon-bridge rpc)
add_subdirectory(../serial serial)
add_subdirectory(../interfaces interfaces)

add_executable(daemon daemon.cpp $<TARGET_OBJECTS:robotpb>
    ${robotinterface_SOURCE_DIR}/gen-robot.pb.cpp)

set_source_files_properties(daemon.cpp ${robotinterface_SOURCE_DIR}/gen-robot.pb.cpp 
    PROPERTIES COMPILE_FLAGS "-std=c++11")

add_definitions(-DHAVE_STDLIB -DHAVE_EXCEPTIONS -DHAVE_RTTI)
target_include_directories(daemon PRIVATE "${LIBSFP_SOURCE_DIR}/include" 
                                  PRIVATE "$ENV{BOOST_ROOT}"
                                  PRIVATE "${robotinterface_SOURCE_DIR}"
                                  PRIVATE "${PROJECT_BINARY_DIR}/interfaces"
                                  PRIVATE "${serial_SOURCE_DIR}/include")
target_link_libraries(daemon sfp rpc serial)

enable_testing()
add_subdirectory(robottests)
