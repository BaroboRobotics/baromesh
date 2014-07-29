cmake_minimum_required(VERSION 2.8.12) 
project(daemon)

add_subdirectory(../libsfp libsfp)
add_subdirectory(../ribbon-bridge rpc)
add_subdirectory(../ribbon-bridge-interfaces ribbon-bridge-interfaces)
add_subdirectory(../serial serial)

add_executable(daemon daemon.cpp $<TARGET_OBJECTS:robotpb>
    ${robotinterface_SOURCE_DIR}/gen-robot.pb.cpp)

set_source_files_properties(daemon.cpp ${robotinterface_SOURCE_DIR}/gen-robot.pb.cpp 
    PROPERTIES COMPILE_FLAGS "-std=c++11")

add_definitions(-DHAVE_STDLIB -DHAVE_EXCEPTIONS -DHAVE_RTTI)
target_include_directories(daemon PRIVATE "${LIBSFP_SOURCE_DIR}/include" 
                                  PRIVATE "$ENV{BOOST_ROOT}"
                                  PRIVATE $<TARGET_PROPERTY:robotpb,INCLUDE_DIRECTORIES>
                                  PRIVATE "${serial_SOURCE_DIR}/include")
target_link_libraries(daemon sfp rpc serial)
