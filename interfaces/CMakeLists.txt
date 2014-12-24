cmake_minimum_required(VERSION 2.8)
project(baromesh-interfaces)

nanopb_add_proto(commontypes-proto commontypes.proto)

nanopb_add_proto(robot-interface robot.proto gen-robot.pb.cpp)
nanopb_add_proto(dongle-interface dongle.proto gen-dongle.pb.cpp)
nanopb_add_proto(daemon-interface daemon.proto gen-daemon.pb.cpp)

set_source_files_properties(
    gen-robot.pb.cpp
    gen-dongle.pb.cpp
    gen-daemon.pb.cpp
    PROPERTIES COMPILE_FLAGS -std=c++11)

target_link_libraries(robot-interface commontypes-proto rpc)
target_link_libraries(dongle-interface commontypes-proto rpc)
target_link_libraries(daemon-interface commontypes-proto rpc)
