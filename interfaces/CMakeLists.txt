cmake_minimum_required(VERSION 3.2.1)
project(baromesh-interfaces)

nanopb_add_proto(commontypes-proto commontypes.proto)

nanopb_add_proto(robot-interface robot.proto gen-robot.pb.cpp)
nanopb_add_proto(dongle-interface dongle.proto gen-dongle.pb.cpp)
nanopb_add_proto(daemon-interface daemon.proto gen-daemon.pb.cpp)

set_target_properties(robot-interface dongle-interface daemon-interface
    PROPERTIES CXX_STANDARD 11
               CXX_STANDARD_REQUIRED ON
			   )

if(MSVC)
	# Without this, the PB_STATIC_ASSERT line generated in robot.pb.c will
	# cause MSVC to choke on a 4K+ long token (error C1064).
	target_compile_definitions(robot-interface PRIVATE PB_NO_STATIC_ASSERT)
	message(WARNING "Disabling PB_STATIC_ASSERT for robot-interface due to Visual C++ token length limit")
endif()

target_link_libraries(commontypes-proto rpc-proto)

target_link_libraries(robot-interface commontypes-proto rpc)
target_link_libraries(dongle-interface commontypes-proto rpc)
target_link_libraries(daemon-interface commontypes-proto rpc)
