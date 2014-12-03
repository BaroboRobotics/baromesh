cmake_minimum_required(VERSION 2.8.12) 
project(baromesh CXX)

##############################################################################
# Boost

# FindBoost.cmake uses BOOST_ROOT as a hint for where to look.
if(NOT DEFINED BOOST_ROOT)
    if(DEFINED ENV{BOOST_ROOT})
        set(BOOST_ROOT "$ENV{BOOST_ROOT}")
    else()
        message(WARNING "BOOST_ROOT is undefined.\nIf Boost libraries fail to "
            "be found, try setting -DBOOST_ROOT=/path/to/boost on the CMake "
            "command line.\n")
    endif()
endif()

# Linking statically to Boost is easier for prototyping. For multi-module
# projects we'll want to use shared libraries and defined -DBOOST_ALL_DYN_LINK
# or the like.
set(Boost_USE_STATIC_LIBS ON)
find_package(Boost 1.54.0 REQUIRED COMPONENTS system log filesystem thread coroutine context)

find_package(Threads)

set(LIBRARY_SOURCES
    src/linkbot.cpp
    src/linkbot.c.cpp
    src/system_error.cpp
    src/iocore.cpp
    src/daemon.cpp
    )

add_library(baromesh ${LIBRARY_SOURCES})

set_source_files_properties(${LIBRARY_SOURCES}
    PROPERTIES COMPILE_FLAGS "-std=c++11")

target_include_directories(baromesh
    PUBLIC ${Boost_INCLUDE_DIRS}
           include
    )
target_link_libraries(baromesh
    PUBLIC sfp rpc cxx-util
           robot-interface daemon-interface
           ${Boost_LIBRARIES}
           ${CMAKE_THREAD_LIBS_INIT}
    )

set(DAEMON_SOURCES
    src/daemon/main.cpp
    src/daemon/dongledevicepath.cpp
    src/daemon/rundongle.cpp
    src/system_error.cpp
    )

add_executable(baromeshd ${DAEMON_SOURCES})
set_source_files_properties(${DAEMON_SOURCES}
    PROPERTIES COMPILE_FLAGS "-std=c++11")
target_include_directories(baromeshd
    PRIVATE ${Boost_INCLUDE_DIRS}
            include
    )
target_link_libraries(baromeshd
    PRIVATE sfp rpc cxx-util
            dongle-interface daemon-interface
            ${Boost_LIBRARIES}
            ${CMAKE_THREAD_LIBS_INIT}
   )

if(WIN32)
    target_link_libraries(baromeshd
        PRIVATE setupapi ws2_32 mswsock
        )
endif()

option(BAROMESH_BUILD_TESTS "Build baromesh tests" OFF)
if(BAROMESH_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
