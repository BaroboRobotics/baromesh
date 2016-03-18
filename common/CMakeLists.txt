cmake_minimum_required(VERSION 3.2.1)
project(baromesh-common VERSION 0.2.0)

if(WIN32 AND NOT CMAKE_COMPILER_IS_GNUCXX)
    add_definitions("-D_ALLOW_KEYWORD_MACROS")
    #set_target_properties(${target} PROPERTIES
    #  COMPILE_FLAGS "-D_ALLOW_KEYWORD_MACROS"
    #)
endif()

# Linking statically to Boost is easier for prototyping. For multi-module
# projects we'll want to use shared libraries and defined -DBOOST_ALL_DYN_LINK
# or the like.
set(Boost_USE_STATIC_LIBS ON)
find_package(Boost 1.59.0 REQUIRED COMPONENTS system chrono log thread program_options)

set(SOURCES
    src/dongledevicepath.cpp
    src/log.cpp
    src/system_error.cpp
    )

add_library(baromesh-common STATIC ${SOURCES})

set_target_properties(baromesh-common
    PROPERTIES CXX_STANDARD 11
               CXX_STANDARD_REQUIRED ON
               )

target_compile_definitions(baromesh-common
    PUBLIC _WEBSOCKETPP_NO_CPP11_SYSTEM_ERROR_ _WEBSOCKETPP_CPP11_RANDOM_DEVICE_)

message(INFO "WS Include is: ${WEBSOCKETPP_INCLUDE}")
target_include_directories(baromesh-common
    PUBLIC ${Boost_INCLUDE_DIRS}
    PUBLIC websocketpp/
           include
    )
target_link_libraries(baromesh-common
    PUBLIC daemon-interface
           cxx-util
           usbcdc
           ${Boost_LIBRARIES}
    )

set_target_properties(baromesh-common PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
    MACOSX_RPATH ON
)
