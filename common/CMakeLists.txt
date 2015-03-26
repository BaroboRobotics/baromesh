cmake_minimum_required(VERSION 3.2.1)
project(baromesh-common CXX C)

set(VERSION_MAJOR 0)
set(VERSION_MINOR 2)
set(VERSION_PATCH 0)
set(VERSION ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH})

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
find_package(Boost 1.54.0 REQUIRED COMPONENTS system chrono)

set(SOURCES
    src/dongledevicepath.cpp
    src/iocore.cpp
    src/system_error.cpp
    )

add_library(baromesh-common STATIC ${SOURCES})

set_target_properties(baromesh-common
    PROPERTIES CXX_STANDARD 11
               CXX_STANDARD_REQUIRED ON
               )

target_include_directories(baromesh-common
    PUBLIC ${Boost_INCLUDE_DIRS}
           include
    )
target_link_libraries(baromesh-common
    PUBLIC daemon-interface
           ${Boost_LIBRARIES}
    )

if(WIN32)
    target_link_libraries(baromesh-common
        PRIVATE setupapi 
        )
elseif(APPLE)
    find_library(coreFoundationLib CoreFoundation)
    find_library(ioKitLib IOKit)
    target_link_libraries(baromesh-common PRIVATE ${ioKitLib} ${coreFoundationLib})
endif()

set_target_properties(baromesh-common PROPERTIES
    VERSION ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}
    SOVERSION ${VERSION_MAJOR}
    MACOSX_RPATH ON
)
