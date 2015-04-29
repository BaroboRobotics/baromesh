cmake_minimum_required(VERSION 3.2.1)
project(baromesh-library CXX C)

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

# Give subprojects access to LinkbotLabs.cmake
list(INSERT CMAKE_MODULE_PATH 0 "${PROJECT_SOURCE_DIR}/../../../cmake-modules")

include(LinkbotLabs)
ll_add_dependency(cxx-util cxx-util)
ll_add_dependency(libsfp sfp)
ll_add_dependency(ribbon-bridge rpc)
ll_add_dependency(baromesh/common baromesh-common)
ll_add_dependency(baromesh/interfaces daemon-interface)

# Linking statically to Boost is easier for prototyping. For multi-module
# projects we'll want to use shared libraries and defined -DBOOST_ALL_DYN_LINK
# or the like.
set(Boost_USE_STATIC_LIBS ON)
find_package(Boost 1.54.0 REQUIRED COMPONENTS system log_setup log filesystem thread)

find_package(Threads)

set(SOURCES
    src/linkbot.cpp
    src/linkbot.c.cpp
    )

add_library(baromesh ${SOURCES})

if(CMAKE_CL_64)
    # 64-bit VS compiles need /bigobj
    # specific template instantiation might fix this
    set_source_files_properties(src/linkbot.cpp
        PROPERTIES COMPILE_FLAGS "/bigobj")
endif()

set_target_properties(baromesh
    PROPERTIES CXX_STANDARD 11
               CXX_STANDARD_REQUIRED ON
               )
target_include_directories(baromesh
    PUBLIC ${Boost_INCLUDE_DIRS}
           include
    )
target_link_libraries(baromesh
    PUBLIC baromesh-common sfp rpc cxx-util
           robot-interface daemon-interface
           Threads::Threads
           ${Boost_LIBRARIES}
    )

if(WIN32)
    target_link_libraries(baromesh PUBLIC ws2_32 mswsock)
endif()

set_target_properties(baromesh PROPERTIES
    VERSION ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}
    SOVERSION ${VERSION_MAJOR}
    MACOSX_RPATH ON
)

option(BAROMESH_BUILD_TESTS "Build baromesh tests" OFF)
if(BAROMESH_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
