cmake_minimum_required(VERSION 2.8.12) 
project(baromesh-daemon CXX C)

set(VERSION_MAJOR 0)
set(VERSION_MINOR 1)
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
find_package(Boost 1.54.0 REQUIRED COMPONENTS system log filesystem thread)

find_package(Threads)

set(SOURCES
    src/main.cpp
    src/dongledevicepath.cpp
    src/rundongle.cpp
    )

add_executable(baromeshd ${SOURCES})
set_source_files_properties(${SOURCES}
    PROPERTIES COMPILE_FLAGS "-std=c++11")
target_include_directories(baromeshd
    PRIVATE ${Boost_INCLUDE_DIRS}
            src
    )
target_link_libraries(baromeshd
    PRIVATE baromesh-common sfp rpc cxx-util
            dongle-interface daemon-interface
            ${Boost_LIBRARIES}
            ${CMAKE_THREAD_LIBS_INIT}
   )

if(WIN32)
    target_link_libraries(baromeshd
        PRIVATE setupapi ws2_32 mswsock
        )
elseif(APPLE)
    find_library(coreFoundationLib CoreFoundation)
    find_library(ioKitLib IOKit)
    target_link_libraries(baromeshd PRIVATE ${ioKitLib} ${coreFoundationLib})
endif()


if(WIN32)
    set(dest .)
elseif(APPLE)
    set(dest Contents/MacOS)
else()
    set(dest bin)
endif()

install(TARGETS baromeshd DESTINATION ${dest})
