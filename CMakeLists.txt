cmake_minimum_required(VERSION 2.8.12) 
project(baromesh CXX)

if(NOT DEFINED DEPS_ROOT)
    if(DEFINED ENV{DEPS_ROOT})
        set(DEPS_ROOT "$ENV{DEPS_ROOT}")
    else()
        message(FATAL_ERROR "Please set DEPS_ROOT to the location of your dependencies (libsfp, ribbon-bridge, etc.).")
    endif()
endif()

function(add_dependency _name)
    add_subdirectory(${DEPS_ROOT}/${_name} ${_name})
endfunction()

add_dependency(libsfp)
add_dependency(ribbon-bridge)
add_dependency(ribbon-bridge-interfaces)
add_dependency(serial)

set(SOURCES
    src/main.cpp)

add_executable(baromesh ${SOURCES})

set_source_files_properties(${SOURCES}
    PROPERTIES COMPILE_FLAGS "-std=c++11")

add_definitions(-DHAVE_STDLIB -DHAVE_EXCEPTIONS -DHAVE_RTTI)
target_include_directories(baromesh PRIVATE "${LIBSFP_SOURCE_DIR}/include" 
                                  PRIVATE "$ENV{BOOST_ROOT}"
                                  PRIVATE $<TARGET_PROPERTY:robot-interface,INCLUDE_DIRECTORIES>
                                  PRIVATE "${serial_SOURCE_DIR}/include")
target_link_libraries(baromesh sfp rpc serial robot-interface)
