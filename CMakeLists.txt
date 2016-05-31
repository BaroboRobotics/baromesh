cmake_minimum_required(VERSION 3.2.1)

add_subdirectory(interfaces)

if(NOT AVR)
    add_subdirectory(common)
    add_subdirectory(library)
endif()
