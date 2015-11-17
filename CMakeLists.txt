cmake_minimum_required(VERSION 2.8)
project(particle2d++)
set( NAME_SRC 
    bee-map.cpp
    logBook.cpp
    main.cpp    
)

INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_SOURCE_DIR}/include )
link_directories( ${CMAKE_BINARY_DIR}/bin)
set( EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR} )
add_executable( particle2d++ ${NAME_SRC} ${NAME_HEADERS} )
