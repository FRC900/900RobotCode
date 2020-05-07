cmake_minimum_required(VERSION 2.8)
set(CMAKE_LEGACY_CYGWIN_WIN32 0) # Remove when CMake >= 2.8.4 is required
project( field_map )
set(CMAKE_BUILD_TYPE Release)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wall -Wextra")

find_package( OpenCV REQUIRED )
list(FILTER OpenCV_LIBS EXCLUDE REGEX "opencv_freetype")
find_package( Boost COMPONENTS filesystem system thread program_options serialization iostreams REQUIRED )

include_directories( ${OpenCV_INCLUDE})
include_directories( ${Boost_INCLUDE_DIR} )

link_directories(/home/ubuntu/opencv-2.4.13/build/lib)
add_executable( main main.cpp )
target_link_libraries( main ${OpenCV_LIBS} )

add_executable( 2020field 2020field.cpp )
target_link_libraries( 2020field ${OpenCV_LIBS} )
