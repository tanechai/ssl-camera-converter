cmake_minimum_required(VERSION 2.8)
SET(CMAKE_CXX_COMPILER "/usr/bin/clang++")
set(CMAKE_CXX_STANDARD 17)
project( Sample )
find_package( OpenCV REQUIRED )
file(GLOB SRC
    "src/**/*.cpp"
    "src/**/*.hpp"
    "src/main.cpp"
)
add_executable( ssl-camera-conberter ${SRC} )
target_link_libraries( ssl-camera-conberter ${OpenCV_LIBS} )

