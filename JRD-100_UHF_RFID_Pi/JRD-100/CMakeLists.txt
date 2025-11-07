cmake_minimum_required(VERSION 3.10)
project(JRD100_RFID)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

include_directories(${CMAKE_SOURCE_DIR}/src)

add_executable(${PROJECT_NAME}
    src/main.cpp
    src/JRD100.cpp
)
