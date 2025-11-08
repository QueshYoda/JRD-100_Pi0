cmake_minimum_required(VERSION 3.10)
project(JRD100_Pi0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")


add_library(JRD100 STATIC
    src/JRD100.cpp
)


target_include_directories(JRD100 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)


add_executable(rfid_version_test ../examples/get_version.cpp)
target_link_libraries(rfid_version_test PRIVATE JRD100)