cmake_minimum_required(VERSION 3.10)
project(JRD100_RFID)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Uyarıları aç
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

# Eğer Pi 0 üzerinde build ediyorsan, sistemin kütüphanelerine göre ayarlayabilirsin
# örnek: WiringPi veya Serial kütüphanesi varsa ekleyebilirsin
# find_library(WIRINGPI_LIB wiringPi)
# target_link_libraries(${PROJECT_NAME} ${WIRINGPI_LIB})

include_directories(${CMAKE_SOURCE_DIR}/src)

add_executable(${PROJECT_NAME}
    src/main.cpp
    src/JRD100.cpp
)
