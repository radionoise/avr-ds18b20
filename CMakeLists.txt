cmake_minimum_required(VERSION 3.9)
set(CMAKE_C_STANDARD 99)

set(CMAKE_C_COMPILER avr-gcc)
set(CMAKE_RANLIB avr-ranlib)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(include)
add_library(avr-ds18b20 src/ds18b20.c include/ds18b20.h)