cmake_minimum_required(VERSION 3.1)
project(WifiFreeRTOS)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c90")

set(SOURCE_FILES
    ESP8622.c
    ESP8622.h
    FreeRTOSConfig.h
    main.c
    Makefile
    Ultrasonic.c
    Ultrasonic.h)

cmake_minimum_required(VERSION 3.1)

project(mytest)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_custom_target(mytest COMMAND make -C ${mytest_SOURCE_DIR}
                         CLION_EXE_DIR=${PROJECT_BINARY_DIR})