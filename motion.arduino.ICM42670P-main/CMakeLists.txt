CMAKE_MINIMUM_REQUIRED(VERSION 3.2)

PROJECT(sensor)

MESSAGE(STATUS "###Start building ${PROJECT_NAME}###")

SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_FLAGS "-Wno-literal-suffix")

IF(${CMAKE_INSTALL_PREFIX} STREQUAL "/usr/local")
    SET(CMAKE_INSTALL_PREFIX package)
ENDIF()
SET(RUNTIME_DEST bin)
SET(INCLUDE_DEST include)
SET(LIBRARY_DEST lib)

IF(${CMAKE_BUILD_TYPE} STREQUAL "debug")
    ADD_COMPILE_OPTIONS(-Wall -gdwarf-2 -fstack-protector-all -g)
ELSE()
    ADD_COMPILE_OPTIONS(-O2 -Wall -fstack-protector-all)
ENDIF()

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(examples/APEX_PedometerAndTilt)
ADD_SUBDIRECTORY(examples/APEX_Pedometer)
ADD_SUBDIRECTORY(examples/APEX_WakeOnMotion)
ADD_SUBDIRECTORY(examples/APEX_Tilt)
ADD_SUBDIRECTORY(examples/Polling_I2C)
ADD_SUBDIRECTORY(examples/FIFO_Interrupt)
ADD_SUBDIRECTORY(examples/SelfTest)