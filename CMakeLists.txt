cmake_minimum_required(VERSION 2.8)
project(g2o_slam2d)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR})

LIST(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake_modules)

FIND_PACKAGE(G2O REQUIRED)
if(G2O_FOUND)
    if(G2O_INCLUDE_DIR)
        message(STATUS "G2O include dir is ${G2O_INCLUDE_DIR}")
    else(G2O_INCLUDE_DIR)
        message(STATUS "G2O include dir not found")
    endif()
    
    if(G2O_TYPES_SLAM2D)
        message(STATUS "G2O_TYPES_SLAM2D lib is ${G2O_TYPES_SLAM2D}")
    else(G2O_TYPES_SLAM2D)
        message(STAUS "G2O_TYPES_SLAM2D Not found")
    endif()
    
    if(G2O_TYPES_SLAM3D)
        message(STATUS "G2O_TYPES_SLAM3D lib is ${G2O_TYPES_SLAM3D}")
    else(G2O_TYPES_SLAM3D)
        message(STAUS "G2O_TYPES_SLAM3D not found")
    endif()
   
    if(G2O_STUFF_LIBRARY)
        message(STATUS " G2O_STUFF_LIBS lib is ${G2O_STUFF_LIBRARY}")
    else(G2O_STUFF_LIBRARY)
        message(STAUS "G2O_STUFF_LIBS not found")
    endif()

    if(G2O_CORE_LIBRARY)
        message(STATUS "G2O_CORE_LIBS lib is ${G2O_CORE_LIBRARY}")
    else(G2O_CORE_LIBS)
        message(STAUS "G2O_CORE_LIBS not found")
    endif()


    #required for g2o/config.h 
    #It assumes the outof core built was used. 
    set(G2O_BUILD_DIR ${G2O_INCLUDE_DIR}/build)

    set(G2O_ALL_LIBS ${G2O_TYPES_SLAM2D} ${G2O_TYPES_SLAM3D} ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} ${G2O_SOLVER_CSPARSE} ${G2O_SOLVER_CSPARSE_EXTENSION} ${G2O_SOLVER_EIGEN} ${G2O_CLI_LIBRARY})
    include_directories(${G2O_INCLUDE_DIR} ${G2O_BUILD_DIR} ${EIGEN3_INCLUDE_DIR})
    MESSAGE(STATUS "Found g2o")
    #add_subdirectory(src)
else()
    MESSAGE(STATUS "Didnot find g2o") 
endif()

# Find Eigen3
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(CSparse)

#file(GLOB_RECURSE SRCS *.cpp *.h)
file(GLOB_RECURSE SRCS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp *.h)
string(REGEX REPLACE "build/[^;]+;?" "" SRCS "${SRCS}")

# Set up the top-level include directories
INCLUDE_DIRECTORIES( ${G2O_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR})

MESSAGE(STATUS "srcs ${SRCS}") 

ADD_EXECUTABLE(g2o_slam2d
   ${SRCS}
)

#SET_TARGET_PROPERTIES(g2o_slam2d PROPERTIES OUTPUT_NAME tutorial_slam2d${EXE_POSTFIX})

TARGET_LINK_LIBRARIES(g2o_slam2d ${G2O_ALL_LIBS} ${CSPARSE_LIBRARY})
