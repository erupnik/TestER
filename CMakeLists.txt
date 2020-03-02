cmake_minimum_required(VERSION 2.8)
project(TestER)

set(BUILD_PATH_BIN  ${PROJECT_SOURCE_DIR}/bin CACHE PATH "binaires install path")
set(INSTALL_DIR ${BUILD_PATH_BIN})

set(g2o_DIR "/home/er/Documents/d_development/MMVII/TestER/thirdparty/g2o")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${g2o_DIR}/cmake_modules/")

 
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})


OPTION(WITH_G2O "Build g2o" OFF)
if (WITH_G2O)
    add_subdirectory(src)
endif()

include_directories("include")
include_directories("src")

#add mvvii 
#set(MMVII_ROOT  "/etc/opt/micmac/MMVII")
#set(MMVII_DIR  "/etc/opt/micmac/MMVII/bin")
#set(MMVII_INCLUDE  "/etc/opt/micmac/MMVII/include")
#set(MMVII_SOURCE  "/etc/opt/micmac/MMVII/src")

#include_directories( ${MMVII_ROOT} )
#include_directories( ${MMVII_INCLUDE} )
#include_directories( ${MMVII_SOURCE} )

file(GLOB_RECURSE TestER_SRC
    "src/main.cpp"
    "src/phgr_basic.cpp"
    "src/TestEqCollinear.cpp"
    "src/TestJets.cpp"
    "src/BALProblemVec.cpp"
    "src/BALProblem_.cpp" 
)


 
#include_directories( ${MMVII_INCLUDE}*.h )
#include(${CMAKE_ROOT}/Modules/ExternalProject.cmake)
#ExternalProject_Add(P2007_PRJ
#    PREFIX  /etc/opt/micmac/MMVII/bin
#    BUILD_COMMAND make -f Mk-MMVII.makefile
#)
 

add_executable(${PROJECT_NAME} "src/main.cpp" "src/TestEqCollinear.cpp" "src/TestJets.cpp" "src/BALProblemVec.cpp" "src/BALProblem_.cpp" "src/Graphs/GraphBasedBA.cpp" ) 

target_link_libraries(${PROJECT_NAME} ${CERES_LIBRARIES}) #

install (TARGETS ${PROJECT_NAME} DESTINATION ${INSTALL_DIR})
