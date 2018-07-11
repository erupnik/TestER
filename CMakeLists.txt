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


OPTION(WITH_G2O "Build g2o" ON)
if (WITH_G2O)
    add_subdirectory(src)
endif()

include_directories("include")
include_directories("src")

file(GLOB_RECURSE TestER_SRC
    "src/main.cpp"
    "src/TestEqCollinear.cpp"
)

add_executable(${PROJECT_NAME} "src/main.cpp" "src/TestEqCollinear.cpp" "src/Graphs/GraphBasedBA.cpp")
#add_executable(${PROJECT_NAME} "src/main.cpp" "src/TestEqCollinear.cpp")

target_link_libraries(${PROJECT_NAME} ${CERES_LIBRARIES})

install (TARGETS ${PROJECT_NAME} DESTINATION ${INSTALL_DIR})
