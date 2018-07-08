cmake_minimum_required(VERSION 2.8)
project(TestER)

set(g2o_DIR "/home/er/Documents/d_development/MMVII/TestER/thirdparty/g2o")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${g2o_DIR}/cmake_modules/")


find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

find_package(g2o REQUIRED)
include_directories(${g2o_SOURCE_DIR})
link_directories(${g2o_LIBRARY_DIRS})
link_libraries(${g2o_LIBRARIES})
message("G2o-libraries " ${g2o_LIBRARIES})

include_directories("include")
include_directories("src")

file(GLOB_RECURSE TestER_SRC
    "src/main.cpp"
    "src/TestEqCollinear.cpp"
    "src/GraphBasedBA.cpp"
)

add_executable(TestER "src/main.cpp" "src/TestEqCollinear.cpp" "src/GraphBasedBA.cpp")

target_link_libraries(${PROJECT_NAME} ${CERES_LIBRARIES})

install (TARGETS ${PROJECT_NAME} DESTINATION /home/er/Documents/d_development/MMVII/TestER/bin)
