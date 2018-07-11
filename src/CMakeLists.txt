find_package(g2o REQUIRED)
include_directories(${g2o_SOURCE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${CERES_INCLUDE_DIRS})
link_directories(${g2o_LIBRARY_DIRS})
link_libraries(${g2o_LIBRARIES})
message("G2o-libraries " ${g2o_LIBRARIES})

add_executable(ABC "main.cpp" "TestEqCollinear.cpp" "Graphs/GraphBasedBA.cpp")
include_directories("../include")

target_link_libraries(ABC ${CERES_LIBRARIES})
install (TARGETS ABC DESTINATION ${INSTALL_DIR})
