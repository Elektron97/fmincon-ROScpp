cmake_minimum_required(VERSION 3.0.2)
project(ros_fmincon)

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
)
find_package(Armadillo REQUIRED)

catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES ${PROJECT_NAME}
  #CATKIN_DEPENDS roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ARMADILLO_INCLUDE_DIRS}
)

#### fmincon Library ####
# Build Optimization Library
add_library(${PROJECT_NAME} src/Rosenbrock.cpp src/Powell.cpp src/Powell.cpp src/Powell_modified.cpp
        src/BFGS.cpp src/fmincon.cpp)
target_link_libraries(${PROJECT_NAME} ${ARMADILLO_LIBRARIES} ${catkin_LIBRARIES})

#### Export Library ####
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)