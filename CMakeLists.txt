
cmake_minimum_required(VERSION 3.22)
project(sci_arma)

set(CMAKE_CXX_STANDARD 14)

set(EXECUTABLE_OUTPUT_PATH ./build)
set(LIBRARY_OUTPUT_PATH ./build)

# Find Armadillo
find_package(Armadillo REQUIRED)
include_directories(./include ${ARMADILLO_INCLUDE_DIRS})
link_directories(${ARMADILLO_LIBRARIES})

# Build Optimization Library
add_library(optim src/fmincon/Rosenbrock.cpp src/fmincon/Powell.cpp src/fmincon/Powell.cpp src/fmincon/Powell_modified.cpp
        src/fmincon/BFGS.cpp src/fmincon/fmincon.cpp)
target_link_libraries(optim ${ARMADILLO_LIBRARIES})

#Add Example Exe
add_executable(sci_arma example_without_gra.cpp)
target_link_libraries(sci_arma ${ARMADILLO_LIBRARIES} optim)