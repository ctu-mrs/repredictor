cmake_minimum_required(VERSION 3.1.2)
project(repredictor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall)
add_definitions(-Wextra)
add_definitions(-O3)
add_definitions(-g)

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${EIGEN3_LIBRARIES})

# include threads
find_package(Threads REQUIRED)

###########
## Build ##
###########

include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  )

add_executable(repredictor_example src/example.cpp)
target_link_libraries(repredictor_example
  ${Eigen_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
  )
