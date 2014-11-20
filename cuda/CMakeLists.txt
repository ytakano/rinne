cmake_minimum_required (VERSION 2.8.12)
project (rinne)

find_package(CUDA REQUIRED)
find_package(Boost COMPONENTS system graph REQUIRED)

set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

IF(NOT DEFINED CCBIN)
  set(CCBIN g++)
ENDIF()

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -ccbin ${CCBIN} -arch=sm_20)


IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  set(CMAKE_CXX_FLAGS "-Wall -framework GLUT -framework OpenGL -Wno-deprecated-declarations")
  set(LIBS glui)
ELSE(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  set(CMAKE_CXX_FLAGS "-Wall")
  set(LIBS glut GL GLU)
ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")


set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(LIBS ${LIBS} ${Boost_SYSTEM_LIBRARY} ${Boost_GRAPH_LIBRARY} pthread)

cuda_add_executable(rinne rinne.cu)
target_link_libraries(rinne ${LIBS})
