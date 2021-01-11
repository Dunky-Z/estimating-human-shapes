file(GLOB header *.h)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -fopenmp")

include_directories(
$ENV{HOME}/usr/include/eigen
)

#add_executable(geodesic ${header})
