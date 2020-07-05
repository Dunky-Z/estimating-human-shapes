
cmake_minimum_required (VERSION 3.8)

project ("estimating-human-shapes")

#PMP
#find_package(PMP)

#Eigen
find_package(Eigen3 REQUIRED)

#SURFACE MESH
include_directories("$ENV{Surface_Mesh_DIR}/include")
include_directories("D:/Lib/pmp-library-debug/include")

#find_package(GLFW REQUIRED)

add_executable(estimating-human-shapes testpmp.cpp estimate_shape.cpp main.cpp convex_hull.cpp estimate_shape.h utils.h convex_hull.h)
#target_link_libraries(estimating-human-shapes pmp)
target_link_libraries(estimating-human-shapes Eigen3::Eigen)
target_link_libraries(estimating-human-shapes debug "D:/Lib/pmp-library-debug/lib/pmp.lib")

target_link_libraries(estimating-human-shapes optimized "D:/Lib/pmp-library-release/lib/pmp.lib")
#target_link_libraries(estimating-human-shapes debug "D:/ITabc/ITabc/BodyReconstruction/BodyReconstruction/libs/python37_d.lib")
