
cmake_minimum_required (VERSION 3.8)

project ("estimating-human-shapes")

#PMP
#find_package(PMP)

#Eigen
find_package(Eigen3 REQUIRED)

include_directories("D:/ITabc/ITabc/BodyReconstruction/BodyReconstruction/include")
LINK_DIRECTORIES("D:/ITabc/ITabc/BodyReconstruction/BodyReconstruction/Lib")

#SURFACE MESH
include_directories("$ENV{Surface_Mesh_DIR}/include")
include_directories("D:/Lib/pmp-library-debug/include")

#find_package(GLFW REQUIRED)

add_executable(estimating-human-shapes testpmp.cpp estimate_shape.cpp main.cpp  measure.cpp reshaper.cpp  mesh_io.cpp utils.cpp measure.h estimate_shape.h utils.h binary_io.h reshaper.h mesh_io.h utils.h 
../alglib/cpp/src/ap.h ../alglib/cpp/src/linalg.h ../alglib/cpp/src/alglibinternal.h ../alglib/cpp/src/alglibmisc.h testalglib.cpp ../alglib/cpp/src/stdafx.h ../alglib/cpp/src/linalg.cpp
../alglib/cpp/src/ap.cpp ../alglib/cpp/src/alglibmisc.cpp  ../alglib/cpp/src/alglibinternal.cpp
../alglib/cpp/src/solvers.cpp ../alglib/cpp/src/solvers.h ../alglib/cpp/src/optimization.cpp ../alglib/cpp/src/optimization.h fit_measurements.cpp fit_measurements.h)
#target_link_libraries(estimating-human-shapes pmp)
target_link_libraries(estimating-human-shapes Eigen3::Eigen)
target_link_libraries(estimating-human-shapes debug "D:/Lib/pmp-library-debug/lib/pmp.lib")
target_link_libraries(estimating-human-shapes debug "D:/ITabc/ITabc/BodyReconstruction/BodyReconstruction/libs/python37_d.lib")
target_link_libraries(estimating-human-shapes optimized "D:/ITabc/ITabc/BodyReconstruction/BodyReconstruction/libs/python37.lib")

target_link_libraries(estimating-human-shapes optimized "D:/Lib/pmp-library-release/lib/pmp.lib")
#target_link_libraries(estimating-human-shapes debug "D:/ITabc/ITabc/BodyReconstruction/BodyReconstruction/libs/python37_d.lib")
