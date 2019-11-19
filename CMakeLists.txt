# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
cmake_minimum_required(VERSION 2.6)

project(DyMu_project)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_subdirectory(src/base)
add_subdirectory(src)
include_directories(tests/data/maps)
#add_subdirectory(tests)

add_executable(runGPPtest
               tests/globalPathPlanningTest.cpp)


target_include_directories(runGPPtest PRIVATE src)

target_link_libraries(runGPPtest DyMu_lib)

