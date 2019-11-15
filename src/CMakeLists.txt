add_subdirectory(base)
add_library(DyMu_lib
            DyMu.hpp
            DyMu_GlobalPathPlanning.cpp
            DyMu_LocalPathRepairing.cpp)

target_link_libraries(DyMu_lib Eigen3::Eigen)
target_include_directories(DyMu_lib PRIVATE ${CMAKE_SOURCE_DIR}/src)
