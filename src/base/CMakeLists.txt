add_library(base-types
Eigen.hpp
Float.hpp
Pose.hpp
Pose.cpp
Time.hpp
Time.cpp
Waypoint.hpp
Waypoint.cpp
samples/Frame.hpp
samples/Frame.cpp
samples/RigidBodyState.hpp
samples/RigidBodyState.cpp)

target_link_libraries(base-types Eigen3::Eigen)

target_include_directories(base-types PRIVATE ${CMAKE_SOURCE_DIR}/src)
