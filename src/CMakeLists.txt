rock_library(path_planning
    SOURCES PathPlanning.cpp NodeMap.cpp
    HEADERS PathPlanning.hpp NodeMap.hpp
    DEPS_PKGCONFIG base-types)

#rock_executable(PathPlanningTest PathPlanningTest.cpp
 #  DEPS path_planning)

