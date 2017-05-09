rock_library(path_planning
    SOURCES PathPlanning.cpp
    HEADERS PathPlanning.hpp
    DEPS_PKGCONFIG base-types)

rock_executable(PathPlanningTest PathPlanningTest.cpp
    DEPS path_planning)

