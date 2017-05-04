PROJECT(path_planning)

cmake_minimum_required(VERSION 2.6)

#add definitions, compiler switches, etc.
ADD_DEFINITIONS(-Wall)

#list all source files here
#ADD_EXECUTABLE(driver main.cpp interface.cpp)
find_package(PkgConfig)

pkg_check_modules(BASE_TYPES REQUIRED "base-types")
INCLUDE_DIRECTORIES(${BASE_TYPES_INCLUDE_DIRS})
LINK_DIRECTORIES(${BASE_TYPES_LIBRARY_DIRS})

#pkg_check_modules(ENVIRE REQUIRED "envire")
#INCLUDE_DIRECTORIES(${BASE_TYPES_INCLUDE_DIRS})
#LINK_DIRECTORIES(${BASE_TYPES_LIBRARY_DIRS})

INCLUDE_DIRECTORIES(/usr/include/eigen2)

ADD_LIBRARY(path_planning SHARED PathPlanning.cpp)
#TARGET_LINK_LIBRARIES(xsens xsensVendorLib) 
#${DFKI_TYPES_LIBS})

#need to link to some other libraries ? just add them here
#TARGET_LINK_LIBRARIES(driver png jpeg)

ADD_EXECUTABLE(PathPlanningTest PathPlanningTest.cpp)
TARGET_LINK_LIBRARIES(PathPlanningTest path_planning)
 
CONFIGURE_FILE(Doxyfile.in Doxyfile @ONLY)
ADD_CUSTOM_TARGET(doc doxygen Doxyfile)
INSTALL(TARGETS path_planning
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib)
INSTALL(FILES PathPlanning.hpp DESTINATION include)

CONFIGURE_FILE(path_planning.pc.in path_planning.pc @ONLY)
INSTALL(FILES ${CMAKE_BINARY_DIR}/path_planning.pc DESTINATION lib/pkgconfig)
