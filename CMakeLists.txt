cmake_minimum_required(VERSION 2.8.3)
project(neo_local_planner)

set(CMAKE_CXX_STANDARD 11)

## By default build with code optimization and no debug info
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release"
  	  CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
endif(NOT CMAKE_BUILD_TYPE)

find_package(catkin REQUIRED
        COMPONENTS
            base_local_planner
            cmake_modules
            costmap_2d
            dynamic_reconfigure
            nav_core
            nav_msgs
            pluginlib
            roscpp
            tf
        )

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

## dynamic reconfigure
#generate_dynamic_reconfigure_options(
#    cfg/NeoPlanner.cfg
#)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES neo_local_planner
    CATKIN_DEPENDS
        dynamic_reconfigure
        nav_msgs
        pluginlib
        roscpp
)

add_library(neo_local_planner src/NeoLocalPlanner.cpp)
add_dependencies(neo_local_planner ${catkin_EXPORTED_TARGETS})
target_link_libraries(neo_local_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_LIBRARIES})

install(TARGETS neo_local_planner
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(FILES blp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
