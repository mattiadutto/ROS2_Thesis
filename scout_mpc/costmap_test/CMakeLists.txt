cmake_minimum_required(VERSION 3.8)
project(costmap_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR})
#set(GUROBI_DIR "$ENV{GUROBI_HOME}")
set(GUROBI_LIB_DIR "/home/garabetov/Gurobi/gurobi1100/linux64/lib")


find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)

#find_package(GUROBI REQUIRED)


################################################################################
# Build
################################################################################
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  #${GUROBI_LIBRARY_DIRS}
)

include_directories(
  include
  include/costmap_test
  ${GAZEBO_INCLUDE_DIRS}
    /home/garabetov/Gurobi/gurobi1100/linux64/include
  #${GUROBI_INCLUDE_DIRS}
  ${GUROBI_LIB_DIR}/include


)

link_directories(${GUROBI_LIB_DIR})
link_libraries(gurobi_g++8.5)


#link_libraries(${GUROBI_LIBRARIES})

set(dependencies
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "sensor_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "nav2_costmap_2d"
  "nav2_util"
  "nav2_core"

  
)



set(EXEC_NAME "costmap_test")

add_executable(${EXEC_NAME} src/costmap_test.cpp
  src/MPC_diffDrive_fblin.cpp
  src/GUROBIsolver.cpp
  src/fblin_unicycle.cpp
  )

ament_target_dependencies(${EXEC_NAME} ${dependencies})



#target_link_libraries(${EXEC_NAME} ${GUROBI_LIBRARY})

target_link_libraries(${EXEC_NAME} /home/garabetov/Gurobi/gurobi1100/linux64/lib/libgurobi110.so)
#target_link_libraries(${EXEC_NAME} ${GUROBI_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS ${EXEC_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

#message("CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")
#message("Gurobi home: ${GUROBI_HOME}")





################################################################################
# Macro for ament package
################################################################################

ament_package()
