cmake_minimum_required(VERSION 3.8)
project(mpc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
#set(GUROBI_LIB_DIR "/home/mario/Gurobi/gurobi1100/linux64/lib")
set(GUROBI_LIB_DIR "$ENV{HOME}/Gurobi/gurobi1100/linux64/lib")



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)




include_directories(
  include
  include/mpc
 # /home/garabetov/Gurobi/gurobi1100/linux64/include
 ${GUROBI_LIB_DIR}/include
)

set(dependencies
  rclcpp
  geometry_msgs
  nav2_costmap_2d
  nav_msgs
  nav2_util
  nav2_core
  tf2



  
)


link_directories(${GUROBI_LIB_DIR})
link_libraries(gurobi_g++8.5)


# Source files
set(SOURCES
  src/mpc.cpp
  src/MPC_diffDrive_fblin.cpp
  src/GUROBIsolver.cpp
  src/fblin_unicycle.cpp
  src/feedback_lin.cpp
)

# Build the main library
add_library(mpc SHARED ${SOURCES})
target_link_libraries(mpc ${GUROBI_LIB_DIR}/libgurobi110.so)



# Build the executable
add_executable(my_mpc_node src/mpc.cpp)
target_link_libraries(my_mpc_node mpc)
ament_target_dependencies(my_mpc_node ${dependencies})



ament_target_dependencies(mpc
  ${dependencies}
)
install(TARGETS
  my_mpc_node
  DESTINATION lib/${PROJECT_NAME})







if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(mpc)
ament_export_dependencies(${dependencies})

ament_package()
