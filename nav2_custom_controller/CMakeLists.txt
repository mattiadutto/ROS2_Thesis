cmake_minimum_required(VERSION 3.8)
project(nav2_custom_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra )
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
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(costmap_converter REQUIRED)
find_package(costmap_converter_msgs REQUIRED)



nav2_package()

include_directories(
  include
  include/nav2_custom_controller
 # /home/garabetov/Gurobi/gurobi1100/linux64/include
 ${GUROBI_LIB_DIR}/include
)

set(dependencies
  rclcpp
  geometry_msgs
  nav2_costmap_2d
  pluginlib
  nav_msgs
  nav2_util
  nav2_core
  tf2
  costmap_converter
  costmap_converter_msgs

  
)


link_directories(${GUROBI_LIB_DIR})
link_libraries(gurobi_g++8.5)



# Source files
set(SOURCES
  src/custom_controller.cpp
  src/feedback_lin.cpp
  src/MPC_diffDrive_fblin.cpp
  src/GUROBIsolver.cpp
  src/fblin_unicycle.cpp
  src/convex_hull.cpp

)

# Build the main library
add_library(nav2_custom_controller SHARED ${SOURCES})

#add_library(nav2_custom_controller SHARED
#        src/custom_controller.cpp
#        src/feedback_lin.cpp
#        src/MPC_diffDrive_fblin.cpp
#        src/GUROBIsolver.cpp
#        )

target_link_libraries(nav2_custom_controller ${GUROBI_LIB_DIR}/libgurobi110.so)

#target_link_libraries(nav2_custom_controller
 #   ${CMAKE_DL_LIBS} # This is often necessary for Gurobi
 #   /home/garabetov/Gurobi/gurobi1100/linux64/lib/libgurobi110.so
#)

# Specify the path to the Gurobi library
#set(GUROBI_LIBRARY "/home/mario/Gurobi/gurobi1100/linux64/lib/libgurobi110.so")

# Link the nav2_custom_controller library with the Gurobi library
#target_link_libraries(nav2_custom_controller INTERFACE ${GUROBI_LIBRARY})

# prevent pluginlib from using boost
target_compile_definitions(nav2_custom_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

ament_target_dependencies(nav2_custom_controller
  ${dependencies}
)




install(TARGETS nav2_custom_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(nav2_custom_controller)
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(nav2_core custom_controller_plugin.xml)

ament_package()
