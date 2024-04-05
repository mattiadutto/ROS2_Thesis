To run:

ros2 launch scout_gazebo_sim scout_mini_empty_world.launch.py use_rviz:=false world_name:=workshop_small.world use_sim_time:=true

ros2 launch scout_navigation nav2.launch.py use_sim_time:=true

NB! make sure that gurobi1100 folder is in Gurobi/gurobi1100 in the home directory (CMAKE will search there)
NB! add the two map files outside of src folder of the ros2 workspace 
