# arm_control

## to test the arm functionality
rostopic pub /scene_observation_pose geometry_msgs/Point32 -1 -- 5.0 0.0 -0.5  <br />
rostopic pub /weed_position geometry_msgs/Point32 -1 -- 0.3 -0.1 -0.3  <br />
rostopic pub /actuation_position geometry_msgs/Point32 -1 -- 0.3 -0.1 -0.3  <br />

## if successful, each of the previous command will return a std_msgs::UInt8 msg, where 0 means successful and 1 means failed
/scene_observation_status  <br />
/weed_observation_status  <br />
/weed_actuation_status  <br />
