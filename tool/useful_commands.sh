 # change image quality of default image compression from camera ros driver
rosrun dynamic_reconfigure dynparam get /camera/color/image_raw/compressed
rosrun dynamic_reconfigure dynparam set /camera/color/image_raw/compressed jpeg_quality 1
roslaunch mirte_bringup minimal_master.launch

# synchronize the time to make sure no delay in timestamps
ssh mirte@192.168.10.176 "sudo date $(date -u +"%m%d%H%M%Y.%S")"
ssh mirte@192.168.234.98 sudo date --set @$(date -u +%s)
sudo timedatectl set-timezone UTC
sudo timedatectl set-ntp true # with internet

# set ros1 master running onboard
sudo service mirte-ros stop
export ROS_IP=192.168.31.83 # every terminal of mirte that start any rosnode or roslaunch

# set ros1 pc terminal variables
export ROS_IP=192.168.31.221
export ROS_MASTER_URI=http://192.168.31.83:11311

# amcl init
rosservice call /global_localization "{}"

# save map
rosrun map_server map_saver -f /home/xaviergg/Desktop/RO_MDP/group13/src/state-estimation/amcl/map/new_map

# kill process that use the port
sudo lsof -ti :11311 | xargs -r sudo kill


ssh mirte@192.168.10.176

# install ros dependencies
rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro noetic -y

ssh mirte@192.168.234.98


export ROS_IP=192.168.234.98


export ROS_IP=192.168.234.210
export ROS_MASTER_URI=http://192.168.234.98:11311

export ROS_MASTER_URI=http://192.168.42.1:11311
export ROS_IP=192.168.42.94

export ROS_IP=192.168.42.1

# save from ssh remote
scp mirte@192.168.42.1:/home/mirte/onboard_tools_ws/2024-06-11-19-33-05.bag /home/xaviergg/Desktop/RO_MDP/rosbag



