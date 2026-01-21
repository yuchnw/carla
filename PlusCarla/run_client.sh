source devel/setup.bash

cmd_output=$(rosnode list 2>&1)
echo ${cmd_output}
if [[ $cmd_output == *"ERROR"* ]]; then
  echo "Start ROS first..."
  roscore &
  sleep 5
  echo "ROS started..."
fi
export ROS_IP=$1