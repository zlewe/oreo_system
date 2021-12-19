# open the port authority and motor controller 
sudo chmod 777 /dev/ttyS0
sudo chmod 777 /dev/ttyS1
sudo chmod 777 /dev/ttyS2
 sudo chmod 777 /dev/ttyACM0
# gnome-terminal -x sh -c "roslaunch navigator keyboard.launch" # connect to laser and motor driver
# gnome-terminal -x sh -c "rosrun cv_camera cv_camera_node"
