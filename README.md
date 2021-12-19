# OREO

This is the repo for OREO bot @ NCTUACL.

## Instructions
To start using oreo:

1. $ source environment.sh
2. $ source start_oreo.sh
   
   Use procman to run the nodes you needed: i.e. joystick control, rviz

   You can use the GUI by right clicking on the entries, or simply use CTRL-S to start and CTRL-T to stop.

3. To record rosbag, include topics you want to record in record_bag.sh.
4. Start recording by running (CTRL-S) the record entry in procman GUI. Stop recording by simply stopping the process on procman. (CTRL-T)

5. To stop using the bot, simply stop all the processes on procman. (CTRL-A then CTRL-T).