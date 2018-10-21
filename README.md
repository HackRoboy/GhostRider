## Instructions

### Initial setup

 * On FPGA in `~/.bashrc` set the environment variable  `ROS_IP` to the IP address of FPGA and the varialbe `ROS_MASTER_URI` to `http://<NUC IP>:11311`


### To start driving
 * SSH to FPGA and run `./roboy_plexus_bk`
 * On your laptop listen to the ros topic `rostopic echo /roboy/middleware/MotorStatus` to monitor how the steering motors behave, make sure that both `positions` and `pwmRef` are around zero.
 * SSH to NUC and start the steering motor controller `rosrun steering motor_controller.py`
 * SSH to NUC and start the steering controller `rosrun steering controller.py`, make sure that visual Lidar is running and that we recorded the initial orientation of the bicycle. The controller will then try to keep that orientation.


### Useful commands
* `rostopic pub -1 /roboy/middleware/MotorCommand roboy_communication_middleware/MotorCommand "{id: 4, motors: [1], setPoints: [-200000]}"` to control steering motors individually. Motor with id 1 corresponds to the left motor, with id 2 to the right motor.
* `rostopic pub -1 /robike/steering std_msgs/String "left"` to slightly turn left, this will pull the left motor and release the right motor.
