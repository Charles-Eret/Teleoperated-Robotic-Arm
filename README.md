# Leader-Follower Robotic Arms by Tau Robotics
## Project Overview
This project was created by Charles Eret as a USRA in the AIR Labs. It uses two small 3D printed robotic arms found in the lab. The user physically moves the leader arm, and the follower arm will mirror the movement. The initial design was created by following the [Low Cost Robot](https://github.com/AlexanderKoch-Koch/low_cost_robot?tab=readme-ov-file) project by Tau Robotics. Additional features were then implemented (haptic feedback and cartesian mapping).

https://github.com/user-attachments/assets/fd4e5559-1e65-499e-9b87-531fbf1ffbae

## Usage Guide
### Leader to Follower mirror control:
teleoperate_real_robot.py runs the mirror control program using 1-to-1 servo position mapping (cartesian mapping implementation is a future step). 
<br>
Install all necessary dependencies then run teleoperate_real_robot.py. Make sure robot.py and dynamixel.py are in the same directory. This can be run on windows and linux systems. 
<br>
Configuration:
<br>
Change the "device name" port values for the leader_dynamixel and follower_dynamixel instanstiations to your device's situation. In windows this can be done by opening Task Manager and going to "ports".

### Cartesian mapping:
KDL_test.py runs the cartesian mapping functionality - it is not yet implemented into the actual control program. The function's code can only be run on linux systems (a virtual machine was used in this project), as it uses the Orocos KDL Library. Servo positions of the leader arm are mapped to the cartesian space, which can then be mapped back to servo positions of the desired "follower" robotic arm by using its urdf. The code currently uses the Tau Robotics urdf. 
<br> 
Follow the respective instructions to install the [Orocos KDL Library](https://github.com/orocos/orocos_kinematics_dynamics) and the [KDL parser](https://github.com/jvytee/kdl_parser/blob/master/README.md). Run KDL_test.py. 
<br>
Configuration:
<br>
To change the follower or leader model that is used: adjust the load_chain function calls to pass the correct urdf file, base tag, and end effector tag. 
