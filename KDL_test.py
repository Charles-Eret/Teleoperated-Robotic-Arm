import PyKDL as kdl
from urdf_parser_py.urdf import URDF
from kdl_parser.urdf import treeFromUrdfModel

from robot import Robot
from dynamixel import Dynamixel
import math, time

def load_chain(filename, base_link, end_effector_link):
    # Load the URDF from a file
    with open(filename, "r") as urdf_file:
        urdf_string = urdf_file.read()
        
    # Load the URDF from the string
    robot = URDF.from_xml_string(urdf_string)
    ok, tree = treeFromUrdfModel(robot)

    # Ensure the URDF was successfully parsed
    if not ok:
        raise RuntimeError("Failed to parse the URDF model!")
        
    # Extract the kinematic chain from base to end-effector
    return tree.getChain(base_link, end_effector_link)

# chain for leader arm
chain1 = load_chain("low-cost-arm.urdf", "base_link", "gripper-static-motor_v2_1")

# Create a KDL joint array for leader arm
num_joints = chain1.getNrOfJoints()
joint_positions = kdl.JntArray(num_joints)

# Create the FK solver
fk_solver = kdl.ChainFkSolverPos_recursive(chain1)

# Output frame (Cartesian position)
end_effector_frame = kdl.Frame()

# Example joint angles (replace these with actual joint positions)
joint_positions[0] = 0.0 #math.pi  # Joint "Revolute 2"
joint_positions[1] = 0.0  # Joint "Revolute 3"
joint_positions[2] = 0.0 # math.pi  # Joint "Revolute 5"
joint_positions[3] = 0.0 #math.pi  # Joint "Revolute 6"
# joint_positions[4] = 4.0  # Joint "Revolute 7"

# Perform forward kinematics
fk_solver.JntToCart(joint_positions, end_effector_frame)

# Extract the position and orientation
position = end_effector_frame.p
orientation = end_effector_frame.M

print("End effector position: x={}, y={}, z={}".format(position.x(), position.y(), position.z()))

y_offset = position.y()

#------------------------------------------------------------------------------------
# Inverse kinematics portion

# chain for follower arm
chain2 = load_chain("low-cost-arm.urdf", "base_link", "gripper-static-motor_v2_1")

# Create a KDL joint array for follower arm
num_joints2 = chain2.getNrOfJoints()

# Convert position and orientation to PyKDL.Vector and PyKDL.Rotation
position_vector = kdl.Vector(position.x(), position.y(), position.z())
orientation_rotation = kdl.Rotation(orientation[0, 0], orientation[0, 1], orientation[0, 2],
                                    orientation[1, 0], orientation[1, 1], orientation[1, 2],
                                    orientation[2, 0], orientation[2, 1], orientation[2, 2])

# Set the desired end-effector frame for IK
target_frame = kdl.Frame(orientation_rotation, position_vector)

# Create an empty joint array to store the IK solution
ik_joint_positions = kdl.JntArray(num_joints2)

# Create the IK solver
ik_solver = kdl.ChainIkSolverPos_LMA(chain2)

# Perform inverse kinematics
result = ik_solver.CartToJnt(joint_positions, target_frame, ik_joint_positions)

# Check if IK was successful
if result >= 0:
    #print("IK solution found:")
    print("Joint angles:")
    for i in range(num_joints):
        print(f"Joint {i}: {ik_joint_positions[i]}")
else:
    print("Error: IK solution not found!")


# Test on the robot
# leader_dynamixel = Dynamixel.Config(baudrate=1_000_000, device_name='/dev/ttyACM1').instantiate()
# follower_dynamixel = Dynamixel.Config(baudrate=1_000_000, device_name='/dev/ttyACM0').instantiate() 
leader_dynamixel = Dynamixel.Config(baudrate=1_000_000, device_name='/dev/ttyACM1').instantiate()
follower_dynamixel = Dynamixel.Config(baudrate=1_000_000, device_name='/dev/ttyACM0').instantiate() 
follower = Robot(follower_dynamixel, servo_ids=[1, 2, 3, 4, 5])
leader = Robot(leader_dynamixel, servo_ids=[1, 2, 3, 4, 5])
leader.set_trigger_torque()

cartControl = True

try:
    while True:
        time.sleep(0.01)
        #leader.apply_haptic_feedback(follower.read_position())
        #print("Follower pos")
        #print(follower.read_position(rad=True))
        follower.set_goal_pos(leader.read_position())
        #print("Leader pos")
        follower_joint_pos = follower.read_position(rad=True)
        leader_joint_pos = leader.read_position(rad=True)
        #print(leader_joint_pos)
        for i in range(4):
            joint_positions[i] = follower_joint_pos[i]
        joint_positions[1] =  -joint_positions[1]
        joint_positions[3] =  -(joint_positions[3]-math.pi/2)
        #print(joint_positions)
        if cartControl:
            # Perform forward kinematics
            fk_solver.JntToCart(joint_positions, end_effector_frame, 4)
            # Extract the position and orientation
            position = end_effector_frame.p
            orientation = end_effector_frame.M
            print("End effector position: x={}, y={}, z={}".format(position.x(), position.y(), position.z()))

            # Convert position and orientation to PyKDL.Vector and PyKDL.Rotation
            position_vector = kdl.Vector(position.x(), position.y(), position.z())
            orientation_rotation = kdl.Rotation(orientation[0, 0], orientation[0, 1], orientation[0, 2],
                                                orientation[1, 0], orientation[1, 1], orientation[1, 2],
                                                orientation[2, 0], orientation[2, 1], orientation[2, 2])

            # Set the desired end-effector frame for IK
            target_frame = kdl.Frame(orientation_rotation, position_vector)
            # Create an empty joint array to store the IK solution
            ik_joint_positions = kdl.JntArray(num_joints2)
            # Create the IK solver
            ik_solver = kdl.ChainIkSolverPos_LMA(chain2)
            # Perform inverse kinematics
            result = ik_solver.CartToJnt(joint_positions, target_frame, ik_joint_positions)
            #print("Obtained joint positions")
            #print(joint_positions)


except KeyboardInterrupt:
    follower._disable_torque()
    leader._disable_torque()
    print("Stopping teleoperation")
