from robot import Robot
from dynamixel import Dynamixel

leader_dynamixel = Dynamixel.Config(baudrate=1_000_000, device_name='/dev/ttyACM1').instantiate()
follower_dynamixel = Dynamixel.Config(baudrate=1_000_000, device_name='/dev/ttyACM0').instantiate()
follower = Robot(follower_dynamixel, servo_ids=[1, 2, 3, 4, 5])
leader = Robot(leader_dynamixel, servo_ids=[1, 2, 3, 4, 5])
leader.set_trigger_torque()

try:
    while True:
        leader.apply_haptic_feedback(follower.read_position())
        follower.set_goal_pos(leader.read_position())
except KeyboardInterrupt:
    follower._disable_torque()
    leader._disable_torque()
    print("Stopping teleoperation")
    