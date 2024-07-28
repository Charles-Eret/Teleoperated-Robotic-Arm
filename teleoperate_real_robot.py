from robot import Robot
from dynamixel import Dynamixel

leader_dynamixel = Dynamixel.Config(baudrate=1_000_000, device_name='COM6').instantiate()
follower_dynamixel = Dynamixel.Config(baudrate=1_000_000, device_name='COM5').instantiate()
follower = Robot(follower_dynamixel, servo_ids=[1, 2, 3, 4, 5])
leader = Robot(leader_dynamixel, servo_ids=[1, 2, 3, 4, 5])
#leader.limit_velocity(10)
#leader.print_velocity_limit()
leader.set_trigger_torque()

try:
    while True:
        leader.apply_haptic_feedback(follower.read_position())
        follower.set_goal_pos(leader.read_position())
except KeyboardInterrupt:
    follower._disable_torque()
    leader._disable_torque()
    print("Stopping teleoperation")
    