import rclpy
from rclpy.node import Node

from rosbot_msgs.msg import InverseMessage, ForwardPositions
import numpy as np


##  Joint Links ##
# l0 refers to the height of the arm base wrt base_link
l0 = 0.05 + 0.0654868 + 0.0338648 + 0.0772047
l1 = 0.130
l2 = 0.130
l3 = 0.055
tool_link = 0.117

lengths = [l0, l1, l2, l3+tool_link, 0]


arm_ids = [1, 2, 3, 4, 5]

# Only joint 2 is limited to protect the screen
min_angles = [-120, -90, -120, -120, -120]
max_angles = [120, 66, 120, 120, 120]
min_pulses = [0, 125, 0, 0, 0]
max_pulses = [1000, 775, 1000, 1000, 1000]


# All joints operate in the range -120 to 120 with a pulse width range of 0 to 1000

# TODO: Write helper scripts to convert from one to the other.

def angle_to_pulse(angles):
    # TODO
    pulses = [np.interp(angles[i], [min_angles[i], max_angles[i]], [min_pulses[i], max_pulses[i]]).astype(int) for i in range(5)]
    return pulses

def pulse_to_angle(pulses):
    # TODO
    angles = [np.interp(pulses[i], [min_pulses[i], max_pulses[i]], [min_angles[i], max_angles[i]]) for i in range(5)]
    return angles

class IKNode(Node):
    def __init__(self):
        super().__init__('ik_node')
        self.inverse_kinematics_sub = self.create_subscription(InverseMessage, 'inverse_kinematics_topic', self.inverse_kinematics_callback, 10)
        self.forward_pub = self.create_publisher(ForwardPositions, 'forward_positions_topic', 10)
    

    def inverse_kinematics_callback(self, msg):
        print("---------- Inverse Kinematics Callback -----------")

        roll_rad = np.radians(msg.roll)
        pitch_rad = np.radians(msg.pitch)
        # angles 1 and 5
        angles = [ 0,0,0,0, roll_rad]
        angles[0] = np.arctan2(msg.y,msg.x)

        # angles 2 and 3
        r = np.hypot(msg.x, msg.y)
        r3 = r - (l3+tool_link)*np.cos(pitch_rad)
        z3 = (msg.z-l0) - (l3+tool_link)*np.sin(pitch_rad)
        le3 = np.hypot(r3, z3)

        angles[2] = -2*np.arctan(np.sqrt( ((l1+l2)**2 - le3**2) / (le3**2 - (l1-l2)**2) ))
        angles[1] =  np.arctan2(z3, r3) - np.arctan2(l2*np.sin(angles[2]), l1 + l2*np.cos(angles[2])) - np.pi/2

        # angle 4
        angles[3] = pitch_rad - angles[1] - angles[2] - np.pi / 2

        forward_msg = ForwardPositions()
        forward_msg.id = [t for t in arm_ids]
        angles_deg = [np.degrees(t) for t in angles]
        forward_msg.position = [int(t) for t in angle_to_pulse(angles_deg)]
    

        print("-"*50)

        self.forward_pub.publish(forward_msg)
        self.get_logger().warn('message')

    # TODO: Write inverse kinematics node
    

def main():
    rclpy.init()
    ik_node = IKNode()
    rclpy.spin(ik_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
