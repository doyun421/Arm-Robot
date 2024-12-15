import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

class RobotArmController:
    def __init__(self):
        rospy.init_node('robot_arm_controller')
        self.joint_pub = rospy.Publisher('/ur5/joint_angles', Float64, queue_size=10)
        rospy.Subscriber('/ur5/joint_states', JointState, self.joint_state_callback)
    
    def joint_state_callback(self, msg):
        # Process the joint states
        pass
    
    def move_arm(self, joint_angles):
        # Command the robot arm to move to specified joint angles
        for i, angle in enumerate(joint_angles):
            self.joint_pub.publish(angle)
            rospy.sleep(0.1)

# Example usage
controller = RobotArmController()
controller.move_arm([0.1, -0.2, 0.3, 0.4, 0.5, 0.6])  # Move robot arm to specific angles
