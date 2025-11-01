#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState

SUFFIXES = [""] + [f"_{i}" for i in range(1, 9)]
JOINTS = [f"gripper_joint{s}" for s in SUFFIXES]


def main():
    rospy.init_node('gripper_joint_state_publisher')
    topic = rospy.get_param('~topic', '/arm_all/joint_states')
    rate_hz = float(rospy.get_param('~rate', 20.0))
    pos = float(rospy.get_param('~position', 0.0))

    pub = rospy.Publisher(topic, JointState, queue_size=10)
    rate = rospy.Rate(rate_hz)

    msg = JointState()
    msg.name = JOINTS
    msg.position = [pos] * len(JOINTS)
    msg.velocity = []
    msg.effort = []

    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    main()
