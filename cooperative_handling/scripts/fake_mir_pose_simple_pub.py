#!/usr/bin/env python3

# publishes a fake localization on "mir_pose_simple" topic

import rospy
from geometry_msgs.msg import PoseStamped
from tf import transformations
from math import pi


class FakeMirPoseSimplePub:
    def __init__(self):
        self.mir_name = rospy.get_param("~mir_name", "mir")
        self.pose_topic = rospy.get_param("~pose_topic", "/mir_pose_simple")
        self.fake_pose = rospy.get_param("~fake_pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.pose_publisher = rospy.Publisher(self.pose_topic, PoseStamped, queue_size=10)
        rospy.sleep(1)
        self.run()

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.publish_fake_pose()
            rate.sleep()


    def publish_fake_pose(self):
        fake_pose = PoseStamped()
        fake_pose.header.stamp = rospy.Time.now()
        fake_pose.header.frame_id = "map"
        fake_pose.pose.position.x = self.fake_pose[0]
        fake_pose.pose.position.y = self.fake_pose[1]
        fake_pose.pose.position.z = self.fake_pose[2]
        print(self.fake_pose)
        q = transformations.quaternion_from_euler(self.fake_pose[3], self.fake_pose[4], self.fake_pose[5])
        fake_pose.pose.orientation.x = q[0]
        fake_pose.pose.orientation.y = q[1]
        fake_pose.pose.orientation.z = q[2]
        fake_pose.pose.orientation.w = q[3]
        self.pose_publisher.publish(fake_pose)

if __name__ == "__main__":
    rospy.init_node("fake_mir_pose_simple_pub")
    FakeMirPoseSimplePub()
    rospy.spin()
