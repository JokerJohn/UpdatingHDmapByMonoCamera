#!/usr/bin/env python
import roslib
roslib.load_manifest('learning_tf')
import rospy
import numpy as np
import tf
import turtlesim.msg

class tf_ctr():
    def __init__(self, gps):
        self.gps = gps
        self.pub_rate = 40
        self.gps_follow()
    def gps_follow(self):
        br = tf.TransformBroadcaster()
        rate = rospy.Rate(self.pub_rate)
        count = 0
        while not rospy.is_shutdown():
            br.sendTransform((self.gps[count][0], self.gps[count][1], self.gps[count][2]),
                             tf.transformations.quaternion_from_euler(0, 0, (-self.gps[count][3])*3.14/180),
                             rospy.Time.now(), "Veihcle", "map")
            count += 1
            print("x:",self.gps[count][0],"y",self.gps[count][1],"z",self.gps[count][2],"heading",self.gps[count][3])
            rate.sleep()
        # rospy.spin()

if __name__ == '__main__':
    rospy.init_node('gps_tf_broadcaster')
    # gps, line_points, heading= np.load("./src/gps_follow/src/gps_pre_frame2.npy")
    gps = np.loadtxt("./src/gps_follow/src/GPS.txt", delimiter=",")
    tf_ctr(gps)