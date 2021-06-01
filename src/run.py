#!/usr/bin/env python


## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic
import rospy
import tf2_ros as tf
from tf.transformations import *
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Quaternion, PoseStamped

import math
import json

class Animator:


    def __init__(self):

        rospy.init_node('animator', anonymous=True)
        # rospy.loginfo(euler_from_quaternion([0,0,0,1]))
        # self.tf_Buffer = tf.Buffer(rospy.Duration(10))
        # self.tf_listener = tf.TransformListener(self.tf_Buffer)
        self.tf_broadcaster = tf.TransformBroadcaster()
        # self.tf_static_broadcaster = tf.StaticTransformBroadcaster()

        self.state = rospy.Publisher('joint_states', JointState, queue_size=1)
        self.load_json()
        
    def load_json(self):
        path = "/home/eric/catkin_ws/src/amp_motion_conversion/cfg/motions/humanoid3d_spinkick.txt"
        path = "/home/eric/catkin_ws/src/amp_motion_conversion/cfg/motions/humanoid3d_cartwheel.txt"
        #path = "/home/eric/catkin_ws/src/amp_motion_conversion/cfg/motions/humanoid3d_punch.txt"
        with open(path) as f:
            lines = f.read()

        self.obj = json.loads(lines)
        self.num_frames = len(self.obj['Frames'])

        rospy.loginfo("Loaded %i Frames" % self.num_frames)

    def quat_to_euler(self, w,x,y,z):
        q = []
        if w < 0:
            q = [-x, -y, -z, -w]
        else:
            q = [x, y, z, w]
        (r,p,y) = euler_from_quaternion(q)
        return [r,p,y]

    def get(self):
        return [
            0.033,
            0,
            0,
            0,
            1, # root rotation
            0,
            0,
            0,
            1, # chest rotation
            0,
            0,
            0,
            1, # neck rotation
            0,
            0,
            0,
            1, # right hip rotation
            0,
            0,
            0,
            0, #knee rotation
            1, # right ankle rotation
            0,
            0,
            0,
            1, # right shoulder rotation
            0,
            0,
            0,
            0, # right elbow rotaiton
            1, # left hip rotation
            0,
            0,
            0,
            0, # left knee
            1, # left ankle
            0,
            0,
            0,
            1, # left shoulder
            0,
            0,
            0,
            0,
        ]

    def get_joint_state(self, frame):
        s = JointState()
        s.header.stamp = rospy.Time.now()

        chest_rotation = self.quat_to_euler(frame[8],frame[9],frame[10],frame[11])
        neck_rotation = self.quat_to_euler(frame[12],frame[13],frame[14],frame[15])
        right_hip_rotation = self.quat_to_euler(frame[16],frame[17],frame[18],frame[19])
        right_knee_rotation = frame[20]
        right_ankle_rotation = self.quat_to_euler(frame[21],frame[22],frame[23],frame[24])
        right_shoulder_rotation = self.quat_to_euler(frame[25],frame[26],frame[27],frame[28])
        right_elbow_rotation = frame[29]
        left_hip_rotation = self.quat_to_euler(frame[30],frame[31],frame[32],frame[33])
        left_knee_rotation = frame[34]
        left_ankle_rotation = self.quat_to_euler(frame[35],frame[36],frame[37],frame[38])
        left_shoulder_rotation = self.quat_to_euler(frame[39],frame[40],frame[41],frame[42])
        left_elbow_rotation = frame[43]
        
        s.name = [
            "root_chest_joint1",
            "root_chest_joint2",
            "root_chest_joint3",
            "chest_neck_joint1",
            "chest_neck_joint2",
            "chest_neck_joint3",
            "root_right_hip_joint1",
            "root_right_hip_joint2",
            "root_right_hip_joint3",
            "right_knee",
            "right_knee_right_ankle_joint1",
            "right_knee_right_ankle_joint2",
            "right_knee_right_ankle_joint3",
            "chest_right_shoulder_joint1",
            "chest_right_shoulder_joint2",
            "chest_right_shoulder_joint3",
            "right_elbow",
            "root_left_hip_joint1",
            "root_left_hip_joint2",
            "root_left_hip_joint3",
            "left_knee",
            "left_knee_left_ankle_joint1",
            "left_knee_left_ankle_joint2",
            "left_knee_left_ankle_joint3",
            "chest_left_shoulder_joint1",
            "chest_left_shoulder_joint2",
            "chest_left_shoulder_joint3",
            "left_elbow"
        ]

        s.position = [
            chest_rotation[0],
            chest_rotation[1],
            chest_rotation[2],
            neck_rotation[0],
            neck_rotation[1],
            neck_rotation[2],
            right_hip_rotation[0],
            right_hip_rotation[1],
            right_hip_rotation[2],
            right_knee_rotation,
            right_ankle_rotation[0],
            right_ankle_rotation[1],
            right_ankle_rotation[2],
            right_shoulder_rotation[0],
            right_shoulder_rotation[1],
            right_shoulder_rotation[2],
            right_elbow_rotation,
            left_hip_rotation[0],
            left_hip_rotation[1],
            left_hip_rotation[2],
            left_knee_rotation,
            left_ankle_rotation[0],
            left_ankle_rotation[1],
            left_ankle_rotation[2],
            left_shoulder_rotation[0],
            left_shoulder_rotation[1],
            left_shoulder_rotation[2],
            left_elbow_rotation
        ]

        return s

    def get_robot_base_pose(self, frame):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"
        t.transform.translation.x = frame[1]
        t.transform.translation.y = -frame[3]
        t.transform.translation.z = frame[2]
        t.transform.rotation.w = frame[4]
        t.transform.rotation.x = frame[5]
        t.transform.rotation.y = -frame[7]
        t.transform.rotation.z = frame[6]
        return t

    def process_frame(self, t):

        frame = self.obj['Frames'][t]
        # frame = self.get()

        s = self.get_joint_state(frame)
        t = self.get_robot_base_pose(frame)
        self.state.publish(s)
        self.tf_broadcaster.sendTransform(t)        

        # return how long to wait
        return frame[0]

    def spin(self):
        counter = 0
        stop = False
        while not rospy.is_shutdown() and not stop:
            sleep_duration = self.process_frame(counter)
            rospy.sleep(sleep_duration * 20)

            counter += 1

            if counter == self.num_frames:
                if self.obj['Loop'] == "wrap":
                    counter = 0
                else:
                    stop = True




if __name__ == '__main__':
    commander = Animator()
    commander.spin()
