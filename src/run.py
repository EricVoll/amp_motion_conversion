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
import os, glob
class Animator:


    def __init__(self):

        rospy.init_node('animator', anonymous=True)

        self.in_dir = rospy.get_param('~in_dir')
        self.out_dir = rospy.get_param('~out_dir')
        self.speed = rospy.get_param('~speed', default=1)
        self.store_converted = rospy.get_param('~store_converted', default=False)

        self.files = []
        file_query = os.path.join(self.in_dir, "*.txt")
        for file in glob.glob(file_query):
            self.files.append(file)
        self.current_file_index = 0
        self.file_count = len(self.files)

        # rospy.loginfo(euler_from_quaternion([0,0,0,1]))
        # self.tf_Buffer = tf.Buffer(rospy.Duration(10))
        # self.tf_listener = tf.TransformListener(self.tf_Buffer)
        self.tf_broadcaster = tf.TransformBroadcaster()
        # self.tf_static_broadcaster = tf.StaticTransformBroadcaster()

        self.state = rospy.Publisher('joint_states', JointState, queue_size=1)
        self.load_next_json()
        
    def load_next_json(self):
        
        path = self.files[self.current_file_index]

        with open(path) as f:
            lines = f.read()

        self.obj = json.loads(lines)
        self.num_frames = len(self.obj['Frames'])
        self.obj['converted'] = []
        self.obj['file_path'] = path

        dir_name, file_name = os.path.split(path)
        self.obj['save_path'] = os.path.join(self.out_dir,  file_name)        
        self.obj['source_name'] = file_name
        self.obj['duration_in_seconds'] = map(sum, zip(*self.obj['Frames']))[0]

        rospy.loginfo("Loaded file %s with %i Frames" % (path, self.num_frames))

    def quat_to_euler(self, w,x,y,z):
        q = []
        if w < 0:
            q = [-x, -y, -z, -w]
        else:
            q = [x, y, z, w]
        (r,p,y) = euler_from_quaternion(q, axes='rxyz')
        # return [r,p,y+math.pi]
        return [-r,p,y]

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
            "chest_fakejoint_x",
            "chest_fakejoint_y",
            "chest_joint",
            "neck_fakejoint_x",
            "neck_fakejoint_y",
            "neck_joint",
            "right_hip_fakejoint_x",
            "right_hip_fakejoint_y",
            "right_hip_joint",
            "right_knee_joint",
            "right_ankle_fakejoint_x",
            "right_ankle_fakejoint_y",
            "right_ankle_joint",
            "right_shoulder_fakejoint_x",
            "right_shoulder_fakejoint_y",
            "right_shoulder_joint",
            "right_elbow_joint",
            "left_hip_fakejoint_x",
            "left_hip_fakejoint_y",
            "left_hip_joint",
            "left_knee_joint",
            "left_ankle_fakejoint_x",
            "left_ankle_fakejoint_y",
            "left_ankle_joint",
            "left_shoulder_fakejoint_x",
            "left_shoulder_fakejoint_y",
            "left_shoulder_joint",
            "left_elbow_joint"
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
        t.child_frame_id = "root"
        t.transform.translation.x = frame[1]
        t.transform.translation.y = -frame[3]
        t.transform.translation.z = frame[2]
        t.transform.rotation.w = frame[4]
        t.transform.rotation.x = frame[5]
        t.transform.rotation.y = -frame[7]
        t.transform.rotation.z = frame[6]
        return t

    def store_frame(self, frame, t, s, transform):
        pos = transform.transform.translation
        rot = transform.transform.rotation
        
        arr = [
            frame[0], 
            pos.x, 
            pos.y, 
            pos.z,
            rot.x,
            rot.y,
            rot.z,
            rot.w
        ]

        arr.extend(s.position)

        self.obj['converted'].append(arr)

    def store_current(self):
        if not self.store_converted:
            return

        self.obj['Frames'] = None
        json_string = json.dumps(self.obj)
        with open(self.obj['save_path'], "w") as f:
            f.write(json_string)
        
        rospy.loginfo("Saving to %s" % self.obj['save_path'])

    def process_frame(self, t):

        frame = self.obj['Frames'][t]
        # frame = self.get()

        s = self.get_joint_state(frame)
        tr = self.get_robot_base_pose(frame)
        self.state.publish(s)
        self.tf_broadcaster.sendTransform(tr)     

        self.store_frame(frame, t, s, tr)   

        # return how long to wait
        return frame[0]

    def spin(self):
        counter = 0
        stop = False
        while not rospy.is_shutdown():
            while not stop:
                sleep_duration = self.process_frame(counter)
                rospy.sleep(sleep_duration * self.speed)

                counter += 1

                if counter == self.num_frames:
                    self.store_current()
                    stop = True

            if self.file_count - 1 > self.current_file_index:
                self.current_file_index += 0
                self.load_next_json()
                counter = 0
                stop = False
            else:
                rospy.loginfo("Finished with all files")
                break



if __name__ == '__main__':
    commander = Animator()
    commander.spin()
