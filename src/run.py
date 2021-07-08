#!/usr/bin/env python


## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic
import math
import json
import argparse
import os, glob
import numpy as np
class Animator:


    def __init__(self):

        parser = argparse.ArgumentParser()
        parser.add_argument('--in_dir', type=str, default='../cfg/motions/')
        parser.add_argument('--out_dir', type=str, default='../cfg/motions/converted')
        args = parser.parse_args()

        self.in_dir = args.in_dir
        self.out_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)), args.out_dir)
        
        self.store_converted = True

        self.files = []
        file_query = os.path.join(os.path.dirname(os.path.realpath(__file__)), self.in_dir, "*.txt")
        for file in glob.glob(file_query):
            self.files.append(file)
        self.current_file_index = 0
        self.file_count = len(self.files)

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
        self.obj['duration_in_seconds'] = self.obj['Frames'][0][0] * len(self.obj['Frames'])

        print("Loaded file %s with %i Frames" % (path, self.num_frames))

        self.current_file_index += 1

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

    

    def get_robot_base_pose(self, frame):
        out_frame = np.empty(7)
        out_frame[0] = frame[1]
        out_frame[1] = -frame[3]
        out_frame[2] = frame[2]
        out_frame[3] = frame[4]
        out_frame[4] = frame[5]
        out_frame[5] = -frame[7]
        out_frame[6] = frame[6]
        return out_frame

    def store_frame(self, frame, body_pose):
        

        arr = [
            frame[0], 
            body_pose[0], 
            body_pose[1], 
            body_pose[2],
            body_pose[3],
            body_pose[4],
            body_pose[5],
            body_pose[6]
        ]

        arr.extend(list(frame[8:None]))

        self.obj['converted'].append(arr)

    def store_current(self):
        if not self.store_converted:
            return

        self.obj['Frames'] = None
        json_string = json.dumps(self.obj)
        with open(self.obj['save_path'], "w") as f:
            f.write(json_string)
        
        #print("Saving to %s" % self.obj['save_path'])

    def process_frame(self, t):

        frame = self.obj['Frames'][t]
        # frame = self.get()

        base_pose = self.get_robot_base_pose(frame)   

        self.store_frame(frame, base_pose)   

        # return how long to wait
        return frame[0]

    def spin(self):
        counter = 0
        stop = False
        done = False
        while not done:
            while not stop:
                sleep_duration = self.process_frame(counter)

                counter += 1

                if counter == self.num_frames:
                    self.store_current()
                    stop = True

            if self.file_count> self.current_file_index:
                self.load_next_json()
                counter = 0
                stop = False
            else:
                print("Finished with all files")
                done = True
            



if __name__ == '__main__':
    commander = Animator()
    commander.spin()
