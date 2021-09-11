#! /usr/bin/env python
import numpy as np
import pandas as pd
import rospy
import os
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from reachable_occupancy.msg import time





## @file skeleton_publisher.py This file containes the csv-parser of our system. The parsed data is packed into a message of type JointState and publsihed onto sk_points;
# while the current tbreak is published onto 'tbreak'. The csv-file has to have the following form: The first column reprsents the current time in the recording in [s].
# The other colums represent the joint positions at the respective time shown in the first column. The remaining columns have the following order and naming:
# ['RShoulder.(x,y,z)', 'RForearm.(x,y,z)', 'RHand.(x,y,z)', 'LShoulder.(x,y,z)', 'LForearm.(x,y,z)', 'LHand.(x,y,z)', 'Neck.(x,y,z)', 'Hip.(x,y,z)', 'RThigh.(x,y,z)', 'RShin.(x,y,z)', 'RFoot.(x,y,z)', 'LThigh.(x,y,z)', 'LShin.(x,y,z)', 'LFoot.(x,y,z)', 'Head.(x,y,z)'].
# Have a look at the test files in the tests-folder for examples.

## This is the main loop needed for parsing and publishing of joint-information
def main():
    # initializing-node
    rospy.init_node('skeleton_publisher', anonymous=True)
    if rospy.has_param('frames'):
        frames = rospy.get_param('frames')
    else:
        frames = 60
    rate = rospy.Rate(frames)
    pub = rospy.Publisher('sk_points', JointState, queue_size=10)
    time_pub = rospy.Publisher('tbreak', Float32, queue_size=10)
    time_set_pub = rospy.Publisher('delta_t', time, queue_size=10)

    # reach_ri mode

    interval = True
    
    delta_t = time()
    delta_t.t_a = 0.0
    delta_t.t_b = 0.2
    delta_stop = time()
    delta_stop.t_a = -1.0
    delta_stop.t_b = -1.0

    # parameters are set using rosparam and defined in the config-yaml-file; They are passed to the ros-param-server thruogh the launch-file
    if rospy.has_param('cmu'):
        cmu = rospy.get_param('cmu')
    else:
        cmu = True
    if cmu:
        if rospy.has_param('body-cmu'):
            joints = JointState()
            joints.name = rospy.get_param('body-cmu')
        else:
            upper = ['RightShoulder', 'RightElbow', 'RightWrist', 'LeftShoulder', 'LeftElbow', 'LeftWrist', 'Neck', 'Hips']
            legs = ['RightHip', 'RightKnee', 'RightAnkle', 'LeftHip', 'LeftKnee', 'LeftAnkle']
            body =  upper+legs+['Head']
            joints = JointState()
            joints.name = body
    else:
        if rospy.has_param('body'):
            joints = JointState()
            joints.name = rospy.get_param('body')
        else:
            upper = ['RShoulder', 'RForearm', 'RHand', 'LShoulder', 'LForearm', 'LHand', 'Neck', 'Hip']
            legs = ['RThigh', 'RShin', 'RFoot', 'LThigh', 'LShin', 'LFoot']
            body = upper+legs+['Head']
            joints = JointState()
            joints.name = body

    if rospy.has_param('min_vel_mode'):
        min_vel_mode = rospy.get_param('min_vel_mode')
    else:
        min_vel_mode = True
    if rospy.has_param('simulate_tbreak'):
        simulate_tbreak = rospy.get_param('simulate_tbreak')
    else:
        simulate_tbreak = True
    if rospy.has_param('max_speed'):
        max_speed = rospy.get_param('max_speed')
    else:
        max_speed = 14.0
    if rospy.has_param('csv_path'):
        csv_path = rospy.get_param('csv_path')
    else:
        cwd = os.getcwd()
        if rospy.has_param('file_name'):
            file_name = rospy.get_param('file_name')
        else:
            file_name = 'simple_skeleton_mvmt'
        csv_path = cwd + '/src/reachable_occupancy/src/tests/' + file_name + '.csv'
        print('Path of current csv-file: ' + '\n'+ csv_path)
    if rospy.has_param('def_delta_t'):
        tbreak = rospy.get_param('def_delta_t')
    else:
        tbreak = 0.02
    if rospy.has_param('size_factor'):
        fact = rospy.get_param('size_fact')
    else:
        if cmu:
            fact = 0.07
        else:
            fact = 1.0/90.0

    # initializing local-variables
    print(csv_path)

    # read the chosen csv-file
    points_csv = pd.read_csv(csv_path)
    # identify the frame-time set in the parsed scv-file
    t_step = points_csv.loc[1][0]
    # get point names
    d = points_csv.loc[0]
    t = rospy.Time.now()
    # initialize the joint-velocities
    joints.velocity = []
    # initialize dummy for current position
    new_pos = []
    # set first frame to zero
    frame = 0
    if rospy.has_param('frame_stop'):
        frame_stop = rospy.get_param('frame_stop')
    else:
        frame_stop = len(points_csv)-1
    
    # setting initial-joint-positions
    for e in joints.name:
        if cmu:
            joints.position.append(fact*d[e+'.X'])
            joints.position.append(fact*d[e+'.Z'])
            joints.position.append(fact*d[e+'.Y'])
        else:
            joints.position.append(fact*d[e+'.x'])
            joints.position.append(fact*d[e+'.z'])
            joints.position.append(fact*d[e+'.y'])
    curr = 0
    
    # Number of times the last frame is repeated before terminal '-1' is sent over the tbreak-topic
    # This is done so that both nodes have time to synchronize their frames before starting volume calculation and giving us consisten results.
    counter = 300
    while not rospy.is_shutdown():
        # As long as there are still frames to publish; tbreak is published; else '-1' is sent
        if simulate_tbreak == True:
            if interval == True:
                if not (curr == frame_stop):
                    time_set_pub.publish(delta_t)
                else:
                    if counter <= 0:
                        time_set_pub.publish(-1.0)
                    else:
                        time_set_pub.publish(delta_stop)
            else:
                if not (curr == frame_stop):
                    time_pub.publish(tbreak)
                else:
                    if counter <= 0:
                        time_pub.publish(-1.0)
                    else:
                        time_pub.publish(tbreak)
                        counter -= 1
        if curr > len(points_csv)-1:
            curr = len(points_csv)-1

        data = points_csv.loc[curr]
        seconds = data[0]
        # print(seconds)

        # Updating joint positions and velocities
        if not (curr == frame_stop):
            new_pos = []
            for e in joints.name:
                if cmu:
                    new_pos.append(fact*data[e+'.X'])
                    new_pos.append(fact*data[e+'.Z'])
                    new_pos.append(fact*data[e+'.Y'])
                else:
                    new_pos.append(fact*data[e+'.x'])
                    new_pos.append(fact*data[e+'.z'])
                    new_pos.append(fact*data[e+'.y'])
            assert(len(joints.position) == len(new_pos))
            
            joints.velocity = []
            # compare the position at time t-1 (joints.position) and t (new_pos) using the frametime in velocity calculation
            for i in range(len(new_pos)):
                if min_vel_mode:
                    joints.velocity.append(min(max_speed,(new_pos[i]-joints.position[i])/(1.0*t_step)))
                else:
                    joints.velocity.append((new_pos[i]-joints.position[i])/(1.0*t_step))
            assert(len(joints.velocity) == len(joints.position))
            joints.position = new_pos
            curr += 1

        # print rate.remaining()
        # Publishing positions and velocities as one JointState message
        pub.publish(joints)
        rate.sleep()






if __name__ == '__main__':
    main()
