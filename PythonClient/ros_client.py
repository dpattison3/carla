#!/usr/bin/env python3

# Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Basic CARLA client example."""

from __future__ import print_function

import argparse
import logging
import random
import time

from math import cos,sin,pi

from carla import image_converter
from carla.client import make_carla_client
from carla.sensor import Camera, Lidar
from carla.settings import CarlaSettings
from carla.tcp import TCPConnectionError
from carla.util import print_over_same_line

import rospy

import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

from autorally_msgs.msg import chassisCommand
from nav_msgs.msg import Odometry
import tf as ros_tf

from pprint import pprint

pose_publisher = None

class temp:
    joystick_command = chassisCommand()

def run_carla_client():
    # start the client and open the port
    with make_carla_client('localhost', 2000) as client:
        print('CarlaClient connected')

        settings = CarlaSettings()
        settings.set(
                SynchronousMode=False,
                SendNonPlayerAgentsInfo=True,
                NumberOfVehicles=0,
                NumberOfPedestrians=0,
                QualityLevel='Epic',
                WeatherId=random.choice([1]))
        settings.randomize_seeds()

        # set up a camera
        camera = Camera('CameraRGB')
        camera.set_image_size(640,512)
        camera.set_position(0.60,0,1.60) # position of the camera relative to the car in cm
        settings.add_sensor(camera)

        # now load these settings into the server
        scene = client.load_settings(settings)
        # Choose one player start at random.
        number_of_player_starts = len(scene.player_start_spots)
        player_start = random.randint(0, max(0, number_of_player_starts - 1))

        # Notify the server that we want to start the episode at the
        # player_start index. This function blocks until the server is ready
        # to start the episode.
        client.start_episode(player_start)

        prev_time = time.time()
        images = 0
        prev_measure_time = None
        # looping for each frame from server
        while True:
            measurements, sensor_data = client.read_data()
            
            im = sensor_data.get('CameraRGB', None)
            if im is not None:
                # pprint(dir(im))
                # pprint(sensor_data.viewvalues())
                # pprint(sensor_data.viewitems())
                np_image = image_converter.to_rgb_array(im)
                image_message = temp.bridge.cv2_to_imgmsg(np_image, encoding="rgb8")
                temp.image_pub.publish(image_message)
                images += 1
                t = time.time()
                if ((prev_time + 1) < t):
                    print('FPS: %d\tSteer: %1.3f Throttle %1.3f, Brake %1.3f' % (images,temp.joystick_command.steering,max(temp.joystick_command.throttle,0),-min(temp.joystick_command.throttle,0)))
                    prev_time += 1
                    images = 0
            
            if prev_measure_time is None:
                prev_measure_time = measurements.game_timestamp
            
            # print(prev_measure_time - measurements.game_timestamp)
            prev_measure_time = measurements.game_timestamp

            measurements = measurements.player_measurements
            pose_msg = Odometry()
            pose_msg.pose.pose.position.x = measurements.transform.location.x
            pose_msg.pose.pose.position.y = measurements.transform.location.y
            pose_msg.pose.pose.position.z = measurements.transform.location.z
            
            quaternion = ros_tf.transformations.quaternion_from_euler(
                    measurements.transform.rotation.roll*pi/180,
                    measurements.transform.rotation.pitch*pi/180,
                    measurements.transform.rotation.yaw*pi/180)
            # print(measurements.transform.rotation.yaw)
            pose_msg.pose.pose.orientation.x = quaternion[0]
            pose_msg.pose.pose.orientation.y = quaternion[1]
            pose_msg.pose.pose.orientation.z = quaternion[2]
            pose_msg.pose.pose.orientation.w = quaternion[3]

            pose_msg.twist.twist.linear.x = measurements.forward_speed * cos(measurements.transform.rotation.yaw*pi/180)
            pose_msg.twist.twist.linear.y = measurements.forward_speed * sin(measurements.transform.rotation.yaw*pi/180)

            temp.pose_pub.publish(pose_msg)

            client.send_control(
                steer=temp.joystick_command.steering,
                throttle=max(temp.joystick_command.throttle,0),
                brake=-min(temp.joystick_command.throttle,0),
                hand_brake=False,
                reverse=False)





def joystick_control_callback(msg):
    temp.joystick_command = msg
    #print(temp.joystick_command)



if __name__ == '__main__':

    rospy.init_node('carla')
    command_sub = rospy.Subscriber('/joystick/chassisCommand', chassisCommand, joystick_control_callback)
    temp.pose_pub = rospy.Publisher('/pose_estimate', Odometry, queue_size=1) 
    temp.image_pub = rospy.Publisher("camera_image",Image, queue_size=1)

    temp.bridge = CvBridge()

    # try to start the client - if connection error, then keep trying until success
    try:
        run_carla_client()
    except TCPConnectionError as error:
        logging.error(error)
        print('tcp connection error')

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')