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

from math import cos,sin,pi,floor

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
from autorally_msgs.msg import chassisState
from nav_msgs.msg import Odometry
from std_msgs.msg import Time
from rosgraph_msgs.msg import Clock
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
                QualityLevel='Low',
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

            publish_clock(measurements.game_timestamp/1000.0)
            publish_chassis_state(measurements.game_timestamp/1000.0)
            
            im = sensor_data.get('CameraRGB', None)
            if im is not None:
                # pprint(dir(im))
                # pprint(sensor_data.viewvalues())
                # pprint(sensor_data.viewitems())
                np_image = image_converter.to_rgb_array(im)
                image_message = temp.bridge.cv2_to_imgmsg(np_image, encoding="rgb8")
                image_message.header.stamp = rospy.Time.from_sec(measurements.game_timestamp/1000.0)
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

            player_measurements = measurements.player_measurements
            pose_msg = Odometry()

            pose_msg.header.stamp = rospy.Time.from_sec(measurements.game_timestamp/1000.0)
            pose_msg.header.frame_id = "odom"
            pose_msg.child_frame_id = "base_link"

            pose_msg.pose.pose.position.x = player_measurements.transform.location.x
            pose_msg.pose.pose.position.y = -player_measurements.transform.location.y
            pose_msg.pose.pose.position.z = 0  # Always on the ground planeplayer_measurements.transform.location.z
            
            quaternion = ros_tf.transformations.quaternion_from_euler(
                    -player_measurements.transform.rotation.roll*pi/180,
                     player_measurements.transform.rotation.pitch*pi/180,
                    -player_measurements.transform.rotation.yaw*pi/180)
            # print(player_measurements.transform.rotation.yaw)
            pose_msg.pose.pose.orientation.x = quaternion[0]
            pose_msg.pose.pose.orientation.y = quaternion[1]
            pose_msg.pose.pose.orientation.z = quaternion[2]
            pose_msg.pose.pose.orientation.w = quaternion[3]

            pose_msg.twist.twist.linear.x = player_measurements.velocity.x
            pose_msg.twist.twist.linear.y = -player_measurements.velocity.y
            pose_msg.twist.twist.linear.z = player_measurements.velocity.z

            # left handed coordinate system obey left hand rule for rotation
            pose_msg.twist.twist.angular.x = -player_measurements.angular_rate.x
            pose_msg.twist.twist.angular.y = player_measurements.angular_rate.y
            pose_msg.twist.twist.angular.z = -player_measurements.angular_rate.z

            temp.pose_pub.publish(pose_msg)

            client.send_control(
                steer=temp.joystick_command.steering,
                throttle=max(temp.joystick_command.throttle,0),
                brake=-min(temp.joystick_command.throttle,0),
                hand_brake=False,
                reverse=False)

def publish_clock(timestamp):
    t = Time()
    t.data = rospy.Time.from_sec(timestamp)
    #Simulator acts as a clock server
    clk = Clock()
    clk.clock.secs = t.data.secs
    clk.clock.nsecs = t.data.nsecs
    temp.time_pub.publish(clk)

def publish_chassis_state(timestamp):
    #Fill out the chassis state message
    chassis_state = chassisState()
    chassis_state.header.stamp = rospy.Time.from_sec(timestamp)
    chassis_state.header.frame_id = "CarlaCar"
    chassis_state.steering = temp.joystick_command.steering
    chassis_state.throttle = temp.joystick_command.throttle
    temp.chassis_pub.publish(chassis_state)    

def joystick_control_callback(msg):
    temp.joystick_command = msg
    #print(temp.joystick_command)

def mppi_control_callback(msg):
    temp.joystick_command = msg

if __name__ == '__main__':

    rospy.init_node('carla')
    command_sub = rospy.Subscriber('/joystick/chassisCommand', chassisCommand, joystick_control_callback)
    mppi_sub = rospy.Subscriber('/mppi_controller/chassisCommand', chassisCommand, mppi_control_callback)
    temp.pose_pub = rospy.Publisher('/pose_estimate', Odometry, queue_size=1) 
    temp.image_pub = rospy.Publisher("camera_image",Image, queue_size=1)
    temp.time_pub = rospy.Publisher("/clock", Clock, queue_size=1)
    temp.chassis_pub = rospy.Publisher("/chassisState", chassisState, queue_size=1)

    temp.bridge = CvBridge()

    # try to start the client - if connection error, then keep trying until success
    try:
        run_carla_client()
    except TCPConnectionError as error:
        logging.error(error)
        print('tcp connection error')

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
