#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
from scipy.spatial import KDTree
import numpy as np
import math
import tf
import cv2
import yaml

import csv

STATE_COUNT_THRESHOLD = 3
label = ['RED', 'YELLOW', 'GREEN', '', 'UNKNOWN']

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        # ROS Subscribers
        self.sub_raw_image = rospy.Subscriber('/image_color', Image, self.image_cb)
        self.sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        self.sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)

        # ROS Publishers
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        # Class Member Variables
        self.pose = None
        self.waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.camera_image = None
        self.lights = []
        self.lights_2d = None
        self.light_tree = None
        self.update_rate = 10
        self.nwp = None
        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.ntlwp = None

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()

        self.initialized = True

        # Image Collection:
        self.i = 0
        self.last_pose = None
        session = 'session2'
        self.session = 'light_classification/collections2/'+session
        self.jpgout = self.session + '_%d.jpg'

        # Set up CSV file for collecting image data
        fieldname = ['x', 'y', 'z', 'ax', 'ay', 'az', 'aw', 'image', 'label']
        self.log_file = open(self.session+'.csv', 'w')
        self.log_writer = csv.DictWriter(self.log_file, fieldnames=fieldname)
        self.log_writer.writeheader()

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.loop()

    def loop(self):
        """Main loop for detecting closest traffic light waypoint

        """
        rate = rospy.Rate(self.update_rate)
        while not rospy.is_shutdown():
            if not self.initialized:
                if self.waypoint_tree and self.pose:
                    self.nwp = self.get_closest_waypoint(self.pose.position.x, self.pose.position.y)
                    closest_idx = self.get_closest_light(self.pose.position.x, self.pose.position.y)
                    stop_line_x = self.config['stop_line_positions'][closest_idx][0]
                    stop_line_y = self.config['stop_line_positions'][closest_idx][1]
                    self.ntlwp = self.get_closest_waypoint(stop_line_x, stop_line_y)
                    distance = self.distance(self.waypoints, self.nwp, self.ntlwp)
                    #print("Stop Line: {}, {}   Current Location: {}, {}    Distance: {}".format(stop_line_x, stop_line_y, self.pose.position.x, self.pose.position.y, distance))
                    if distance > 100. or self.ntlwp < (self.nwp-2):
                        self.ntlwp = None
                    # Image Collection, uncomment to automatically store images
                    #else:
                    #    self.save_image()
                    if self.ntlwp is not None and self.sub_raw_image is None:
                        self.sub_raw_image = rospy.Subscriber('/image_color', Image, self.image_cb)
                    elif self.ntlwp is None and self.sub_raw_image is not None:
                        self.sub_raw_image.unregister()
                        self.sub_raw_image = None
                        self.last_wp = -1
                        self.upcoming_red_light_pub.publish(Int32(self.last_wp))
                    else:
                        self.upcoming_red_light_pub.publish(Int32(self.last_wp))
            elif self.light_classifier is None:
                self.upcoming_red_light_pub.publish(Int32(-1))
            rate.sleep()

    def pose_cb(self, msg):
        """Records the current pose, position, and orientation of the self driving vehicle.

        Args:
            msg (PoseStamped): current pose of vehicle

        """
        self.pose = msg.pose
        self.position = self.pose.position
        self.orientation = self.pose.orientation

    def waypoints_cb(self, waypoints):
        """Stores the base waypoints locally for processing.

        Args:
            waypoints (Lane): list of all waypoints that the car should follow

        """
        if self.waypoints is None:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)
            self.waypoints = []
            for waypoint in waypoints.waypoints:
                self.waypoints.append(waypoint)

    def traffic_cb(self, msg):
        """Receives list of traffic light locations for localization.

        Args:
            msg (TrafficLightArray): array of Traffic Light objects

        """
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        if hasattr(msg, 'encoding'):
            if msg.encoding == '8UC3':
                msg.encoding = "rgb8"
        else:
            msg.encoding = 'rgb8'

        self.camera_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
        self.state_count += 1
        if self.initialized:
            self.initialized = False

    def save_image(self):
        """Saves traffic light training images given vehicle position is not
            the same as previous image.

        """
        # SAVING IMAGES FOR TRAINING
        if self.state >= 0:
            if self.last_pose is None or self.last_pose != self.pose:
                self.last_pose = self.pose
                self.log_writer.writerow({
                    'x': self.pose.position.x,
                    'y': self.pose.position.y,
                    'z': self.pose.position.z,
                    'ax': self.pose.orientation.x,
                    'ay': self.pose.orientation.y,
                    'az': self.pose.orientation.z,
                    'aw': self.pose.orientation.w,
                    'image': "'"+self.jpgout%(self.i)+"'",
                    'label': self.state})
                self.log_file.flush()
                cv2.imwrite(self.jpgout%(self.i), cv2.cvtColor(self.camera_image, cv2.COLOR_RGB2BGR))
                self.i += 1
                rospy.logwarn("Output Image Number: {}".format(self.i))


    def distance(self, waypoints, wp1, wp2):
        """Receives list of traffic light locations for localization.

        Args:
            waypoints (Lane): list of waypoints containing the two waypoints passed in
            wp1 (int): index of the first waypoint within waypoints
            wp2 (int): index of the second waypoint within waypoints

        Returns:
            double: distance between the two waypoints
        """
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def get_closest_waypoint(self, x, y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]
        return closest_idx

    def get_closest_light(self, x, y):
        """Identifies the closest traffic light to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest light in self.lights

        """
        if not self.light_tree:
            self.lights_2d = [[light.pose.pose.position.x, light.pose.pose.position.y] for light in self.lights]
            self.light_tree = KDTree(self.lights_2d)
        distance, closest_idx = self.light_tree.query([x, y], 1)
        coords = self.lights_2d[closest_idx]
        return closest_idx

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # Get classification if traffic_classifier is initialized, else UNKNOWN
        if self.light_classifier is not None:
             classification = self.light_classifier.get_classification(self.camera_image)
        else:
             classification = TrafficLight.UNKNOWN

        # Get the nearest traffic light index if data is available
        if len(self.lights) > 0 and self.pose:
            light_idx = self.get_closest_light(self.pose.position.x,
                                                self.pose.position.y)
            # The closest Traffic Light
            light_wp = self.lights[light_idx]
            light_state = light_wp.state
            # Simulator traffic light correction
            if light_state != classification:
                classification = light_state

        return classification


    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        if self.initialized:
            state = self.get_light_state(0)
            return -1, TrafficLight.UNKNOWN
        elif self.ntlwp:
            state = self.get_light_state(self.ntlwp)
            return self.ntlwp, state
        return -1, TrafficLight.UNKNOWN


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
