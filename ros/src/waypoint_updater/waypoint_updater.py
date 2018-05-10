#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TwistStamped, PoseStamped
from std_msgs.msg import Int32
from styx_msgs.msg import TrafficLightArray, TrafficLight, Lane, Waypoint
from scipy.spatial import KDTree
import numpy as np
import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.
'''

# Globals
LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
MAX_DECEL = .5      #


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # ROS Subscribers
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        # rospy.Subscriber('/obstacle_waypoint', TODO, self.obstacle_cb)

        # ROS Publishers
        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        # Class member variables
        self.pose = None
        self.waypoints = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.stopline_wp_idx = -1
        self.prev_stopline = -1
        self.final_lane = None

        self.loop()

    def loop(self):
        """Main loop for controlling frequency of publishing waypoints

        """
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.pose and (self.waypoint_tree is not None):
                self.publish_waypoints()
            rate.sleep()

    def publish_waypoints(self):
        """Prepare and publish the message for /final_waypoints

        """
        final_lane = self.generate_lane()
        self.final_lane = final_lane
        final_lane.header.stamp = rospy.Time.now()
        self.final_waypoints_pub.publish(final_lane)

    def generate_lane(self):
        """Generate the lane ahead of the car

        Returns:
            Lane: list of waypoints with target velocities set as needed

        """
        lane = Lane()
        closest_idx = self.get_closest_waypoint_idx()
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        self.base_waypoints = self.waypoints.waypoints[closest_idx:farthest_idx]

        # If next stop line is within our projected waypoints, decelerate
        if self.stopline_wp_idx > (closest_idx - 2) and (self.stopline_wp_idx <= farthest_idx):
            self.decelerate_waypoints(closest_idx)

        lane.waypoints = self.base_waypoints
        return lane


    def get_closest_waypoint_idx(self):
        """Get the closest waypoint index to current pose

        Returns:
            int: integer index of the nearest waypoint in global waypoints list

        """
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]

        # Check if closest is ahead of or behind vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx - 1]

        # Equation for hyperplane through closest_coords
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])

        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)

        if val > 0.0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        return closest_idx

    def decelerate_waypoints(self, closest_idx):
        """Modify forward waypoint velocities to decelerate the car

        Args:
            waypoints (Lane): list of waypoints to modify velocities
            closest_idx (int): nearest waypoint to car and first waypoint

        Returns:
            Lane: list of waypoints with modified target velocities

        """
        temp = []
        # Two waypoints back from line so front of car stops at line
        stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0)

        for i, wp in enumerate(self.base_waypoints):
            dist = self.distance(self.base_waypoints, i, stop_idx)
            # Deceleration equation:
            vel = math.sqrt(2 * MAX_DECEL * dist)

            if vel < 1.:
                vel = 0.

            self.set_waypoint_velocity(wp, vel)

    def pose_cb(self, msg):
        """Pose callback to store pose data from /current_pose topic

        Args:
            msg (PoseStamped): self driving vehicle pose

        """
        self.pose = msg

    def waypoints_cb(self, waypoints):
        """Waypoints callback to store waypoints from /base_waypoints topic

        Args:
            waypoints (Lane): default list of waypoints to follow
        """
        self.waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        """Traffic callback to store stop line waypoint index from /traffic_waypoint topic

        Args:
            msg (Int32): integer value of waypoint to stop at for red light

        """
        self.stopline_wp_idx = msg.data
        # If this is a new stop line, update the waypoints
        if self.stopline_wp_idx != self.prev_stopline:
            self.prev_stopline = self.stopline_wp_idx
            self.publish_waypoints()

    def obstacle_cb(self, msg):
        """Obstacle callback to store obstacle data from /obstacle_waypoint topic.
            Unimplemented.

        """
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        """Get waypoint velocity and return it.

        Returns:
            float: waypoint linear velocity

        """
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoint, velocity):
        """Set specific waypoint velocity

        Args:
            waypoint (Waypoint): waypoint to update
            velocity (float): velocity value to be updated to

        """
        waypoint.twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        """Receives list of traffic light locations for localization.

        Args:
            waypoints (Lane): list of waypoints containing the two waypoints passed in
            wp1 (int): index of the first waypoint within waypoints
            wp2 (int): index of the second waypoint within waypoints

        Returns:
            float: distance between the two waypoints
        """
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
