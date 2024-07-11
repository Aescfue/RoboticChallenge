#!/usr/bin/env python3

""" A ROS planning node that subscribes to a costmap
  and generates a path by using the Djikstra algorithm"""

import sys
import math
import rospy
import tf
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped, PointStamped
from nav_msgs.msg import Path   
from nav_msgs.msg import OccupancyGrid
from dijkstra import Dijkstra

class Planner:
    def __init__(self):
        # As a first possibility, we will search for the init and the goal 
        # at the parameter server
        rospy.loginfo("In the planner constructor")

        self.base_frame_id = rospy.get_param("~base_frame_id", "base_footprint")
        self.global_frame_id = rospy.get_param("~global_frame_id", "map")
        self.listener = tf.TransformListener()
        self.new_goal = False
        
        # Get the position of the robot as the initial position
        self.initial_pos = None
        while self.initial_pos is None: 
            base_pos = PointStamped()
            base_pos.header.frame_id = self.base_frame_id
            base_pos.header.stamp = rospy.Time()
            try:
                self.initial_pos = self.listener.transformPoint(self.global_frame_id, base_pos)
                self.initx = self.initial_pos.point.x
                self.inity = self.initial_pos.point.y
            except:
                self.initial_pos = None
        
        if rospy.has_param('~goal'):
            self.goalx = rospy.get_param('~goal/x')
            self.goaly = rospy.get_param('~goal/y')
        rospy.loginfo("Here")
        print('Init (%f, %f). Goal (%f,%f): '%(self.initx, self.inity, self.goalx,self.goaly))

        self.path_publisher = rospy.Publisher('/path', Path, queue_size=10, latch=True)
        rospy.Subscriber('costmap_2d/costmap/costmap', OccupancyGrid, self.map_callback)
        rospy.Subscriber('goal_pose', PoseStamped, self.goal_callback) 
        self.listener = tf.TransformListener()
        self.init = False  # This flag would be raised in the map callback

    def map_callback(self, map):
        self.map = map
        if self.init == False or self.new_goal== True:
            self.init = True
            self.new_goal = False
            self.path = self.calculate_path(self.initx, self.inity, self.goalx, self.goaly)

            xpath = self.path[0]
            ypath = self.path[1]
            goals = []

            for i in range(len(xpath)):
                goals.append((xpath[i], ypath[i]))

            path_m = Path()

            path_m.header.stamp = rospy.Time.now()
            path_m.header.frame_id = "map"
            path_m.header.seq = 0

            for goal in goals:
                x = goal[0]
                y = goal[1]

                pose = PoseStamped()
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = 0

                path_m.poses.append(pose)

            self.path_publisher.publish(path_m)

            

    def goal_callback(self, map):
        print("TODO: Complete the goal callback so that a new path is generated whenever a goal is received")
        print("Note: you should get the current position and calculate the path")
        # Get the position of the robot as the initial position
        self.initial_pos = None
        while self.initial_pos is None: 
            base_pos = PointStamped()
            base_pos.header.frame_id = self.base_frame_id
            base_pos.header.stamp = rospy.Time()
            try:
                self.initial_pos = self.listener.transformPoint(self.global_frame_id, base_pos)
                self.initx = self.initial_pos.point.x
                self.inity = self.initial_pos.point.y
            except:
                self.initial_pos = None

        if rospy.has_param('~goal'):
            self.goalx = map.pose.position.x
            self.goaly = map.pose.position.y
            print("Goal %f %f",self.goalx,self.goaly)
        self.new_goal = True
        self.map_callback(self.map)
        

    def calculate_path (self, ix, iy, gx, gy):
        self.dijkstra = Dijkstra(self.map)
        return self.dijkstra.planning(ix, iy, gx, gy)
  
        
if __name__ == '__main__':
    try:
        # initiliaze
        rospy.init_node('path_publisher', anonymous=False)

        # Tell the user we are waiting for a costmap in order to operate
        rospy.loginfo("Starting planning node. Waiting for valid map. Press CTRL+C to exit")

        planner = Planner()     

        # The planning node will wait for new goals and the map at this rate
        r = rospy.Rate(10)

        # as long as you haven't ctrl + c keeping doing...
        while not rospy.is_shutdown():
            r.sleep()
    except:
        rospy.loginfo("Planning node terminated.")
