#!/usr/bin/env python3


# A very basic TurtleBot script that moves TurtleBot forward indefinitely. Press CTRL + C to stop.  To run:
# On TurtleBot:
# roslaunch turtlebot_bringup minimal.launch
# On work station:
# python goforward.py

import sys
import math
import rospy
import tf
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped

from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker


class Turtlebot():

    def __init__(self):

        # Create a publisher which can "talk" to TurtleBot and tell it to move
        # Tip: You may need to change cmd_vel_follower_mux/input/navi to /cmd_vel_follower if you're not using TurtleBot2
        self.cmd_vel_follower = rospy.Publisher(
            'cmd_vel_follower', Twist, queue_size=10)

        self.recorrido = None
        self.contador = 0
        rospy.Subscriber("path", Path, self.path_callback)
        self.listener = tf.TransformListener()
        self.path_received = False
        # Marcador para debugear path
        self.marker_pub = rospy.Publisher(
            "~marker", Marker, queue_size=10, latch=True)

        # Carga inicial de variables
        self.max_linear_speed = rospy.get_param("~max_linear_speed", 0.2)
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 0.3)
        self.indice_punto = 0

        rospy.loginfo("Max linear speed: %f", self.max_linear_speed)
        rospy.loginfo("Max angular speed: %f", self.max_angular_speed)

    # This module control the path of the robot
    def path_callback(self, path):
        if self.path_received is not True:
            self.path_received = True
        self.recorrido = path.poses
        rospy.loginfo("Camino recibido.")

    # This module control the aceleration of the robot
    def speedControl(self, base_goal):
        linear = math.sqrt(math.pow(base_goal.point.x, 2) +
                           math.pow(base_goal.point.y, 2))
        angular = math.atan2(base_goal.point.y, base_goal.point.x)
        if (math.dist((0, 0), (base_goal.point.x, base_goal.point.y)) < 0.1):
            linear = 0
            angular = 0
            self.shutdown()
        if (linear > self.max_linear_speed):
            linear = self.max_linear_speed
        if(angular > self.max_angular_speed):
            angular = self.max_angular_speed
        if(angular < (self.max_angular_speed*-1)):
            angular = self.max_angular_speed*-1
        self.publish(linear, angular)

    def pathControl(self):
        base_goal = None
        # calcular punto mas cercano convirtiendo puntos a base del robot
        # y viendo cual tiene menos distancia respecto el 0,0
        puntos = self.recorrido
        new_goal = None
        dist_goal = float('inf')  # distancia maxima del sistema
        i = 0
        for punto in puntos:

            goal = PointStamped()  # Punto actual
            base_goal = PointStamped()  # Punto comvertido a referencia de laz
            goal.header.frame_id = "map"
            goal.header.stamp = rospy.Time()
            goal.point.x = punto.pose.position.x
            goal.point.y = punto.pose.position.y
            goal.point.z = 0.0

            base_goal = self.odom_to_base(goal)

            new_goal_dist = math.dist(
                (0, 0), (base_goal.point.x, base_goal.point.y))

            if new_goal == None or new_goal_dist < dist_goal:
                new_goal = base_goal
                dist_goal = new_goal_dist
                self.indice_punto = i

            i += 1

        # poner como objetivo el punto siguiente tras este si existe
        # buscar siguiente punto objetivo sabiendo la posicion que era indice_punto
        # partiendo desde indice_punto mirar los sguientes puntos hasta encontrar uno
        # que cumpla una minima distancia objetiva

        min_dist = 0.5
        encontrado = False
        j = self.indice_punto
        while(j < len(puntos) and not encontrado):
            punto = puntos[j]

            goal = PointStamped()
            base_goal = PointStamped()
            goal.header.frame_id = "map"
            goal.header.stamp = rospy.Time()
            goal.point.x = punto.pose.position.x
            goal.point.y = punto.pose.position.y
            goal.point.z = 0.0

            base_goal = self.odom_to_base(goal)

            if(math.dist((0, 0), (base_goal.point.x, base_goal.point.y)) >= min_dist):
                new_goal = base_goal
                encontrado = True

            if not encontrado:
                new_goal = base_goal

            j += 1

        # debug de new goal para ver en el marcador donde se asigna

        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "base_footprint"
        marker.id = 0
        marker.ns = "map"
        marker.lifetime.secs = 100
        # x width punto y height punto
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0
        p = Point()
        p.x = new_goal.point.x
        p.y = new_goal.point.y
        p.z = 0
        marker.points.append(p)

        marker.type = marker.POINTS
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.b = 0.0
        marker.color.g = 0.0

        self.marker_pub.publish(marker)

        # fin debug marker

        return new_goal

    def odom_to_base(self, goal):
        try:
            base_goal = self.listener.transformPoint('base_footprint', goal)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("Problem TF")
            return
        return base_goal

    def command(self, gx, gy):

        # TODO: put the control law here

        if self.path_received:
            base_goal = self.pathControl()
            self.speedControl(base_goal)

    def publish(self, lin_vel, ang_vel):
        # Twist is a datatype for velocity
        move_cmd = Twist()
        # let's go forward
        move_cmd.linear.x = lin_vel
        # let's turn
        move_cmd.angular.z = ang_vel
        self.cmd_vel_follower.publish(move_cmd)

    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
        # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel_follower.publish(Twist())
        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)


if __name__ == '__main__':

    # initiliaze
    rospy.init_node('robotcontrol', anonymous=False)

    # tell user how to stop TurtleBot
    rospy.loginfo("To stop TurtleBot CTRL + C")

    robot = Turtlebot()
    # What function to call when you ctrl + c
    rospy.on_shutdown(robot.shutdown)

    goalx = rospy.get_param("~xPoint", 6.0)
    goaly = rospy.get_param("~yPoint", 3.0)

    print(goalx)
    print(goaly)

    # TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ
    r = rospy.Rate(10)

    # as long as you haven't ctrl + c keeping doing...
    while not rospy.is_shutdown():
        # publish the velocity
        robot.command(goalx, goaly)
        # wait for 0.1 seconds (10 HZ) and publish again
        r.sleep()
