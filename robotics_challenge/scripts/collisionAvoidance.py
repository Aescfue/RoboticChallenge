#!/usr/bin/env python3
import sys
import math
import rospy
import tf
import numpy as np
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker


class CollisionAvoidance():

    def __init__(self):
        self.listener = tf.TransformListener()
        # Escala de los puntos, tamaño
        self.scale = rospy.get_param("~scale", default=0.3)
        # Velocidad max linear en metros
        self.max_linear_speed = rospy.get_param("~max_linear_speed", 0.2)
        # Velocidad max de giro en metros
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 0.3)
        self.min_distance = rospy.get_param("~min_dist", default=2)
        # Parámetro del downsampler numero de saltos bucle
        self.n = rospy.get_param("~n", default=10)
        # mass del campo potencial
        self.mass = rospy.get_param("~mass", default=0.00001)

        # Subscribers: scan to see the obstacles; then cmd_vel_follower to get the commands from the follower
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        # Get the desired commands from the follower
        rospy.Subscriber("/cmd_vel_follower", Twist, self.cmd_vel_callback)
        self.marker_pub = rospy.Publisher(
            "~marker", Marker, queue_size=10, latch=True)
        # Publish the safe commands to the robot
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.laser_pub = rospy.Publisher(
            "~laser", LaserScan, queue_size=10, latch=True)
        # Setup the status flags
        self.laser_init = False
        self.cmd_vel_data = None

    # Cuando se reciban datos del laser del robot se guardaran en los atributos de clase

    def laser_callback(self, data):
        self.laser_data = data

        if not self.laser_init:
            self.laser_init = True

        if self.laser_init:

            # Ver si el laser detecta algo, si lo detecta primero hay que evaluar si está cerca
            # respecto a la base del robot y sumar las fuerzas opuestas que estén a menos de cierta distancia contando
            # la distancia entre ambos con una mass fija para cada punto

            self.laser_init = True
            # Codigo de ScanDownsampler que reduce las mediciones para no tener tantas

            angle = data.angle_min
            self.obstacles = []
            for i in range(0, len(data.ranges), self.n):
                if np.abs(data.ranges[i]) < 20.0:
                    p = Point()
                    p.x = data.ranges[i] * np.cos(angle)
                    p.y = data.ranges[i] * np.sin(angle)
                    p.z = 0
                    # Añadirlo a la lista de puntos obstaculos
                    self.obstacles.append(p)

                angle += self.n * data.angle_increment

            data.ranges = list(data.ranges)
            angle = data.angle_min
            for i in range(0, len(data.ranges)):
                if i % self.n != 0 or np.cos(angle) < 0:
                    data.ranges[i] = np.Infinity

                angle += data.angle_increment

            self.potential_control()

    def potential_control(self):
        # control potencial, segun la distancia al obtaculo crea fuerza repulsiva decreciente en distancia

        if self.cmd_vel_data is not None:
            found = False
            xSpeed, ySpeed = self.angularToCartesian(
                self.cmd_vel_data.linear.x, self.cmd_vel_data.angular.z)
            # Poner valores antiguos en la salida en caso de no detectar problemas
            linear = self.cmd_vel_data.linear.x
            angular = self.cmd_vel_data.angular.z
            if(linear == 0 and angular == 0):
                self.shutdown()

            too_close = False
            # obtener obtaculos, mirar si estan suficientemente cerca, sumar a la velocidad actual
            for i in range(0, len(self.obstacles)):
                dist = np.power(
                    self.obstacles[i].x, 2) + np.power(self.obstacles[i].y, 2)
                if(dist < self.min_distance):
                    # Sumar velocidad repulsiva
                    found = True
                    xSpeed = xSpeed - (self.mass*(self.obstacles[i].x / dist))
                    ySpeed = ySpeed - (self.mass*(self.obstacles[i].y / dist))

                if (0.15 < np.power(self.obstacles[i].x, 2) + np.power(self.obstacles[i].y, 2)):
                    too_close = True

            if found:
                linear, angular = self.cartesianToAngular(xSpeed, ySpeed)

            if linear > self.max_linear_speed:
                linear = self.max_linear_speed
            if angular > self.max_angular_speed:
                angular = self.max_angular_speed
                if too_close:
                    linear=0.05
            if(angular < (self.max_angular_speed*-1)):
                angular = self.max_angular_speed*-1
                if too_close:
                    linear=0.05

            rospy.loginfo("linear %f, Angular %f, Nuevo linear %f, Nuevo angular %f",
                          self.cmd_vel_data.linear.x, self.cmd_vel_data.angular.z, linear, angular)
            self.publish(linear, angular)

    # Transformar velocidad
    def angularToCartesian(self, linear, angular):
        xSpeed = linear * np.cos(angular)
        ySpeed = linear * np.sin(angular)
        return xSpeed, ySpeed

    def cartesianToAngular(self, xSpeed, ySpeed):
        linear = np.sqrt([np.power(xSpeed, 2) + np.power(ySpeed, 2)])
        angular = np.arctan2(ySpeed, xSpeed)
        return linear, angular

    def cmd_vel_callback(self, cmd_vel_data):
        self.cmd_vel_data = cmd_vel_data

    def command(self):
        angular = 0.0
        linear = 0.0
        if self.laser_init and self.cmd_vel_data is not None:
            rospy.loginfo_once(
                "Collision Avoidance: Laser and commands received")

            # Establecer limite de velocidad
            if linear > self.max_linear_speed:
                linear = self.max_linear_speed

            if angular > self.max_angular_speed:
                angular = self.max_angular_speed

    def publish(self, lin_vel, ang_vel):
        move_cmd = Twist()
        move_cmd.linear.x = lin_vel
        move_cmd.angular.z = ang_vel
        self.cmd_vel_pub.publish(move_cmd)

    def shutdown(self):
        rospy.loginfo("Stop Collision Avoidance")
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        rospy.init_node('collision_avoidance', anonymous=False)

        ca_instance = CollisionAvoidance()

        rospy.on_shutdown(ca_instance.shutdown)

        # La frecuencia de actualizacion del bucle es de 10 Hz
        r = rospy.Rate(10)

        while not rospy.is_shutdown():
            ca_instance.command()
            r.sleep()

    except:
        rospy.loginfo("Collision avoidance node terminated.")
