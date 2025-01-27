#! /bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped
import scipy.interpolate
import numpy as np
from scipy.spatial.distance import cdist


class Lane:

    def __init__(self, support_points):
        self.support_points = support_points
        self.spline_x = scipy.interpolate.CubicSpline(self.support_points[:, 0], self.support_points[:, [1]],
                                                      bc_type='periodic')
        self.spline_y = scipy.interpolate.CubicSpline(self.support_points[:, 0], self.support_points[:, [2]],
                                                      bc_type='periodic')

    def length(self):
        return self.support_points[:, 0][-1]

    def interpolate(self, param):
        return np.column_stack((self.spline_x(param), self.spline_y(param)))

    def closest_point(self, point, precision=0.001, min_param=0.0, max_param=-1.0):
        step_size = 0.2
        if max_param < 0:
            max_param = self.length()
        closest_param = -1.0

        while step_size > precision:
            params = np.arange(min_param, max_param, step_size)
            points = self.interpolate(params)

            closest_index = cdist([point], points, 'sqeuclidean').argmin()
            closest_param = params[closest_index]
            min_param = max(min_param, closest_param - step_size)
            max_param = min(max_param, closest_param + step_size)
            step_size *= 0.5

        return self.interpolate(closest_param), closest_param

    def lookahead_point(self, point, lookahead_distance):
        closest_point, closest_param = self.closest_point(point)
        return self.interpolate(closest_param + lookahead_distance), closest_param + lookahead_distance

    def curvature(self, param):
        x_1 = self.spline_x(param, 1)
        x_2 = self.spline_x(param, 2)

        y_1 = self.spline_y(param, 1)
        y_2 = self.spline_y(param, 2)

        return (x_1 * y_2 - y_1 * x_2) / ((x_1 ** 2.0 + y_1 ** 2.0) ** 1.5)


class Map:

    def __init__(self):
        self.lane_1 = np.load("/home/fabio/autominy/catkin_ws/src/assignment11/src/lane1.npy")
        self.lane_2 = np.load("/home/fabio/autominy/catkin_ws/src/assignment11/src/lane2.npy")
        self.lanes = [
            Lane(self.lane_1[[0, 50, 209, 259, 309, 350, 409, 509, 639, 750, 848, 948, 1028, 1148, 1200, 1276], :]),
            Lane(self.lane_2[[0, 50, 100, 150, 209, 400, 600, 738, 800, 850, 900, 949, 1150, 1300, 1476], :])]


class MapVisualization:

    def __init__(self):
        self.map = Map()
        rospy.init_node('listener', anonymous=True)
        self.lane_pub = rospy.Publisher("lane", Marker, queue_size=10)
        self.clicked_point_subscriber = rospy.Subscriber("/clicked_point", PointStamped, self.on_click, queue_size=1)

        self.rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            i = 0
            for lane in self.map.lanes:
                msg = Marker(type=Marker.LINE_STRIP, action=Marker.ADD)
                msg.header.frame_id = "map"
                msg.scale.x = 0.01
                msg.scale.y = 0.01
                msg.color.r = 1.0
                msg.color.a = 1.0
                msg.id = i

                for i in range(int(lane.length() * 100.0)):
                    inter = lane.interpolate(i / 100.0)
                    msg.points.append(Point(inter[0][0], inter[0][1], 0.0))
                i += 1

                self.lane_pub.publish(msg)

            self.rate.sleep()

    def on_click(self, point_msg):
        i = 2
        point = np.array([point_msg.point.x, point_msg.point.y])
        for lane in self.map.lanes:
            msg = Marker(type=Marker.SPHERE, action=Marker.ADD)
            msg.header.frame_id = "map"
            msg.scale.x = 0.1
            msg.scale.y = 0.1
            msg.scale.z = 0.1
            msg.color.b = 1.0
            msg.color.a = 1.0
            msg.id = i

            p, param = lane.closest_point(point)
            msg.pose.position.x = p[0][0]
            msg.pose.position.y = p[0][1]

            i += 1

            self.lane_pub.publish(msg)

            msg.color.b = 0.0
            msg.color.g = 1.0
            p, param = lane.lookahead_point(point, 0.5)
            msg.pose.position.x = p[0][0]
            msg.pose.position.y = p[0][1]
            msg.id = i

            i += 1

            self.lane_pub.publish(msg)


if __name__ == "__main__":
    MapVisualization()
