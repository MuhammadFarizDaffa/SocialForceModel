#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D
import cv2
import numpy as np
import math

lidar_data = []
previous_robot_position = Pose2D()

def normalize_angle(angle):
    normalized_angle = angle % 360
    if normalized_angle < 0:
        normalized_angle += 360
    return normalized_angle

def laserCallback(msg):
    global lidar_data
    lidar_data = []

    for i in range(len(msg.ranges)):
        angle = msg.angle_min + i * msg.angle_increment
        L = msg.ranges[i] 

        if msg.range_min < L < msg.range_max:
            consAngle = angle + math.radians(previous_robot_position.theta)  # Sudut global lidar
            normalized_angle = normalize_angle(math.degrees(consAngle))

            xl = L * math.cos(math.radians(normalized_angle))
            yl = L * math.sin(math.radians(normalized_angle))

            lidar_data.append((xl, yl))

def position_callback(data):
    global previous_robot_position
    previous_robot_position = data

def main():
    rospy.init_node('lidar_mapping_node', anonymous=True)
    rospy.Subscriber('/hokuyo', LaserScan, laserCallback)
    rospy.Subscriber('/localization', Pose2D, position_callback)

    width, height = 800, 600
    mapping_image = np.zeros((height, width, 3), dtype=np.uint8)

    while not rospy.is_shutdown():
        robot_x = int(previous_robot_position.x * -50) + height // 2
        robot_y = int(previous_robot_position.y * -50) + width // 2

        if previous_robot_position.x != 0 and previous_robot_position.y != 0:
            tetha = math.radians(previous_robot_position.theta)
            xe = robot_x + int(-20 * math.cos(tetha))
            ye = robot_y + int(-20 * math.sin(tetha))

            cv2.circle(mapping_image, (robot_y, robot_x), 2, (255, 0, 0), -1)

            if lidar_data:
                for xl, yl in lidar_data:
                    x = int((xl + previous_robot_position.x) * -50) + height // 2
                    y = int((yl + previous_robot_position.y) * -50) + width // 2
                    cv2.circle(mapping_image, (y, x), 2, (0, 0, 255), -1)

        cv2.circle(mapping_image, (498, 322), 10, (0, 255, 0), -1)

        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(mapping_image, 'goal', (498 - 30, 322 - 30), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

        cv2.imshow('Lidar Mapping', mapping_image)
        cv2.waitKey(1)

    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
