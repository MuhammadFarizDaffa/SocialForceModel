
import rospy
from std_msgs.msg import Float32MultiArray
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


def lidar_callback(data):
    global lidar_data
    lidar_data = data.data


def position_callback(data):
    global previous_robot_position
    previous_robot_position = data

def main():
    rospy.init_node('lidar_mapping_node', anonymous=True)
    rospy.Subscriber('/scanning', Float32MultiArray, lidar_callback)
    rospy.Subscriber('/localization', Pose2D, position_callback)

    width, height = 800, 600

    mapping_image = np.zeros((height, width, 3), dtype=np.uint8)
    
    while not rospy.is_shutdown():
        robot_x = int(previous_robot_position.x * -50) + height // 2
        robot_y = int(previous_robot_position.y * -50) + width // 2

        tetha = math.radians(previous_robot_position.theta)
        xe = robot_x + int(-20 * math.cos(tetha)) 
        ye = robot_y + int(-20 * math.sin(tetha)) 
        
        cv2.arrowedLine(mapping_image, (robot_y, robot_x), (ye, xe), (0, 0, 255), 2)

        if lidar_data:
            for i in range(0, len(lidar_data), 2):
                x = int((lidar_data[i] + previous_robot_position.x) * 50) + height // 2
                y = int((lidar_data[i + 1] + previous_robot_position.y) * 50) + width // 2
                cv2.circle(mapping_image, (y, x), 2, (0, 255, 0), -1)  # Menyimpan titik-titik lidar

        cv2.imshow('Lidar Mapping', mapping_image)
        cv2.waitKey(1)

    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
