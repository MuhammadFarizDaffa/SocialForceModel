#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
import cv2
import numpy as np

vectort = Point()
vectors = Point()
vectorr = Point()

def target_callback(msg):
    global vectort
    vectort= msg

def statis_callback(msg):
    global vectors
    vectors= msg

def resultant_callback(msg):
    global vectorr
    vectorr= msg

def main():
    rospy.init_node('vector_display_node', anonymous=True)
    rospy.Subscriber('target', Point, target_callback)
    rospy.Subscriber('Statis', Point, statis_callback)
    rospy.Subscriber('Resultant', Point, resultant_callback)

    arrow_origin = (320, 240)  # Posisi tengah gambar sebagai titik awal panah

    rate = rospy.Rate(10)  # Frekuensi loop ROS (Hz)

    while not rospy.is_shutdown():
        # Buat gambar hitam
        image = np.zeros((480, 640, 3), dtype=np.uint8)

        # Gambar panah berdasarkan vectort
        endt = (int(arrow_origin[0] + vectort.y * -30), int(arrow_origin[1] + vectort.x * -30))
        ends = (int(arrow_origin[0] + vectors.y * -30), int(arrow_origin[1] + vectors.x * -30))
        endr = (int(arrow_origin[0] + vectorr.y * -5), int(arrow_origin[1] + vectorr.x * -5))

        # Warna panah dan keterangan nama panah
        color_t = (255, 0, 0)  # Panah target berwarna biru
        color_s = (0, 255, 0)  # Panah statis berwarna hijau
        color_r = (0, 0, 255)  # Panah resultant berwarna merah

        cv2.arrowedLine(image, arrow_origin, endt, color_t, 2)
        cv2.arrowedLine(image, arrow_origin, ends, color_s, 2)
        cv2.arrowedLine(image, arrow_origin, endr, color_r, 2)

        # Tambahkan keterangan nama panah
        cv2.putText(image, 'Target', endt, cv2.FONT_HERSHEY_SIMPLEX, 1, color_t, 2)
        cv2.putText(image, 'Statis', ends, cv2.FONT_HERSHEY_SIMPLEX, 1, color_s, 2)
        cv2.putText(image, 'Resultant', endr, cv2.FONT_HERSHEY_SIMPLEX, 1, color_r, 2)

        cv2.imshow('Vector Direction', image)  # Menampilkan gambar dengan panah
        cv2.waitKey(1)  # Tunggu 1 ms untuk menampilkan gambar

        rate.sleep()

    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
