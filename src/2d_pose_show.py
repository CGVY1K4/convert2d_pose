#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
import cv2


# 3D -> 2D 변환 함수
def transform_3d_to_2d(point_3d, scale_factor, grid_min, grid_max):
    norm_x = ((grid_max[0] - grid_min[0]) - 1) / (grid_max[0] - grid_min[0])
    norm_z = ((grid_max[1] - grid_min[1]) - 1) / (grid_max[1] - grid_min[1])
    x = int((np.floor(point_3d[0] * scale_factor) - grid_min[0]) * norm_x)
    z = int((np.floor(point_3d[2] * scale_factor) - grid_min[1]) * norm_z)
    return (x, z)

def camera_pose_callback(msg):
    # 현재 카메라 3D 포즈 정보 로깅
    rospy.loginfo(f"Received camera pose: Position (x: {-msg.pose.pose.position.y}, y: {-msg.pose.pose.position.z}, z: {msg.pose.pose.position.x})")

    # 3D 좌표를 2D로 변환
    scale_factor = 10 # 스케일 인자
    grid_min = (-163, -361) # 그리드 최소값
    grid_max = (221, 377) # 그리드 최대값
    point_3d = (-msg.pose.pose.position.y, -msg.pose.pose.position.z, msg.pose.pose.position.x)
    point_2d = transform_3d_to_2d(point_3d, scale_factor, grid_min, grid_max)

    # 변환된 2D 좌표 로깅
    rospy.loginfo(f"Transformed 2D position: (x: {point_2d[1]}, y: {point_2d[0]})")

    msg = Vector3()
    msg.x = point_2d[1]
    msg.y = point_2d[0]
    msg.z = 0

    currpos_pub.publish(msg)

    # 이미지에 2D 좌표 표시
    out_fname = 'output'  # 출력 파일 이름 지정
    color_image = cv2.imread('/home/cgvlab/catkin_ws/src/convert2d_pose/src/map.png')  # 기존 이미지 파일 로드, 'your_image_path.png'는 실제 이미지 경로로 변경

    # 변환된 2D 좌표에 원 그리기
    cv2.circle(color_image, (point_2d[1], point_2d[0]), 5, (0, 255, 0), -1)  # 녹색 원으로 표시

    # 변환된 이미지 저장 및 표시
    # cv2.imwrite(f'{out_fname}.png', color_image)  # 이미지 파일로 저장
    cv2.imshow(out_fname, color_image)  # 화면에 이미지 표시

    cv2.waitKey(1)  # 변경: 이전에는 cv2.waitKey(0)이었지만, ROS 콜백 내에서는 cv2.waitKey(1)을 사용하여 GUI 이벤트 처리




def camera_pose_listener():
    # ROS 노드 초기화
    rospy.init_node('camera_pose_subscriber_node', anonymous=True)

    # '/run_slam/camera_pose' 토픽 구독
    rospy.Subscriber('/run_slam/camera_pose', Odometry, camera_pose_callback)

    # 프로그램 유지
    rospy.spin()

if __name__ == '__main__':
    try:
        currpos_pub = rospy.Publisher('current_pose', Vector3, queue_size=10)
        camera_pose_listener()
    except rospy.ROSInterruptException:
        pass