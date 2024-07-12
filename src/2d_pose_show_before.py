#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Vector3
import cv2

# pip install transforms3d
import transforms3d

received_paths = False

# 3D -> 2D 변환 함수
def transform_3d_to_2d(point_3d, scale_factor, grid_min, grid_max):
    norm_x = ((grid_max[0] - grid_min[0]) - 1) / (grid_max[0] - grid_min[0])
    norm_z = ((grid_max[1] - grid_min[1]) - 1) / (grid_max[1] - grid_min[1])
    x = int((np.floor(point_3d[0] * scale_factor) - grid_min[0]) * norm_x)
    z = int((np.floor(point_3d[2] * scale_factor) - grid_min[1]) * norm_z)
    return (x, z)

paths = []
def path_callback(msg):
    global paths

    # 토픽 메시지 수신 시 호출되는 콜백 함수
    # rospy.loginfo("Received Vector3 message: ({}, {}, {})".format(msg.x, msg.y, msg.z))
    # 수신된 Vector3 메시지를 경로 시퀀스에 추가합니다.
    paths.append((msg.x, msg.y, msg.z))

    global received_paths
    received_paths = True

def path_subscriber():
    # ROS 노드 초기화
    # rospy.init_node('path_subscriber', anonymous=False)

    # 'vector3' 토픽을 구독하는 subscriber 생성
    print('Ready to go!')
    rospy.Subscriber('path_goal', Vector3, path_callback)

    while not rospy.is_shutdown() and not received_paths:
        rospy.loginfo("Waiting for paths...")
        rospy.sleep(5)

camera_poses = set()
def camera_pose_callback(msg):
    # 현재 카메라 3D 포즈 정보 로깅
    # rospy.loginfo(f"Received camera pose: Position (x: {-msg.pose.pose.position.y}, y: {-msg.pose.pose.position.z}, z: {msg.pose.pose.position.x})")
    # rospy.loginfo(f"Received camera pose: Quaternion (x: {msg.pose.pose.orientation.x}, y: {msg.pose.pose.orientation.y}, z: {msg.pose.pose.orientation.z}, w: {msg.pose.pose.orientation.w})")
    # 3D 좌표를 2D로 변환

    #############################################
    #### first map -> map.png ####
    #### file name -> 0123    ####
    '''
    scale_factor = 10 # 스케일 인자
    grid_min = (-163, -361) # 그리드 최소값
    grid_max = (221, 377) # 그리드 최대값
    '''
    #############################################

    #############################################
    #### second map -> map_oh_0314_1_scale_30.png
    #### file name  -> 0314_1_more_keyfrm
    '''
    scale_factor = 30
    grid_min = (-620, -738)
    grid_max = (266, 411)
    '''
    #############################################

    #############################################
    #### map_0429.jpg
    #### file name  -> oh_4f_0429
    
    scale_factor = 50
    grid_min = (-175, -331)
    grid_max = (375, 569)
    
    #############################################
    
    point_3d = (-msg.pose.pose.position.y, -msg.pose.pose.position.z, msg.pose.pose.position.x)
    point_2d = transform_3d_to_2d(point_3d, scale_factor, grid_min, grid_max)

    # 변환된 2D 좌표 로깅
    rospy.loginfo(f"Transformed 2D position: (x: {point_2d[1]}, y: {point_2d[0]})")
    

    new_msg = Vector3()
    new_msg.x = point_2d[1]
    new_msg.y = point_2d[0]
    global camera_poses
    camera_poses.add((point_2d[0], point_2d[1]))
    angle = transforms3d.euler.quat2euler([msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z], axes='sxyz')
    yaw = angle[2] - 3.14/4 # need normalization???
    rospy.loginfo(f"Transformed yaw: (z: {yaw})")
    new_msg.z = yaw

    currpos_pub.publish(new_msg)

    draw_image(point_2d[0], point_2d[1], msg.pose.pose.orientation.w)


def draw_image(x, y, yaw):
    # 이미지에 2D 좌표 표시
    out_fname = 'output'  # 출력 파일 이름 지정
    color_image = cv2.imread('/home/cgvlab/catkin_ws/src/convert2d_pose/src/map_0429_2.jpg')  # 기존 이미지 파일 로드, 'your_image_path.png'는 실제 이미지 경로로 변경
    # color_image = cv2.imread('/home/cgvlab/catkin_ws/src/convert2d_pose/src/map_oh_0314_1_scale_30.png')

    global paths
    map_val =  550
    for i in paths:
                                #      x          y 
        cv2.circle(color_image, (int(i[0]), map_val-int(i[1])), 1, (255, 0, 0), -1)
    
    # 변환된 2D 좌표에 원 그리기
    
    global camera_poses
    for i in camera_poses:
        cv2.circle(color_image, (i[1], i[0]), 1, (0, 255, 0), -1)  # 녹색 원으로 표시
    
    cv2.circle(color_image, (y, x), 5, (0, 255, 0), -1)  # 녹색 원으로 표시

    # 화살표 길이
    arrow_length = 5

    # 화살표 끝점 계산
    end_x = int(x + arrow_length * np.sin(yaw))
    end_y = int(y + arrow_length * np.cos(yaw))

    # 화살표 그리기
    cv2.arrowedLine(color_image, (y, x), (end_y, end_x), (0, 255, 0), 2, tipLength=0.5)

    # 변환된 이미지 저장 및 표시
    # cv2.imwrite(f'{out_fname}.png', color_image)  # 이미지 파일로 저장
    cv2.imshow(out_fname, color_image)  # 화면에 이미지 표시

    cv2.waitKey(1)  # 변경: 이전에는 cv2.waitKey(0)이었지만, ROS 콜백 내에서는 cv2.waitKey(1)을 사용하여 GUI 이벤트 처리



if __name__ == '__main__':
    # draw_image(1, 1)
    
    try:
        rospy.init_node('camera_pose_subscriber_node', anonymous=True)

        currpos_pub = rospy.Publisher('current_pose', Vector3, queue_size=10)
        # ROS 노드 초기화
    
        # '/run_slam/camera_pose' 토픽 구독
        path_subscriber()
        rospy.Subscriber('/run_slam/camera_pose', Odometry, camera_pose_callback)

        # 프로그램 유지
        rospy.spin()

    except rospy.ROSInterruptException:
        pass