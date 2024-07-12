#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import pygame
from pygame.locals import *

import numpy as np
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Vector3, PoseWithCovarianceStamped, PoseStamped, Pose, Quaternion, Twist
from std_msgs.msg import Header, Int32, Bool

# pip install transforms3d
import transforms3d

import math
import tf

from enum import Enum

class State(Enum):
    START = 0
    IDLE = 1
    TRACKING = 2
    TRACKING_LOSS = 3
    RECOVERY = 4  # not used

class RobotTracker:
    def __init__(self, map_image_path):
        
        self.map_image_path = map_image_path

        pygame.init()
        self.screen = pygame.display.set_mode((800, 600))
        pygame.display.set_caption("Robot Tracker")
        self.clock = pygame.time.Clock()

        pygame.font.init()
        self.state_text = pygame.font.SysFont('Comic Sans MS', 30)


        self.robot_pose = (0, 0, 0)  # (x, y, yaw)
        # self.robot_points = [(5, -5), (0, 0), (5, 5)]

        self.map_image = pygame.image.load(self.map_image_path)
        self.map_rect = self.map_image.get_rect()

        # self.robot_dot = self.canvas.create_polygon(self.robot_points, fill='red', outline='black')

        self.past_robot_poses = list()
        self.searched_paths = []
        self.prev_paths = []
        self.map_val =  550


        # ROS Subscribers
        rospy.init_node('robot_tracker')
        rospy.Subscriber('/run_slam/camera_pose', Odometry, self.camera_pose_callback)
        rospy.Subscriber('/run_hybrid_astar/searched_path', Path, self.path_callback)
        rospy.Subscriber('path_index', Int32, self.path_index_callback)
        rospy.Subscriber('run_slam/loss', Bool, self.trackloss_callback)
        self.currpos_pub = rospy.Publisher('current_pose', Vector3, queue_size=10)

        self.initpose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        self.goalpose_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        self.searched_path_pub = rospy.Publisher('/2d_pose_show_newnew/searched_path', Path, queue_size=10)

        self.twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        self.mouse_down_pos = (0, 0) 
        self.mouse_up_pos = (0, 0)
        self.goal_yaw = 0

        self.goal_pose = (0, 0, 0) # x, y, yaw
        # rostopic pub --once /initialpose geometry_msgs/PoseWithCovarianceStamped '{header: {seq: 0, stamp: now, frame_id: "world"}, pose: {pose: {position: {x: 300.0, y: 383.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]}}'
        # rostopic pub --once /move_base_simple/goal geometry_msgs/PoseStamped "{header: {seq: 0, stamp: now, frame_id: 'world'}, pose: {position: {x: 500.0, y: 383.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"

        self.running = True

        self.path_index = 0
        
        self.state = State.START
    
        
    def trackloss_callback(self, msg):
        if self.state == State.IDLE:
            self.state = State.START
        elif self.state == State.TRACKING:
            self.state = State.TRACKING_LOSS
        # elif self.state == State.RECOVERY:
        #     self.state = State.TRACKING_LOSS

    def path_index_callback(self, msg):
        self.path_index = msg.data

        if self.path_index >= len(self.searched_paths):
            self.state = State.IDLE

    def path_callback(self, msg):
        # rospy.loginfo("Received Vector3 message: ({}, {}, {})".format(msg.x, msg.y, msg.z))

        self.prev_paths = self.searched_paths

        self.searched_paths.clear()
        for pose in msg.poses:
            # rospy.loginfo(f"{pose.pose.position}")
            pos = pose.pose.position
            self.searched_paths.append((pos.x, self.map_val - pos.y))
            # self.searched_paths.append((msg.x, msg.y, msg.z))

        # send to vec3_subscribe_moveToBase_new_new
        self.searched_path_pub.publish(msg)


        if self.state == State.IDLE:
            self.state = State.TRACKING

    # 3D -> 2D 변환 함수
    def transform_3d_to_2d(self, point_3d, scale_factor, grid_min, grid_max):
        norm_x = ((grid_max[0] - grid_min[0]) - 1) / (grid_max[0] - grid_min[0])
        norm_z = ((grid_max[1] - grid_min[1]) - 1) / (grid_max[1] - grid_min[1])
        x = int((np.floor(point_3d[0] * scale_factor) - grid_min[0]) * norm_x)
        z = int((np.floor(point_3d[2] * scale_factor) - grid_min[1]) * norm_z)
        return (x, z)

    def camera_pose_callback(self, msg):
        #############################################
        #### map_0429.jpg
        #### file name  -> oh_4f_0429
        scale_factor = 50 
        grid_min = (-175, -331)
        grid_max = (375, 569)
        #############################################

        msg = msg.pose.pose
        
        point_3d = (-msg.position.y, -msg.position.z, msg.position.x)
        point_2d = self.transform_3d_to_2d(point_3d, scale_factor, grid_min, grid_max)

        angle = transforms3d.euler.quat2euler([msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z], axes='sxyz')
        yaw = angle[2]
        

        # update robot pose
        # x, y, yaw
        
        self.past_robot_poses.append(self.robot_pose)
        if len(self.past_robot_poses) > 100:
            self.past_robot_poses.pop(0)
            
        self.robot_pose = (point_2d[1], point_2d[0], yaw)

        self.publish_robot_pose()

        if self.state == State.START:
            self.state = State.IDLE
        elif self.state == State.TRACKING_LOSS:
            # self.state = State.RECOVERY
            # self.publish_init_goal_pose() # find new path
            self.state = State.IDLE
        

    def draw_searched_paths(self, ):
        if len(self.searched_paths) >= 2:
            pygame.draw.lines(self.screen, 'green', closed=False, points=self.searched_paths, width=3)
        elif len(self.prev_paths) >= 2:
            pygame.draw.lines(self.screen, (0,150,0), closed=False, points=self.prev_paths, width=3)
        
        # draw arrow
        self.draw_arrow(self.screen, 'blue', 'blue', self.mouse_down_pos, self.mouse_up_pos, self.goal_yaw)


    def draw_robot_pose(self, color):
        self_x, self_y, self_yaw = self.robot_pose
        yaw = self_yaw
        yaw+=(math.pi)/2
        if(yaw > math.pi):
            yaw-=2*math.pi
        yaw*(-1)
        radius = 5
        pygame.draw.circle(self.screen, color, (int(self_x), int(self_y)), radius)  # 빨간 동그라미 그리기
        pygame.draw.line(self.screen, 'black', (int(self_x), int(self_y)), ((int(self_x) + 10*math.cos(yaw)), int(self_y) - 10*math.sin(yaw)), width=3)


    def publish_robot_pose(self):
        # rospy.loginfo(f"Robot pose: (x: {self.robot_pose[0]}, y: {self.robot_pose[1]}, yaw: {self.robot_pose[2]})")
        new_msg = Vector3()
        new_msg.x, new_msg.y, new_msg.z = self.robot_pose
        self.currpos_pub.publish(new_msg) 

    
    def publish_init_pose(self, x, y, yaw):
        ''' yaw is in radian '''

        yaw = yaw+(math.pi/2)
        if(yaw > math.pi):
            yaw = yaw - 2*math.pi
        # yaw = yaw*(-1)

        y = self.map_val - y

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "world" 

        pose_msg.pose.pose.position.x = x 
        pose_msg.pose.pose.position.y = y
        pose_msg.pose.pose.position.z = 0.0

        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        pose_msg.pose.pose.orientation = Quaternion(*quaternion)

        # 아래 covariance 부분은 필요 없을 수도 있음. 나중에 테스트 해 보고 가능하면 빼도 됨
        pose_msg.pose.covariance = [0.0] * 36
        pose_msg.pose.covariance[0] = 0.25
        pose_msg.pose.covariance[7] = 0.25
        pose_msg.pose.covariance[35] = 0.06853892326654787

        self.initpose_pub.publish(pose_msg)


    def publish_goal_pose(self, x, y, yaw):
        ''' yaw is in radian '''

        y = self.map_val - y

        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "world"

        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = 0.0  

        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        pose_msg.pose.orientation = Quaternion(*quaternion)

        self.goalpose_pub.publish(pose_msg)


    def draw_arrow(self, screen, lcolor, tricolor, start, end, trirad):
        '''https://stackoverflow.com/questions/43527894/drawing-arrowheads-which-follow-the-direction-of-the-line-in-pygame'''
        pygame.draw.line(screen, lcolor, start, end, 2)
        rotation = math.degrees(math.atan2(start[1]-end[1], end[0]-start[0]))+90
        pygame.draw.polygon(screen, tricolor, ((end[0]+trirad*math.sin(math.radians(rotation)), end[1]+trirad*math.cos(math.radians(rotation))), (end[0]+trirad*math.sin(math.radians(rotation-120)), end[1]+trirad*math.cos(math.radians(rotation-120))), (end[0]+trirad*math.sin(math.radians(rotation+120)), end[1]+trirad*math.cos(math.radians(rotation+120)))))


    def publish_init_goal_pose(self, ):
        up_x, up_y = self.mouse_up_pos
        down_x, down_y = self.mouse_down_pos

        self.goal_yaw = math.atan2( (self.map_val-up_y) - (self.map_val-down_y), up_x - down_x )

        self.publish_init_pose(*self.robot_pose)
        self.publish_goal_pose(*self.mouse_down_pos, self.goal_yaw)
        print(f'published goal pose: {self.mouse_down_pos}, {self.goal_yaw}') 
    

    def run(self):
        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False

                if self.state in [State.IDLE, State.TRACKING]:
                    if event.type == pygame.MOUSEBUTTONDOWN:
                        if event.button == 1:  # Left mouse button
                            # click_x, click_y = event.pos
                            self.mouse_down_pos = event.pos

                            # 일시정지
                            self.prev_paths = self.searched_paths
                            self.searched_paths.clear()

                            msg = Path()
                            msg.poses = []
                            self.searched_path_pub.publish(msg)

                            # twist_msg = Twist()
                            # twist_msg.linear.x = 0
                            # twist_msg.angular.z = 0
                            # self.twist_pub.publish(twist_msg)

                    elif event.type == pygame.MOUSEBUTTONUP:
                        if event.button == 1:
                            self.mouse_up_pos = event.pos
                            self.publish_init_goal_pose()
                            

            self.screen.blit(self.map_image, self.map_rect)

            if self.state == State.START:
                text_surface = self.state_text.render('START', False, (0, 0, 0))
            elif self.state == State.IDLE:
                text_surface = self.state_text.render('IDLE', False, (0, 0, 0))
            elif self.state == State.TRACKING:
                text_surface = self.state_text.render('TRACKING', False, (0, 0, 0))
            elif self.state == State.TRACKING_LOSS:
                text_surface = self.state_text.render('TRACKING_LOSS', False, (0, 0, 0))
            else:
                text_surface = self.state_text.render('???', False, (0, 0, 0))
            self.screen.blit(text_surface, (0,0))

            self.draw_searched_paths()
            
            for i, p in enumerate(self.past_robot_poses):
                # p = self.past_robot_poses
                pygame.draw.circle(self.screen, (255, 0, 0, max(0, 255-len(self.past_robot_poses))+1+i), (p[0], p[1]), 4)  # 빨간 동그라미 그리기

            if self.state in [State.IDLE, State.TRACKING]:
                self.draw_robot_pose('red')
            else:
                self.draw_robot_pose('grey')

            pygame.display.flip()
            self.clock.tick(30)

    pygame.quit()


if __name__ == "__main__":
    map_image = '/home/cgvlab/catkin_ws/src/convert2d_pose/src/map_0429_2.jpg'
    tracker = RobotTracker(map_image)
    tracker.run()