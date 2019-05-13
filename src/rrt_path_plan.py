#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Rapidly-exploring Random Tree
"""
import os
import sys
import rospy
import math
import cv2
import random
import numpy as np   
import copy
from sys import maxsize
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Quaternion, PoseWithCovarianceStamped

class Node(object):
    """
    RRT Node
    """

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


class Map(object):
    def __init__(self):
        self.refresh_map = True 
        self.height = None
        self.width = None
        self.origin_x = None
        self.origin_y = None
        self.resolution = None
        self.potential_vector = None
        self.map = None
        self.obstacle_list = []
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.mapCallback)
    
    def mapCallback(self, msg):
        # @TODO
        # 在每次接受地图的时候将接收到的地图与当前地图进行比较，判断是否需要更新地图
        # 这一部分可以添加避障功能，由amcl实时更新地图
        if self.refresh_map:
            # 使用一个参数来控制地图的刷新
            # 接收来自／map话题的消息，并储存称为我们需要的格式的地图
            self.height = msg.info.height
            self.width = msg.info.width
            self.resolution = msg.info.resolution
            self.origin_x = msg.info.origin.position.x
            self.origin_y = msg.info.origin.position.y
            self.map_origin_x = -self.origin_y / self.resolution
            self.map_origin_y = -self.origin_x / self.resolution
            
            print ("Received map with size of: ({}, {}), header: {}".format(self.width, self.height, msg.header))
            print ("----------------------------------------------------")
            print (msg.info)
            print ("----------------------------------------------------")
            # 初始化地图
            self.map = np.array(msg.data, dtype=np.int8).reshape((self.height, self.width))
            cv2.imwrite("/media/psf/Home/intelligence_Engineering/catkin_ws/src/intelligence_engineering/maps/map_test.png", self.map)
            print (self.map.shape)
            self.set_obstacle_list()
            print ("Successfully generated a state map with size: ({}, {})".format(self.height, self.width))
            # self.map_sub.unregister()
            self.refresh_map = False    
    
    def set_obstacle_list(self):
        for i in range(self.height):
            for j in range(self.width):
                if self.map[i][j] == 100:
                    tmp = [i, j, 1]
                    self.obstacle_list.append(tmp)
        print (len(self.obstacle_list))
    
    def map_to_world(self, map_x, map_y):
        if self.origin_x and self.origin_y and self.resolution:
            world_x = (map_y - self.map_origin_y)*self.resolution
            world_y = (map_x - self.map_origin_x)*self.resolution
            return [world_x, world_y]
        else:
            rospy.loginfo("No map data received! Please check the map server")
            return [-1, -1]
    
    def world_to_map(self, world_x, world_y):
        if world_x < self.origin_x or world_y < self.origin_y:
            rospy.loginfo("Invalid world coordinate")
            return [-1, -1]
        
        else:
            map_x = int(self.map_origin_x + int(world_y / self.resolution))
            map_y = int(self.map_origin_y + int(world_x / self.resolution))
            if map_x > self.width or map_y > self.height:
                return [-1, -1]
            else:
                return [map_x, map_y]

class RRT(object):
    """
    Class for RRT Planning
    """

    def __init__(self, map):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:random sampling Area [min,max]

        """
        self.map = map
        self.start = None
        self.end = None
        self.min_rand = 0
        self.path = []
        self.worlf_path = Path()
        self.max_x_rand = map.height
        self.max_y_rand = map.width
        self.expandDis = 1.0
        self.goalSampleRate = 0.05  # 选择终点的概率是0.05
        self.maxIter = 500
        self.obstacleList = map.obstacle_list
        self.start_path_plan = False
        self.get_goal_pose = False
        self.get_start_pose = False

        self.path_pub = rospy.Publisher("/path", Path, queue_size=15)
        self.start_sub = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.start_pose_callback)
        self.end_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_pose_callback)

    def start_pose_callback(self, msg):
        rospy.loginfo("Receive start pose: ")
        print (msg)
        self.start_pose = msg
        self.map_start_point = self.map.world_to_map(msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self.map_start_point == [-1, -1]:
            print ("Please set the valid start pose")
        else:
            print ("Start pose convert to map: ({}, {})".format(self.map_start_point[0], self.map_start_point[1]))
        self.get_start_pose = True
        self.start = Node(self.map_start_point[0], self.map_start_point[1])
        self.node_list = [self.start]

    def goal_pose_callback(self, msg):
        # 重置各个参数，并且开始路径搜素
        rospy.loginfo("Receive goal pose: ")
        print (msg)
        self.goal_pose = msg
        self.map_goal_point = self.map.world_to_map(msg.pose.position.x, msg.pose.position.y)
        if self.map_goal_point == [-1, -1]:
            print ("Please set the valid goal pose")
        else:
            print ("Goal pose convert to map: ({}, {})".format(self.map_goal_point[0], self.map_goal_point[1]))
        self.start_path_plan = True
        self.get_goal_pose = True
        self.end = Node(self.map_goal_point[0], self.map_goal_point[1])

    def random_node(self):
        """
        产生随机节点
        :return:
        """
        node_x = random.uniform(self.min_rand, self.max_x_rand)
        node_y = random.uniform(self.min_rand, self.max_y_rand)
        node = [node_x, node_y]

        return node

    @staticmethod
    def get_nearest_list_index(node_list, rnd):
        """
        获取当前node list中里随机生成的点最近的一个node
        :param node_list:
        :param rnd:
        :return:
        """
        d_list = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1]) ** 2 for node in node_list]
        min_index = d_list.index(min(d_list))
        return min_index

    @staticmethod
    def collision_check(new_node, obstacle_list):
        a = 1
        for (ox, oy, size) in obstacle_list:
            dx = ox - new_node.x
            dy = oy - new_node.y
            d = math.sqrt(dx * dx + dy * dy)
            if d <= size:
                a = 0  # collision
        return a  # safe

    def planning(self):
        """
        Path planning

        animation: flag for animation on or off
        """

        while True:
            if self.start_path_plan == True:
                # Random Sampling
                if random.random() > self.goalSampleRate:
                    # 设定足够小的概率，直接随机到目标节点
                    # 此处设定的是5%的概率
                    rnd = self.random_node()
                else:
                    rnd = [self.end.x, self.end.y]

                # Find nearest node
                min_index = self.get_nearest_list_index(self.node_list, rnd)
                # print(min_index)

                # expand tree
                nearest_node = self.node_list[min_index]

                # 返回弧度制
                theta = math.atan2(rnd[1] - nearest_node.y, rnd[0] - nearest_node.x)

                new_node = copy.deepcopy(nearest_node)
                new_node.x += self.expandDis * math.cos(theta)
                new_node.y += self.expandDis * math.sin(theta)
                new_node.parent = min_index

                if not self.collision_check(new_node, self.obstacleList):
                    continue

                self.node_list.append(new_node)

                # check goal
                dx = new_node.x - self.end.x
                dy = new_node.y - self.end.y
                d = math.sqrt(dx * dx + dy * dy)
                if d <= self.expandDis:
                    print ("Congratz! We Have Found a Path ！")
                    break

            elif self.get_start_pose == False:
                print ("Waiting for the start pose and goal pose")
                rospy.sleep(5)
            elif self.get_goal_pose == False:
                print ("Waiting for the goal pose")
                rospy.sleep(5)

        self.path = [[self.end.x, self.end.y]]
        last_index = len(self.node_list) - 1
        while self.node_list[last_index].parent is not None:
            node = self.node_list[last_index]
            self.path.append([node.x, node.y])
            last_index = node.parent
        self.path.append([self.start.x, self.start.y])
        self.map_path_publish()
    
    def map_path_publish(self):
        for pose in self.path:
            current_time = rospy.get_rostime()
            current_pose = PoseStamped()
            current_pose.pose.position.x, current_pose.pose.position.y = self.map.map_to_world(pose[0], pose[1])
            current_pose.pose.orientation.w = 1.0
            # @TODO
            # 添加每个点的朝向信息，由起点倒退出来
            self.world_path.header.stamp == current_time
            self.world_path.header.frame_id = "map"
            self.world_path.poses.append(current_pose)
            # print (self.world_path)
            self.path_pub.publish(self.world_path)
            self.last_time = current_time
            rospy.sleep(.1)
        return 0

if __name__ == '__main__':
    rospy.init_node("rrtPathPlanner")
    ori_map = Map()
    rospy.sleep(2)
    while (ori_map.refresh_map):
        print ("Waiting for map server")
        rospy.sleep(1)
    Planner = RRT(ori_map)
    Planner.planning()
    rospy.spin()
    
