#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
    D* 算法
    D-star的主要特点就是由目标位置开始向起始位置进行路径搜索，
    当物体由起始位置向目标位置运行过程中，发现路径中存在新的障碍时，
    对于目标位置到新障碍之间的范围内的路径节点，新的障碍是不会影响到其到目标的路径的
    D*算法中，当前点到目标的估算成本并没有体现，类似于Dijkstra算法
    这种搜索更多的是一种由目标位置向四周发散搜索，直到把起始位置纳入搜索范围为止
"""
import os
import sys
import rospy
import math
import cv2
import numpy as np   
from sys import maxsize
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Quaternion, PoseWithCovarianceStamped

class State(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.state = ""
        self.t = "new"
        # 代价函数估计，表示当前State到G的开销估计
        self.h = 0
        # Key Function，该值是优先队列Openlist中的排序依据，K值最小的State位于队列头
        self.k = 0

    def cost(self, target, angle=None):
        if self.state == "#" or target.state == "#":
            # 若存在障碍物，则将这条路径上的代价设置为无穷大
            return maxsize 
        # 若不存在障碍物，则设置欧式距离为代价
        if angle != None:
            if angle > 0.706:
                return (math.sqrt(math.pow((self.x - target.x),2) + math.pow((self.y - target.y),2)))*0.5
            if angle < 0.706:
                return (math.sqrt(math.pow((self.x - target.x),2) + math.pow((self.y - target.y),2)))*2
        else:
            return (math.sqrt(math.pow((self.x - target.x),2) + math.pow((self.y - target.y),2)))
    
    def set_state(self, state):
        """
            S---Start
            G---Goal
            #---Obstacle
            U---Unknow
            .---accessible
            +---path
        """
        if state not in ["S", "#", "U", "G", ".", "+"]:
            return -1
        self.state = state

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
        self.state_map = []
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
            cv2.imwrite("/media/psf/Home/intelligence_Engineering/catkin_ws/src/intelligence_engineering/maps/test.png", self.map)
            print (self.map.shape)
            self.state_map = self.set_state_map()
            print ("Successfully generated a state map with size: ({}, {})".format(self.height, self.width))
            # self.map_sub.unregister()
            self.refresh_map = False    
    
    def set_state_map(self):
        print ("start generate the state map")
        map_list = []
        for i in range(self.height):
            # i = (self.height-1) - i
            tmp = []
            for j in range(self.width):
                if self.map[i][j] == -1:
                    tmp.append(0)
                else:
                    state_point = State(i,j)
                    if self.map[i][j] == 0:
                        state_point.set_state(".")
                    if self.map[i][j] == 100:
                        state_point.set_state("#")
                    tmp.append(state_point)
            # print (len(tmp))
            map_list.append(tmp)
            # print (len(map_list))
            # print (i*self.width + j)
        return map_list
    
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

    
    def get_neighbors(self, target):
        # 获取target的八邻域
        neighbor_list = []
        for i in [-1, 0, 1]:
            for j in [-1, 0, 1]:
                if i == 0 and j == 0:
                    continue
                if target.x + i < 0 or target.x + i >= self.height:
                    continue
                if target.y + j < 0 or target.y + j >= self.width:
                    continue   
                neighbor_list.append(self.state_map[target.x+i][target.y+j])
        return neighbor_list

class DstarPathPlanning(object):
    def __init__(self, map):
        self.map = map
        self.open_list = set()
        self.map_path = []

        self.start_path_plan = False
        self.get_start_pose = False
        self.get_goal_pose = False
        self.goal_pose = PoseStamped()
        self.start_pose = PoseWithCovarianceStamped
        self.world_path = Path()
        self.map_start_point = None
        self.map_goal_point = None
        # 存储由起始点指向目标点的向量
        # 用类似虚拟势场的方式来优化搜寻过程
        self.target_vector = None
        self.last_time = rospy.get_rostime()

        rospy.loginfo("Initilize ROS Subscribers & Publisher")
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
            

    def goal_pose_callback(self, msg):
        # 重置各个参数，并且开始路径搜素
        self.map_path = []
        self.world_path = Path()
        self.open_list = set()
        rospy.loginfo("Receive goal pose: ")
        print (msg)
        self.goal_pose = msg
        self.map_goal_point = self.map.world_to_map(msg.pose.position.x, msg.pose.position.y)
        if self.map_goal_point == [-1, -1]:
            print ("Please set the valid goal pose")
        else:
            print ("Goal pose convert to map: ({}, {})".format(self.map_goal_point[0], self.map_goal_point[1]))
        norm = (pow((self.map_start_point[0]-self.map_goal_point[0]),2) + pow((self.map_start_point[1]-self.map_goal_point[1]), 2))**0.5
        print ("Normalization Value: {}".format(norm))
        print ("Original Potential Vector: {}".format((self.map_start_point[0]-self.map_goal_point[0], self.map_start_point[1]-self.map_goal_point[1])))
        # 以下为归一化之后的势场方向向量
        self.potential_vector = ((self.map_start_point[0]-self.map_goal_point[0])/norm, (self.map_start_point[1]-self.map_goal_point[1])/norm)
        print ("The path direction is {}".format(self.potential_vector))
        self.start_path_plan = True
        self.get_goal_pose = True


    def kmin_point(self):
        if not self.open_list:
            return None
        # 搜索open list中k值最小的点
        min_value = maxsize
        min_point = State(0,0)
        for item in self.open_list:
            if item == 0:
                continue
            elif item.k < min_value:
                # print (item.k)
                min_value = item.k 
                min_point = item
        return min_point
        # min_point = min(self.open_list, key=lambda x: x.k)
    
    def get_kmin(self):
        # 获取open list中k（f）值最小的k
        if not self.open_list:
            return -1
        min_value = maxsize
        min_point = None
        for item in self.open_list:
            if item == 0:
                continue
            elif item.k < min_value:
                min_value = item.k 
                min_point = item
        return min_value
    
    def insert(self, point, h_new):
        # 将新遍历的点添加到open list
        # 同时若有代价更小的路径，则修改open list中点的代价
        if point.t == "new":
            point.k = h_new
        elif point.t == "open":
            point.k = min(point.k, h_new)
        elif point.t == "close":
            point.k = min(point.k, h_new)
        point.h = h_new
        point.t = "open"
        # print ("Insert point ({}, {}) to the open list and its new cost is {}".format(point.x, point.y, point.h))
        self.open_list.add(point)
    
    def remove(self, point):
        if point.t == "open":
            point.t = "close"
            # print ("Remove point ({}, {}) from the open list, and set its state to {}".format(point.x, point.y, point.t))
        self.open_list.remove(point)

    def modify_cost(self, x):
        if x.t == "close":  # 是以一个openlist，通过parent递推整条路径上的cost
            self.insert(x, x.parent.h + x.cost(x.parent))
    
    def process(self):
        # 路径扩充的部分
        # 先读取open list中k值最小的节点
        tmp = self.kmin_point()
        print ("Extend to the point ({}, {})".format(tmp.x, tmp.y))

        if tmp is None:
            print ("Sorry i cant find the target point")

            sys.exit()
        # 读取当前最小k值的点的k值
        k_old = self.get_kmin()
        print ("Current Cost to Goal Pose: {}".format(k_old))
        # 先从open list中将这一个点移除
        self.remove(tmp)

        if k_old < tmp.h:
            for y in self.map.get_neighbors(tmp):
                if y == 0:
                    continue
                norm = (pow(y.x - tmp.x, 2) + pow(y.y - tmp.y, 2))**0.5
                vector = ((y.x - tmp.x)/norm, (y.y - tmp.y)/norm)
                angle = vector[0]*self.potential_vector[0]+vector[1]*self.potential_vector[1]
                # print ("Current Extension direction: {}".format(angle))
                
                if y.h <= k_old and tmp.h > y.h + tmp.cost(y, angle):
                    tmp.parent = y
                    # print ("Update RAISED cost from {} to {}".format(tmp.h, y.h + tmp.cost(y, angle)))
                    tmp.h = y.h + tmp.cost(y, angle)
        elif k_old == tmp.h:
            for y in self.map.get_neighbors(tmp):
                if y == 0:
                    continue
                norm = (pow(y.x - tmp.x, 2) + pow(y.y - tmp.y, 2))**0.5
                vector = ((y.x - tmp.x)/norm, (y.y - tmp.y)/norm)
                angle = vector[0]*self.potential_vector[0]+vector[1]*self.potential_vector[1]
                # print ("Current Extension direction: {}".format(angle))
                if y.t == "new" or (y.parent == tmp and y.h != tmp.h + tmp.cost(y, angle)) or (y.parent != tmp and y.h > tmp.h + tmp.cost(y, angle)):
                    y.parent = tmp
                    self.insert(y, tmp.h+tmp.cost(y, angle))
        else:
            for y in self.map.get_neighbors(tmp):
                if y == 0:
                    continue
                norm = (pow(y.x - tmp.x, 2) + pow(y.y - tmp.y, 2))**0.5
                vector = ((y.x - tmp.x)/norm, (y.y - tmp.y)/norm)
                angle = vector[0]*self.potential_vector[0]+vector[1]*self.potential_vector[1]
                # print ("Current Extension direction: {}".format(angle))
                if y.t == "new" or (y.parent == tmp and y.h != tmp.h + tmp.cost(y, angle)):
                    y.parent = tmp
                    self.insert(y, tmp.h+tmp.cost(y, angle))
                else:
                    if y.parent != tmp and y.h > tmp.h + tmp.cost(y, angle) and y.t == "close" and y.h > k_old:
                        self.insert(y, y.h)
        return self.get_kmin()
    
    def main(self):
        while True:
            # 参数为起始点和终点
            while (self.get_start_pose == False):
                print ("Waiting for the start pose and goal pose")
                rospy.sleep(5)
            while (self.get_goal_pose == False):
                print ("Waiting for the goal pose")
                rospy.sleep(5)
            if self.start_path_plan:
                start_pose = self.map.state_map[self.map_start_point[0]][self.map_start_point[1]]
                goal_pose = self.map.state_map[self.map_goal_point[0]][self.map_goal_point[1]]
                print ("Start Planning path from ({}, {}) to ({},{})".format(goal_pose.x, goal_pose.y, start_pose.x, start_pose.y))
                # 由于D*算法是从终点向当前点扩展的算法，所以第一步先将终点添加进open list
                self.open_list.add(goal_pose)
                while True:
                    self.process()
                    if start_pose.t == "close":
                        break
                    # rospy.sleep(1)
                start_pose.set_state("S")
                tmp = start_pose
                while tmp != goal_pose:
                    # 通过父指针进行回溯，储存整个规划出来的路径
                    tmp = tmp.parent
                    # tmp.set_state("+")
                    tmp_point = (tmp.x, tmp.y)
                    self.map_path.append(tmp_point)
                tmp.set_state("G")
                self.path_publish()
                self.start_path_plan = False
                self.get_start_pose = False
                self.get_goal_pose = False
    
    def path_smooth(self):
        return 0
    
    def path_publish(self):
        print ("Congratz! We Have Found a Path ！")
        for pose in self.map_path:
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
    rospy.init_node("DStarPathPlanner")
    ori_map = Map()
    rospy.sleep(2)
    while (ori_map.refresh_map):
        print ("Generating State Map, Please wait")
        rospy.sleep(2)
    Planner = DstarPathPlanning(ori_map)
    Planner.main()
    rospy.spin()


        
        



        

        
