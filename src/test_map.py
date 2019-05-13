#! /usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np 


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

    def cost(self, target):
        if self.state == "#" or target.state == "#":
            # 若存在障碍物，则将这条路径上的代价设置为无穷大
            return maxsize 
        # 若不存在障碍物，则设置欧式距离为代价
        return math.sqrt(math.pow((self.x - target.x),2) + math.pow((self.y - target.y),2))
    
    def set_state(self, state):
        """
            S---Start
            G---Goal
            #---Obstacle
            U---Unknow
            .---accessible
        """
        if state not in ["S", "#", "U", "G", "."]:
            return -1
        self.state = state



def init_state_map(map_):
    map_ = np.array(map_)
    height = map_.shape[0]
    width = map_.shape[1]
    print (height, width)
    map_list = []
    for i in range(height):
        tmp = []
        for j in range(width):
            state_point = State(i,j)
            if map_[i][j] == 0:
                state_point.set_state(".")
            if map_[i][j] == -1:
                state_point.set_state("U")
            if map_[i][j] == 100:
                state_point.set_state("#")
            tmp.append(state_point.state)
        map_list.append(tmp)
    print (len(map_list))
    print (map_list)

if __name__ == '__main__':
    map_ = [[100,0,0,0,0,-1,0,0,0,0,0],
                [0,0,0,0,0,0,0,0,0,0,0],
                [0,0,0,0,0,0,0,0,0,0,0],
                [0,0,0,0,0,0,0,0,0,0,0],
                [0,0,0,0,0,0,0,0,0,0,0],
                [0,0,0,0,0,100,0,0,0,0,0],
                [0,0,0,0,0,0,0,0,0,0,0],
                [0,0,0,0,0,0,0,0,0,0,0],
                [0,0,0,0,0,0,0,0,0,0,0]]
    init_state_map(map_)