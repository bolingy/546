import math, sys, pygame, random
from math import *
from pygame import *

class Node(object):
    def __init__(self, point, parent):
        super(Node, self).__init__()
        self.point = point
        self.parent = parent

class RRT():
    def __init__(self, xd, yd, epsilon, nodeNum):
        self.XDIM = xd
        self.YDIM = yd
        self.windowSize = [XDIM, YDIM]
        self.EPSILON = epsilon
        self.NUMNODES = nodeNum
        self.GOAL_RADIUS = 10
        self.MIN_DISTANCE_TO_ADD = 1.0
        self.white = 255, 255, 255
        self.black = 0, 0, 0
        self.red = 255, 0, 0
        self.blue = 0, 255, 0
        self.green = 0, 0, 255
        self.cyan = 0, 180, 105