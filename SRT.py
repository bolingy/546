import math, sys, pygame, random
from math import *
from pygame import *

'''
K: number of milestones(tress)
m: --------- configurations(points per tree)
nc: --------- closest-neighbor milestones
nr: --------- random-neighbor milestones
np: --------- close pairs for straight-line planner
ni: --------- iterations of tree-connection
'''
class SRT():
    def __init__(self, K, m, nc, nr, np, ni):
        self.K = K
        self.m = m
        self.nc = nc
        self.nr = nr
        self.np = np
        self.ni = ni

    