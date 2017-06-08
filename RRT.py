import math, sys, pygame, random
import numpy as np
from math import *
from pygame import *

class Node(object):
    def __init__(self, point, parent):
        super(Node, self).__init__()
        self.point = point
        self.parent = parent

class RRT():
    def __init__(self, d, bounds, epsilon, nodeNum, collides, tree):
        self.d = d
        self.bounds = bounds
        self.EPSILON = epsilon
        self.NUMNODES = nodeNum
        self.collides = collides
        self.count = 0
        self.nodes = tree

    def dist(self, p1, p2):
        # distance between two points
        sum = 0
        for i in range(self.d):
            sum += math.pow(p1[i]-p2[i],2)

        return sqrt(sum)

    def point_circle_collision(self, p1, p2, radius):
        # check goal area
        distance = self.dist(p1, p2)
        if (distance <= radius):
            return True
        return False

    def step_from_to(self, p1, p2):
        # generate point toward p2
        if self.dist(p1, p2) < self.EPSILON:
            return p2
        else:
            dist = self.dist(p1,p2)
            unit = [a - b for a, b in zip(p2, p1)]
            unit = [self.EPSILON * u / dist for u in unit]
            result = [p1[j] + unit[j] for j in range(len(unit))]
            return result

    def get_random_clear(self):
        # return a random point with collision check
        while True:
            temp = []
            for i in range(self.d):
                temp.append(random.random() * self.bounds[i])
            p = tuple(temp)
            if not self.collides(p):
                return p

    def explore(self, screen, color, GOAL_RADIUS = 10):
        # RRT
        while True:
            self.count = self.count + 1
            if self.count < self.NUMNODES:
                foundNext = False
                while foundNext == False:
                    rand = self.get_random_clear()
                    closestNode = self.nodes[0]
                    for p in self.nodes:
                        if self.dist(p.point, rand) <= self.dist(closestNode.point, rand):
                            newPoint = self.step_from_to(p.point, rand)
                            if self.collides(newPoint) == False:
                                closestNode = p
                                foundNext = True

                newnodePoint = self.step_from_to(closestNode.point, rand)
                self.nodes.append(Node(newnodePoint, closestNode))
                pygame.draw.line(screen, (0, 180, 105), closestNode.point, newnodePoint)
                pygame.display.update()
                #if self.point_circle_collision(newnodePoint, goalPoint.point, GOAL_RADIUS):
                #    goalNode = self.nodes[len(self.nodes) - 1]
                #    return 'goalFound'
            else:
                #print("exploration done")
                return