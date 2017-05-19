import math, sys, pygame, random
from math import *
from pygame import *

class Node(object):
    def __init__(self, point, parent):
        super(Node, self).__init__()
        self.point = point
        self.parent = parent

class RRT():
    def __init__(self, xd, yd, epsilon, nodeNum, rectObs, tree):
        self.XDIM = xd
        self.YDIM = yd
        self.EPSILON = epsilon
        self.NUMNODES = nodeNum
        self.rectObs = rectObs
        self.count = 0
        self.nodes = tree

    def dist(self, p1, p2):
        # distance between two points
        return sqrt((p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]))

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
            theta = atan2(p2[1] - p1[1], p2[0] - p1[0])
            return p1[0] + self.EPSILON * cos(theta), p1[1] + self.EPSILON * sin(theta)

    def collides(self, p):
        # check if point collides with the obstacle
        for rect in self.rectObs:
            if rect.collidepoint(p) == True:
                return True
        return False

    def get_random_clear(self):
        # return a random point with collision check
        while True:
            p = random.random() * self.XDIM, random.random() * self.YDIM
            noCollision = self.collides(p)
            if noCollision == False:
                return p

    def explore(self, screen, color, goalPoint, goalNode, GOAL_RADIUS = 10):
        # RRT
        while True:
            self.count = self.count + 1
            pygame.display.set_caption('Performing RRT')
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
                pygame.draw.line(screen, color, closestNode.point, newnodePoint)
                if self.point_circle_collision(newnodePoint, goalPoint.point, GOAL_RADIUS):
                    goalNode = self.nodes[len(self.nodes) - 1]
                    return 'goalFound'
            else:
                print("exploration done")
                return;