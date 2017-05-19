import math, sys, pygame, random
from math import *
from pygame import *
from scipy import spatial
import RRT

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
        self.Vt = [] # list of trees
        self.Et = []
        self.Q = []
        self.Ec = []
        #self.kdTree = spatial.KDTree(self.Q) # for nearest neighbor
        # pygame
        pygame.init()
        self.XDIM = 720
        self.YDIM = 500
        self.windowSize = [self.XDIM, self.YDIM]
        self.GOAL_RADIUS = 10
        self.MIN_DISTANCE_TO_ADD = 1.0
        self.white = 255, 255, 255
        self.black = 0, 0, 0
        self.red = 255, 0, 0
        self.blue = 0, 255, 0
        self.green = 0, 0, 255
        self.cyan = 0, 180, 105
        self.fpsClock = pygame.time.Clock()
        self.screen = pygame.display.set_mode(self.windowSize)
        self.rectObs = []
        self.GOAL_RADIUS = 10

    def init_obstacles(self, configNum):
        # initialized the obstacle, configNum select pre-set obstacles
        self.rectObs = []
        print("config " + str(configNum))
        if (configNum == 0):
            self.rectObs.append(pygame.Rect((self.XDIM / 2.0 - 50, self.YDIM / 2.0 - 100), (100, 200)))
        if (configNum == 1):
            self.rectObs.append(pygame.Rect((100, 50), (200, 150)))
            self.rectObs.append(pygame.Rect((400, 200), (200, 100)))
            self.rectObs.append(pygame.Rect((20, 100), (330, 100)))
        if (configNum == 2):
            self.rectObs.append(pygame.Rect((100, 50), (200, 150)))
        if (configNum == 3):
            self.rectObs.append(pygame.Rect((100, 50), (200, 150)))

        for rect in self.rectObs:
            pygame.draw.rect(self.screen, self.black, rect)

    def reset(self):
        # set pygame and obstacles
        self.screen.fill(self.white)
        self.init_obstacles(1)

    def collides(self, p):
        # check if point collides with the obstacle
        for rect in self.rectObs:
            if rect.collidepoint(p) == True:
                return True
        return False

    def rrt_explore(self, initPose):
        nodes = []
        initialPoint = RRT.Node(initPose, None)
        nodes.append(initialPoint)
        pygame.draw.circle(self.screen, self.red, initialPoint.point, self.GOAL_RADIUS)
        tree = RRT.RRT(self.XDIM, self.YDIM, 5, self.m, self.rectObs, nodes)
        tree.explore(self.screen, self.cyan, GOAL_RADIUS=10)
        return nodes

    def plan(self):
        # perform SRT search
        # clean storage
        self.Vt = []    #trees
        self.Et = []    #edge bwt trees
        self.Q = []     #tree centroids
        self.Ec = []    #
        #
        while len(self.Vt) < self.K:
            


if __name__ == '__main__':
    srt = SRT( 10, 200, 10, 10, 10, 10)
    srt.init_obstacles(1)
    srt.reset()
    srt.rrt_explore((133,390))
    srt.rrt_explore((200, 200))
    while True:
        for e in pygame.event.get():
            if e.type == QUIT:
                sys.exit('Exiting')
        pygame.display.update()
        srt.fpsClock.tick(10000)