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

    def rrt_demo(self):
        initPoseSet = False
        initialPoint = RRT.Node(None, None)
        goalPoseSet = False
        goalPoint = RRT.Node(None, None)
        currentState = 'init'

        nodes = []
        self.reset()
        goalNode = RRT.Node(None, None)

        while True:
            # pygame wait for inital and final point
            if currentState == 'init':
                pygame.display.set_caption('Select Starting Point and then Goal Point')
                self.fpsClock.tick(10)
            elif currentState == 'goalFound':
                currNode = goalNode.parent
                pygame.display.set_caption('Goal Reached')
                while currNode.parent != None:
                    pygame.draw.line(self.screen, self.red, currNode.point, currNode.parent.point)
                    currNode = currNode.parent
                optimizePhase = True
            elif currentState == 'optimize':
                self.fpsClock.tick(0.5)
                pass
            elif currentState == 'buildTree':
                tree = RRT.RRT(self.XDIM, self.YDIM, 10, 500, self.rectObs, nodes)
                currentState = tree.explore(self.screen, self.cyan, goalPoint, goalNode, GOAL_RADIUS = 10)

            # handle events
            for e in pygame.event.get():
                if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                    sys.exit("Exiting")
                if e.type == MOUSEBUTTONDOWN:
                    print('mouse down')
                    if currentState == 'init':
                        if initPoseSet == False:
                            nodes = []
                            if self.collides(e.pos) == False:
                                print('initiale point set: ' + str(e.pos))

                                initialPoint = RRT.Node(e.pos, None)
                                nodes.append(initialPoint)  # Start in the center
                                initPoseSet = True
                                pygame.draw.circle(self.screen, self.red, initialPoint.point, 10)
                        elif goalPoseSet == False:
                            print('goal point set: ' + str(e.pos))
                            if self.collides(e.pos) == False:
                                goalPoint = RRT.Node(e.pos, None)
                                goalPoseSet = True
                                pygame.draw.circle(self.screen, self.green, goalPoint.point, 10)
                                currentState = 'buildTree'
                    else:
                        currentState = 'init'
                        initPoseSet = False
                        goalPoseSet = False
                        self.reset()

            pygame.display.update()
            self.fpsClock.tick(10000)



if __name__ == '__main__':
    srt = SRT( 10, 10, 10, 10, 10, 10)
    srt.init_obstacles(1)
    srt.rrt_demo()