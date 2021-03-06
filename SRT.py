import math, sys, pygame, random
import numpy as np
import AStar
from math import *
from pygame import *
from scipy import spatial
import RRT

def cast_tuple(tp):
    temp = []
    for v in tp:
       temp.append(int(v))
    return tuple(temp)


def dist( p1, p2):
    # distance between two points
    sum = 0
    for i in range(len(p1)):
        sum += math.pow(p1[i] - p2[i], 2)

    return sqrt(sum)

def point_circle_collision(p1, p2, radius):
    distance = dist(p1, p2)
    if (distance <= radius):
        return True
    return False

def text_to_screen(screen, text, point, size=40, color = (200,000,000)):
    myfont = pygame.font.SysFont("monospace", size)
    # render text
    label = myfont.render(str(text), 1, color)
    screen.blit(label, point)

'''
K: number of milestones(tress)
m: --------- configurations(points per tree)
nc: --------- closest-neighbor milestones
nr: --------- random-neighbor milestones
np: --------- close pairs for straight-line planner
ni: --------- iterations of tree-connection
'''
class SRT():
    def __init__(self, d, K, m, nc, nr, np, ni, obs_check, stepSize, b_list):
        self.d = d   # dimension
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
        self.En = []
        self.upBound = b_list
        self.stepSize = stepSize
        self.obs_check = obs_check
        #self.kdTree = spatial.KDTree(self.Q) # for nearest neighbor
        # pygame
        pygame.init()
        self.demo = False
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

    def set_upper_bound(self, b_list):
        self.upBound = b_list

    def init_obstacles(self, configNum):
        # initialized the obstacle, configNum select pre-set obstacles
        self.demo = True
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
            self.rectObs.append(pygame.Rect((20, 10), (200, 300)))
            self.rectObs.append(pygame.Rect((200, 50), (200, 100)))
            self.rectObs.append(pygame.Rect((500, 300), (200, 150)))
            #self.rectObs.append(pygame.Rect((60, 80), (200, 300)))
        if (configNum == 4):
            self.rectObs.append(pygame.Rect((100, 10), (200, 100)))
            self.rectObs.append(pygame.Rect((100, 300), (200, 150)))
            self.rectObs.append(pygame.Rect((550, 50), (200, 150)))

        for rect in self.rectObs:
            pygame.draw.rect(self.screen, self.black, rect)

    def reset(self, col):
        # set pygame and obstacles
        self.screen.fill(self.white)
        self.init_obstacles(col)

    def collides(self, p):
        # check if point collides with the obstacle
        if self.demo:
            for rect in self.rectObs:
                if rect.collidepoint(p) == True:
                    return True
        else:
            if len(p) != self.d:
                print 'point has incorrect dimention'
            else:
                return self.obs_check(p)

        return False

    def get_random_clear(self):
        # return a random point with collision check
        while True:
            #p = random.random() * self.XDIM, random.random() * self.YDIM
            #noCollision = self.collides(p)
            #if noCollision == False:
            #    return p
            temp = []
            for i in range(self.d):
                temp.append(random.random() * self.upBound[i])
            p = tuple(temp)
            if not self.collides(p):
                return p

    def rrt_explore(self, initPose):
        nodes = []
        initialPoint = RRT.Node(initPose, None)
        nodes.append(initialPoint)
        if self.demo:
            pygame.draw.circle(self.screen, self.red, initialPoint.point, self.GOAL_RADIUS)
        tree = RRT.RRT(self.d, self.upBound, self.stepSize, self.m, self.collides, nodes)
        tree.explore(self.screen, self.cyan, GOAL_RADIUS=10)
        return nodes

    def arrangeTuple(self, a, b):
        # edge index management
        if (a < b):
            tp = (a, b)
        else:
            tp = (b, a)
        return tp

    def lineCollision(self, p1, p2):
        step = 20
        difList = []
        for i in range(self.d):
            difList.append((p1[i] - p2[i])/step)
        newPoint = list(p2)
        for i in range(1,step):
            newPoint = [newPoint[j] + difList[j] for j in range(len(newPoint))]
            p = cast_tuple(tuple(newPoint))
            #if self.demo:
                #pygame.draw.circle(self.screen, self.green, p, self.GOAL_RADIUS)
            if self.collides(p):
                return True
        # not collision
        return False

    def checkEdgeIden(self,e1, e2):
        result = False
        if e1 == e2:
            result = True
        elif e1[0] == e2[1] and e1[1] == e2[0]:
            result = True
        return result

    def explorationScore(self):
        p1 = cast_tuple(self.get_random_clear())
        count = 0
        for p2 in self.Q:
            if point_circle_collision(p1, p2, 100):
                count+=1
        if self.demo:
            pygame.draw.circle(self.screen, self.blue, p1, 100, 2)
        text_to_screen(self.screen, count, p1)
        return count

    def backTrack(self, currNode):
        result = [currNode]
        while currNode.parent != None:
            if self.demo:
                pygame.draw.line(self.screen, self.red, currNode.point, currNode.parent.point)
            currNode = currNode.parent
            result.append(currNode)

        return result

    def plan(self, init, final):
        # perform SRT search
        # clean storage
        self.Vt = []    #trees
        self.Et = []    #edge bwt trees
        self.Q = []     #tree centroids
        self.Ec = []    #candidate edges between trees
        self.En = []
        # generate T_init and T_final
        Tinit = self.rrt_explore(init)
        Tfinal = self.rrt_explore(final)
        if self.demo:
            pygame.draw.circle(self.screen, self.green, init, self.GOAL_RADIUS)
            pygame.draw.circle(self.screen, self.blue, final, self.GOAL_RADIUS)
        self.Vt.append(Tinit), self.Vt.append(Tfinal)
        self.Q.append(init), self.Q.append(final)
        # generate random trees
        while len(self.Vt) < self.K:
            # build a rrt in random config
            qT = cast_tuple(self.get_random_clear())
            T = self.rrt_explore(qT)
            self.Vt.append(T)
            self.Q.append(qT)
        # find nc closest qt
        kdtree = spatial.KDTree(self.Q)
        for qT in self.Q:
            # nc closest trees
            distance,index = kdtree.query(qT, k=self.nc)
            curIdx = index[0]
            # remove first member (current root)
            distance = np.delete(distance,0,0)
            index = np.delete(index, 0, 0)
            closeIdx = index
            # nr random tress
            randomIdx = []
            while len(randomIdx) < self.nr:
                new = random.randint(0, self.K-1)
                if (not new in randomIdx) and new != curIdx:
                    randomIdx.append(new)
            # store edges
            for i in closeIdx:
                tp = self.arrangeTuple(curIdx, i)
                if not tp in self.Ec:
                    self.Ec.append(tp)
            for i in randomIdx:
                tp = self.arrangeTuple(curIdx, i)
                if not tp in self.Ec:
                    self.Ec.append(tp)

        # for all edge in Ec find
        for edge in self.Ec:
            tree1 = self.Vt[edge[0]]
            tree2 = self.Vt[edge[1]]
            # find closest configurations
            minDis = float("inf")
            for n1 in tree1:
                for n2 in tree2:
                    distance = dist(n1.point, n2.point)
                    if (not self.lineCollision(n1.point,n2.point)) and distance < minDis:
                        minDis = distance
                        minPair = (n1, n2)
            # store tree edge
            if minDis != float("inf"):
                self.Et.append(edge)
                self.En.append(minPair)
        print 'done finding tree edges'
        print 'all tree connection:\n', self.Et
        print self.Ec
        # create pose list
        treeConnecter = AStar.AStar(0,1,self.Q, self.Et)
        treePath = treeConnecter.plan()
        print 'tree path: ', treePath
        # find tree connection
        leafEdges = []
        for i in range(1,len(treePath)):
            connection = (treePath[i],treePath[i-1])
            print connection
            print self.Et[0]
            j = 0
            while not self.checkEdgeIden(connection, self.Et[j]):
                #print 'compare ', connection, ' ', self.Et[j]
                j += 1
            #print 'edge idx is ', j
            leafEdges.append(self.En[j])
        #print leafEdges
        tempList = []
        trj = []
        for e in leafEdges:
            if self.demo:
                pygame.draw.line(self.screen, self.red, e[0].point, e[1].point)
            #if not e[0] in tempList:
            #    tempList.append(e[0])

            trj+=list(reversed(self.backTrack(e[0])))
            trj+=self.backTrack(e[1])

            #if not e[1] in tempList:
            #    tempList.append(e[1])

        #for node in tempList:
        #    currNode = node
        #    while currNode.parent != None:
        #        if self.demo:
        #            pygame.draw.line(self.screen, self.red, currNode.point, currNode.parent.point)
        #        currNode = currNode.parent

        return trj, self.explorationScore()




if __name__ == '__main__':
    #         d, K, m, nc, nr, np, ni, stepSize
    srt = SRT(2, 25, 15, 5, 5, 10, 10, 'dummy', 15, [720,500])
    srt.reset(3)
    srt.plan((338, 19),(233, 159))
    while True:
        for e in pygame.event.get():
            if e.type == QUIT:
                sys.exit('Exiting')
        pygame.display.update()
        srt.fpsClock.tick(10000)