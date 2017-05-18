import math, sys, pygame, random
from math import *
from pygame import *


class Node(object):
    def __init__(self, point, parent):
        super(Node, self).__init__()
        self.point = point
        self.parent = parent

# Algorithm Constant
XDIM = 720
YDIM = 500
windowSize = [XDIM, YDIM]
EPSILON = 10.0
NUMNODES = 5000

# pygame constant
GAME_LEVEL = 1
GOAL_RADIUS = 10
MIN_DISTANCE_TO_ADD = 1.0
white = 255, 255, 255
black = 0, 0, 0
red = 255, 0, 0
blue = 0, 255, 0
green = 0, 0, 255
cyan = 0, 180, 105

# pygame setup
pygame.init()
fpsClock = pygame.time.Clock()
screen = pygame.display.set_mode(windowSize)
count = 0
rectObs = []

# distance between two points
def dist(p1, p2):
    return sqrt((p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]))

# check goal area
def point_circle_collision(p1, p2, radius):
    distance = dist(p1, p2)
    if (distance <= radius):
        return True
    return False

# generate point toward p2
def step_from_to(p1, p2):
    if dist(p1, p2) < EPSILON:
        return p2
    else:
        theta = atan2(p2[1] - p1[1], p2[0] - p1[0])
        return p1[0] + EPSILON * cos(theta), p1[1] + EPSILON * sin(theta)

# check if point collides with the obstacle
def collides(p):
    for rect in rectObs:
        if rect.collidepoint(p) == True:
            return True
    return False

# return a random point with collision check
def get_random_clear():
    while True:
        p = random.random() * XDIM, random.random() * YDIM
        noCollision = collides(p)
        if noCollision == False:
            return p

# initialized the obstacle
def init_obstacles(configNum):
    global rectObs
    rectObs = []
    print("config " + str(configNum))
    if (configNum == 0):
        rectObs.append(pygame.Rect((XDIM / 2.0 - 50, YDIM / 2.0 - 100), (100, 200)))
    if (configNum == 1):
        rectObs.append(pygame.Rect((100, 50), (200, 150)))
        rectObs.append(pygame.Rect((400, 200), (200, 100)))
        rectObs.append(pygame.Rect((20, 100), (330, 100)))
    if (configNum == 2):
        rectObs.append(pygame.Rect((100, 50), (200, 150)))
    if (configNum == 3):
        rectObs.append(pygame.Rect((100, 50), (200, 150)))

    for rect in rectObs:
        pygame.draw.rect(screen, black, rect)

# set pygame and obstacles
def reset():
    global count
    screen.fill(white)
    init_obstacles(GAME_LEVEL)
    count = 0


def main():
    global count

    initPoseSet = False
    initialPoint = Node(None, None)
    goalPoseSet = False
    goalPoint = Node(None, None)
    currentState = 'init'

    nodes = []
    reset()

    while True:
        #pygame wait for inital and final point
        if currentState == 'init':
            pygame.display.set_caption('Select Starting Point and then Goal Point')
            fpsClock.tick(10)
        elif currentState == 'goalFound':
            currNode = goalNode.parent
            pygame.display.set_caption('Goal Reached')
            while currNode.parent != None:
                pygame.draw.line(screen, red, currNode.point, currNode.parent.point)
                currNode = currNode.parent
            optimizePhase = True
        elif currentState == 'optimize':
            fpsClock.tick(0.5)
            pass
        elif currentState == 'buildTree':
            # RRT
            count = count + 1
            pygame.display.set_caption('Performing RRT')
            if count < NUMNODES:
                foundNext = False
                while foundNext == False:
                    rand = get_random_clear()
                    closestNode = nodes[0]
                    for p in nodes:
                        if dist(p.point, rand) <= dist(closestNode.point, rand):
                            newPoint = step_from_to(p.point, rand)
                            if collides(newPoint) == False:
                                closestNode = p
                                foundNext = True

                newnodePoint = step_from_to(closestNode.point, rand)
                nodes.append(Node(newnodePoint, closestNode))
                pygame.draw.line(screen, cyan, closestNode.point, newnodePoint)

                if point_circle_collision(newnodePoint, goalPoint.point, GOAL_RADIUS):
                    currentState = 'goalFound'
                    goalNode = nodes[len(nodes) - 1]

            else:
                print("Ran out of nodes... :(")
                return;

        # handle events
        for e in pygame.event.get():
            if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                sys.exit("Exiting")
            if e.type == MOUSEBUTTONDOWN:
                print('mouse down')
                if currentState == 'init':
                    if initPoseSet == False:
                        nodes = []
                        if collides(e.pos) == False:
                            print('initiale point set: ' + str(e.pos))

                            initialPoint = Node(e .pos, None)
                            nodes.append(initialPoint)  # Start in the center
                            initPoseSet = True
                            pygame.draw.circle(screen, red, initialPoint.point, GOAL_RADIUS)
                    elif goalPoseSet == False:
                        print('goal point set: ' + str(e.pos))
                        if collides(e.pos) == False:
                            goalPoint = Node(e.pos, None)
                            goalPoseSet = True
                            pygame.draw.circle(screen, green, goalPoint.point, GOAL_RADIUS)
                            currentState = 'buildTree'
                else:
                    currentState = 'init'
                    initPoseSet = False
                    goalPoseSet = False
                    reset()

        pygame.display.update()
        fpsClock.tick(10000)


if __name__ == '__main__':
    main()







