import Queue as q
import math

class Node(object):
    # node for each state, h the estimate cost, [x, y] is coordinate
    def __init__(self, pose, h, num = None):
        self.pose = pose
        self.num = num
        self.h = h
        self.g = None
        self.f = None
        self.parent = None

    def set_g(self, g_in):
        self.g = g_in
        self.f = self.g + self.h

    def set_parent(self, parent):
        self.parent = parent

    def check_goal(self, goal):
        return self.pose == goal


def cal_distance(point_a, point_b):
    # calculate direct distance from point a to b
    return math.sqrt((point_a[0] - point_b[0]) ** 2 + (point_a[1] - point_b[1]) ** 2)

class AStar():
    def __init__(self, start, goal, poseList, edgeList):
        self.start = start  # start num
        self.goal = goal    # goal num
        self.poseList = poseList
        self.edgeList = edgeList
        self.qCount = 0
        self.startPose = poseList[self.start]
        self.goalPose = poseList[self.goal]

    def put_to_q(self, input_q, node):
        # put a node to a priority queue
        # qCount for case when nodes share the same f value
        input_q.put((node.f, self.qCount, node))
        self.qCount += 1

    def backtrack(self, node):
        # print the shortest path and the cost
        current = node
        temp = []
        while current is not None:
            temp.append(current)
            current = current.parent
        result = []
        while not not temp:
            a = temp.pop()
            result.append(a.num)
            print(str(a.pose) + '  ' + str(round(a.g, 3)))
        return result

    def add_to_open(self, child, open_list, close_list):
        # add child to open_list according to members in open and close list
        temp_list = []
        in_open = False
        in_close = False
        # check exist in open list
        while not open_list.empty():
            n = open_list.get()
            if n[2].pose != child.pose:
                temp_list.append(n[2])
            elif n[2].g > child.g:
                temp_list.append(child)
                in_open = True
            else:
                temp_list.append(n[2])
                in_open = True
        # restore open_list, not not a list return implicit boolean, True while not empty
        while not not temp_list:
            t_node = temp_list.pop()
            self.put_to_q(open_list, t_node)
        # check exist in close list
        if not in_open:
            for c in close_list:
                if c.pose == child.pose and c.g > child.g:
                    close_list.remove(c)
                    self.put_to_q(open_list, child)
                    in_close = True
        # not in open or close
        if not in_open and not in_close:
            self.put_to_q(open_list, child)

    def cal_h(self, point):
        # calculate direct distance from point to goal
        c = math.sqrt((self.goalPose[0] - point[0]) ** 2 + (self.goalPose[1] - point[1]) ** 2)
        return c

    def plan(self):
        open_list = q.PriorityQueue()
        close_list = []
        #firstNode = self.nodeList[self.start]
        firstNode = Node(self.startPose, self.cal_h(self.startPose), self.start)
        firstNode.set_g(0)
        firstNode.set_parent(None)
        self.put_to_q(open_list, firstNode)
        while not open_list.empty():
            current = open_list.get()
            close_list.append(current[2])
            # check goal state
            if current[2].check_goal(self.goalPose):
                print 'in goal state!'
                return self.backtrack(current[2])
            # generate successors
            successors = []
            for e in self.edgeList:
                if e[0] == current[2].num:
                    successors.append(e[1])
                elif e[1] == current[2].num:
                    successors.append(e[0])

            # for each discrete transition in
            for s in successors:
                #child = self.nodeList[s]
                pose = self.poseList[s]
                child = Node(pose, self.cal_h(pose), s)
                child.set_parent(current[2])
                child.set_g(current[2].g + cal_distance(child.pose, current[2].pose))
                # add child to open_list according to members in open and close list
                self.add_to_open(child, open_list, close_list)

        print 'Fail to find a path to goal state'
        return



'''if __name__ == '__main__':
    edges = [(0,1), (0,2), (1,3), (2,3), (3,4)]
    poses = [(1,1),(2,4),(2,0),(3,1),(4,3)]
    planner = AStar(0,4,poses,edges)
    planner.plan()'''