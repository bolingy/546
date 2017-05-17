import math
import numpy as np
import Queue as q

class Node(object):
    ''' Each node represent one discrete state in the hybrid system '''
    def __init__(self, j, guard, reset):
        self.j = j
        self.h = 0
        self.g = None # sum of edge costs from start
        self.f = None
        self.parent = None
        self.guard = guard
        self.reset = reset  # a list of r point from
        self.imp = [0] * len(reset)

    def set_h(self, h_in):
        self.h = h_in
        #self.f = self.g + self.h

    def set_g(self, g_in):
        self.g = g_in
        self.f = self.g + self.h

    def set_parent(self, parent):
        self.parent = parent

    def set_imp(self, idx, value):
        self.imp[idx] = value

    def check_goal(self, goal):
        return self.j == goal


# motion planner takes in (S = QX,f = dynamic function,I = init state,F = final state,E = discrete transition)
# Q = discrete set, X = continuous state space
class hyPlanner():
    def __init__(self, system, s_init, s_final):
        self.f = system.F # dynamic function in hybrid system class
        self.E = system.R # reset function in hybrid system class
        self.I = s_init   # [discrete state -- j, continuous state -- x]
        self.F = s_final  # [discrete state -- j, continuous state -- x]
        #self.S # Q*X not yet define
        self.dsList = system.dsList
        # A* varibles
        self.qCount = 0

    def put_to_q(self, input_q, node):
        # put a node to a priority queue
        # qCount for case when nodes share the same f value
        input_q.put((node.f, self.qCount, node))
        self.qCount += 1

    def backtrack_sigma(self, node):
        current = node
        temp = []
        while current is not None:
            temp.append(current)
            current = current.parent
        result = temp
        while not not temp:
            a = temp.pop()
            print(a.j, ' ', round(a.g, 3))
        return result

    def add_to_open(self, child, open_list, close_list):
        # add child to open_list according to members in open and close list
        temp_list = []
        in_open = False
        in_close = False
        # check exist in open list
        while not open_list.empty():
            n = open_list.get()
            if n[2].j != child.j:
                temp_list.append(n[2])
            elif n[2].g < child.g:
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
                if c.j == child.j and c.g < child.g:
                    close_list.remove(c)
                    self.put_to_q(open_list, child)
                    in_close = True
        # not in open or close
        if not in_open and not in_close:
            self.put_to_q(open_list, child)

    def A_star(self):
        '''this is an iterative deepening a* algorithm which serve as the Guide process for
        the hybrid planner'''
        open_list = q.PriorityQueue()
        close_list = []
        firstNode = self.dsList[self.I[0]]
        firstNode.set_g(0) 
        firstNode.set_parent(None)
        self.put_to_q(open_list, firstNode)
        while not open_list.empty():
            current = open_list.get()
            close_list.append(current[2])
            # check goal state
            if current[2].check_goal(self.F[0]):
                print 'in goal state!'
                return self.backtrack_sigma(current[2])
            # for each discrete transition in
            idx = 0
            for s in current[2].reset:
                child = self.dsList[s]
                child.set_parent(current[2])
                child.set_g(current[2].g + current[2].imp[idx])
                # add child to open_list according to members in open and close list
                self.add_to_open(child, open_list, close_list)
                idx += 1

        print 'Fail to find a path to goal state'
        return
