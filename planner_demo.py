from random import randint
import matplotlib.pyplot as plt
import numpy as np
#import pandas
from mpl_toolkits.mplot3d import Axes3D
import math
from systems import Hybrid
from copy import deepcopy
from hybridPlanner import Node
from hybridPlanner import hyPlanner
import pygame
from pygame import *
import SRT

class demoSys(Hybrid):
    def __init__(self):
        self.dsList = []
        Hybrid.__init__(self)

    def generateSampleList(self):
        n0 = Node(0, self.G, [1,2])
        n0.set_imp(0, 0.1)          #imp_0,1
        n0.set_imp(1, 0.2)          #imp_0,2
        n1 = Node(1, self.G, [3])
        n1.set_imp(0, 0.7)
        n2 = Node(2, self.G, [3])
        n2.set_imp(0, 0.3)
        n3 = Node(3, self.G, [4])
        n3.set_imp(0, 0.5)
        n4 = Node(4, self.G, [])
        self.dsList = [n0, n1, n2, n3, n4]

    '''def F(self, (k, t), (j, x), **p):
        tp = x
        if j == 0:
            dx = np.array([-5])
        else:
            dx = np.array([5])
        return dx'''

    def G(self, (k, t), (j, x), **p):
        if j == 0:
            # off
            g = 1#x[0] - g2
        else:
            # on
            g = 1#g1 - x[0]
        return g

    '''def R(self, (k, t), (j, x), **p):
        tp = x
        k_ = k + 1
        t_ = t
        x_ = x
        if j == 1:
            j_ = 0
        elif j == 0:
            j_ = 1

        return (k_, t_), (j_, x_)

    def O(self, (k, t), (j, x), **p):
        return [x]

    def sim(self, (K, T), (j0, x0), dt, rx, **params):
        """
        This function is identical to the one in Hybrid class, rewritting it here is for debugging purpose
        """
        dt0 = dt

        k = 0
        t = [0.]
        j = deepcopy(j0)
        x = [deepcopy(x0)]

        trj = dict(k=k, t=t, j=j, x=x)
        trjs = []

        while (trj['t'][-1] <= T  # don't exceed max continuous time
               and trj['k'] < K  # don't exceed max discrete transitions
               and not trj['j'] is None):  # don't allow discrete state is None
            k0 = trj['k']
            t0 = trj['t'][-1]
            j0 = trj['j']
            x0 = trj['x'][-1]
            if 0:  # forward Euler
                dx = self.F((k0, t0), (j0, x0), **params)
            else:  # 4th-order Runge-Kutta
                f = lambda t, x: self.F((k0, t), (j0, x), **params)
                dx1 = f(t0, x0) * dt
                dx2 = f(t0 + .5 * dt, x0 + .5 * dx1) * dt
                dx3 = f(t0 + .5 * dt, x0 + .5 * dx2) * dt
                dx4 = f(t0 + dt, x0 + dx3) * dt
                dx = (1. / 6.) * (dx1 + 2 * dx2 + 2 * dx3 + dx4) / dt

            k = k0
            j = j0
            t = t0 + dt
            x = x0 + dt * dx
            g = self.G((k, t), (j, x), **params)

            # halve step size until trajectory doesn't violate guard more than rx
            i = 0
            imax = 50
            while np.any(g < -rx) and (i <= imax):
                dt = dt / 2.
                t = t0 + dt
                x = x0 + dt * dx
                g = self.G((k, t), (j, x), **params)
                i += 1

            # if (i >= imax):
            #  raise RuntimeError,'(sim)  guard iterations exceeded -- you probably have a buggy guard'

            # append state to trj
            trj['t'].append(t)
            trj['x'].append(x)

            if 0 and 'debug' in params and params['debug']:
                print '  : (k,t)=(%s,%0.3f), (j,x)=(%s,%s), dt=%0.2e, g = %s, x = %s, dx = %s' % (
                k, t, j, x, dt, g, x, dx)

            # if in guard
            # print "g before any is: ", g
            # print "any return: ", np.any(g < 0)
            if np.any(g < 0):
                # print"inside np.any"
                # spend time in guard
                if i >= imax:
                    t = t + rx
                else:
                    t = t + (rx + g.min())
                trj['t'].append(t)
                trj['x'].append(x)

                if 'debug' in params and params['debug']:
                    print 'rx: (k,t)=(%s,%0.3f), (j,x)=(%s,%s), dt=%0.2e, g = %s, x = %s' % (k, t, j, x, dt, g, x)

                # append trj to trjs
                trjs.append(trj)

                if 'Zeno' in params and params['Zeno'] and (len(trj['t']) <= 4):
                    print '(sim)  possible Zeno @ stepsize dt = %0.6f' % dt0
                    print 'rx: (k,t)=(%s,%0.3f), (j,x)=(%s,%s), dt=%0.2e, g = %s, x = %s' % (k, t, j, x, dt, g, x)
                    return trjs

                # apply reset to modify trj
                (k, t), (j, x) = self.R((k, t), (j, x), **params)
                trj = dict(k=k, t=[t], j=j, x=[x])

                # re-initialize step size
                dt = dt0

                if 'debug' in params and params['debug']:
                    g = self.G((k, t), (j, x), **params)
                    print 'rx: (k,t)=(%s,%0.3f), (j,x)=(%s,%s), dt=%0.2e, g = %s, x = %s' % (k, t, j, x, dt, g, x)

        trjs.append(trj)

        return trjs'''


if __name__ == "__main__":
    sys = demoSys()
    sys.generateSampleList()
    planner = hyPlanner(sys, [0,1],[4,1])
    guide = planner.A_star()

    # 0: (71, 247),(423, 245)
    # 1: (77, 344),(579, 172)
    # 2: (170, 17), (192,266)
    # 3: (470, 58), (513,468)
    # 4: (28 , 78), (381,429)
    for n in guide:
        #              K, m, nc, nr, np, ni, stepSize
        srt = SRT.SRT(30, 15, 5, 5, 10, 10, 15)
        srt.reset(n)
        if n == 0:
            srt.plan((71, 247),(423, 245))
        elif n == 1:
            srt.plan((77, 344),(579, 172))
        elif n == 2:
            srt.plan((170, 17), (192,266))
        elif n == 3:
            srt.plan((470, 58), (513,468))
        elif n == 4:
            srt.plan((28 , 78), (381,429))
        flag = True
        print 'LEVEL: ', n
        while flag:
            pygame.display.update()
            srt.fpsClock.tick(10000)

            for e in pygame.event.get():
                if e.type == MOUSEBUTTONDOWN:
                    flag = False

