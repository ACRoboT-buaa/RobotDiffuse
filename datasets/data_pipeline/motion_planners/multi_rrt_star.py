from random import random
from time import time

from .utils import INF, argmin


class OptimalNode(object):

    def __init__(self, config, parent=None, d=0, path=[], iteration=None):
        self.config = config
        self.parent = parent
        self.children = set()
        self.d = d
        self.path = path
        if parent is not None:
            self.cost = parent.cost + d
            self.parent.children.add(self)
        else:
            self.cost = d
        self.solution = False
        self.creation = iteration
        self.last_rewire = iteration

    def set_solution(self, solution):
        if self.solution is solution:
            return
        self.solution = solution
        if self.parent is not None:
            self.parent.set_solution(solution)

    def retrace(self):
        if self.parent is None:
            return self.path + [self.config]
        return self.parent.retrace() + self.path + [self.config]

    def rewire(self, parent, d, path, iteration=None):
        if self.solution:
            self.parent.set_solution(False)
        self.parent.children.remove(self)
        self.parent = parent
        self.parent.children.add(self)
        if self.solution:
            self.parent.set_solution(True)
        self.d = d
        self.path = path
        self.update()
        self.last_rewire = iteration

    def update(self):
        self.cost = self.parent.cost + self.d
        for n in self.children:
            n.update()

    def clear(self):
        self.node_handle = None
        self.edge_handle = None

    def draw(self, env):
        from manipulation.primitives.display import draw_node, draw_edge
        color = (0, 0, 1, .5) if self.solution else (1, 0, 0, .5)
        self.node_handle = draw_node(env, self.config, color=color)
        if self.parent is not None:
            self.edge_handle = draw_edge(
                env, self.config, self.parent.config, color=color)

    def __str__(self):
        return self.__class__.__name__ + '(' + str(self.config) + ')'
    __repr__ = __str__


def safe_path(sequence, collision):
    path = []
    for q in sequence:
        if collision(q):
            break
        path.append(q)
    return path


def multi_rrt_star(start, goals, distance, sample, extend, collision, radius, max_time=INF, max_iterations=INF, goal_probability=.2, informed=True):
    k=len(goals)

    if collision(start):
        return None,None,None
    nodes = [OptimalNode(start)]
    goal_ns = []
    doable=[]
    for i in range(k):
        goal_ns.append(None)
        if collision(goals[i]):
            doable.append(0)
        else:
            doable.append(1)
    t0 = time()
    it = 0
    while (time()-t0) < max_time and it < max_iterations:
        goal=goals[it%k]
        p=it%k
        if doable[it%k]==0:
            it+=1
            continue
        goal_n=goal_ns[it%k]
        do_goal = goal_n is None and (it == 0 or random() < goal_probability)
        s = goal if do_goal else sample()


        if it % 100 == 0:
            print (it, time() - t0, goal_n is not None, do_goal, [goal_ns[i].cost if goal_ns[i] is not None else INF for i in range(k)])


        if informed and goal_n is not None and distance(start, s) + distance(s, goal) >= goal_n.cost:
            it+=1
            continue
        it += 1
        nearest = argmin(lambda n: distance(n.config, s), nodes)
        path = safe_path(extend(nearest.config, s), collision)
        if len(path) == 0:
            continue
        new = OptimalNode(path[-1], parent=nearest, d=distance(
                nearest.config, path[-1]), path=path[:-1], iteration=it)


        if goal_n is None and distance(new.config, goal) < 1e-6:
            goal_n = new
            goal_ns[p]=goal_n
            goal_n.set_solution(True)

        #neighbors = filter(lambda n: distance(
         #   n.config, new.config) < radius, nodes)


        #for n in neighbors:
        #    d = distance(n.config, new.config)
        #    if n.cost + d < new.cost:
        #        path = safe_path(extend(n.config, new.config), collision)
        #        if len(path) != 0 and distance(new.config, path[-1]) < 1e-6:
        #            new.rewire(n, d, path[:-1], iteration=it)
        neighbors = filter(lambda n: distance(
            n.config, new.config) < radius, nodes)
        for n in neighbors:
            d = distance(new.config, n.config)
            if new.cost + d < n.cost:
                path = safe_path(extend(new.config, n.config), collision)
                if len(path) != 0 and distance(n.config, path[-1]) < 1e-6:
                    n.rewire(new, d, path[:-1], iteration=it)
        nodes.append(new)
    res=[]
    cost=[]
    for i in range(k):
        if goal_ns[i]==None:
            res.append(None)
            cost.append(None)
        else:
            res.append(goal_ns[i].retrace())
            cost.append(goal_ns[i].cost)
    return res, nodes, cost
