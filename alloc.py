import numpy as np
import gurobipy as gp
from gurobipy import *
from astar import *

# solver status
status_dict = {1: "loaded",
               2: "optimal",
               3: "infeasible",
               4: "infeasible and unbounded",
               5: "unbounded",
               6: "cut off",
               7: "iteration limit",
               8: "node limit",
               9: "time limit",
               10: "solution limit",
               11: "interrupted",
               12: "numeric",
               13: "suboptimal",
               14: "in progress",
               15: "user objective limit",
               16: "work limit",
               17: "memory limit"}


# problem
def optimization(g, agents, tasks):
    n = len(agents) + len(tasks)

    # creating matrix distance matrix filled with zeros
    t = np.zeros(shape=(n, n))
    r = [[[] for i in range(n)] for j in range(n)]

    # newly defined nodes
    nodes = {}
    prize = {}
    for an in range(len(agents)):
        nodes[an + 1] = agents[an]
        for an_ in range(len(agents)):
            prize[(an + 1, an_ + 1)] = 0
    for tn in range(len(tasks)):
        nodes[tn + len(agents) + 1] = tasks[tn]
        for an in range(len(agents)):
            prize[(tn + len(agents) + 1, an + 1)] = -1

    for an in range(len(agents)):
        prize[(0, an + 1)] = 0

    indices = list(nodes.keys())

    for frm in indices:
        for to in range(1, frm):
            cost, result = planning(g, nodes[frm], nodes[to], True)
            r[frm - 1][to - 1] += result
            if cost > 10000:
                cost = 10000
            t[frm - 1][to - 1] = cost
            t[to - 1][frm - 1] = t[frm - 1][to - 1]

    # set depot
    depot = 0
    indices.append(depot)

    # arch's weight
    t_0 = np.zeros((1, t.shape[0]))
    t = np.vstack((t_0, t))
    t_0 = np.zeros((1, t.shape[0]))
    t = np.concatenate((t_0.T, t), axis=1)

    # define and initialize the optimal model
    m = gp.Model()  # minimization is default
    m.Params.outputFlag = False

    # re-definite distance matrix, t
    dist = {}
    dist_ = {}
    dist_selected = {}
    for i, row in enumerate(t):
        for j, elem in enumerate(row):
            for aidx in range(len(agents)):
                if (i != j):
                    dist[(i, j, aidx+1)] = t[i][j]
                    dist_[(i, j, aidx + 1)] = 0

    # edge
    e_vars = m.addVars(dist_.keys(), obj=dist_, vtype=GRB.BINARY, name="e")
    # prize
    p_vars = m.addVars(prize.keys(), obj=prize, vtype=GRB.BINARY, name="p")

    # Constraint 1: only one vehicle can visit one stop except for the depot.
    cons1 = m.addConstrs(p_vars.sum(i+1, "*") <= 1 for i in range(len(agents) + len(tasks)))
    # Constraint 2: visited node i must have an outgoing edge.
    cons2 = m.addConstrs(e_vars.sum(i, "*", a+1) == p_vars[(i, a+1)] for i in indices for a in range(len(agents)))
    # Constraint 3: visited node j must have an ingoing edge.
    cons3 = m.addConstrs(e_vars.sum("*", j, a+1) == p_vars[(j, a+1)] for j in indices for a in range(len(agents)))
    # Constraint 4: considering the origin.
    cons4_1 = m.addConstr(p_vars.sum(depot, "*") == len(agents))
    cons4_2 = m.addConstrs(p_vars[a+1, a+1] == 1 for a in range(len(agents)))
    cons4_3 = m.addConstrs(e_vars[0, a+1, a+1] == 1 for a in range(len(agents)))
    #cons4_4 = m.addConstrs(e_vars[a+1, 0, a+1] == 1 for a in range(len(agents)))
    # Constraint 5: there is workload limit.
    cons_5 = m.addConstrs(gp.quicksum(dist[i, j, a+1] * e_vars[i, j, a+1]
                                      for i in indices
                                      for j in indices if i != j) <= 100 for a in range(len(agents)))

    def subtourlim(model, where):
        if where == GRB.Callback.MIPSOL:
            # make a list of edges selected in the solution
            vals = model.cbGetSolution(model._vars)
            selected = gp.tuplelist((i, j, k) for i, j, k in model._vars.keys() if vals[i, j, k] > 0.5)
            # find the shortest cycle in the selected edge list
            tour = subtour(selected)
            for an in range(len(agents)):
                if tour[an]:
                    for tan in tour[an]:
                        if len(tan) < n:
                            # add subtour elimination constraint for every pair of cities in tour
                            model.cbLazy(gp.quicksum(model._vars[i, j, an + 1]
                                                     for i, j in itertools.permutations(tan, 2)) <= len(tan) - 1)

    def subtour(edges, exclude_depot=True):
        cycle = [[] for an in range(len(agents))]

        for an in range(len(agents)):
            unvisited = indices.copy()

            while unvisited:            # true if list is non-empty
                this_cycle = []
                neighbors = unvisited

                while neighbors:
                    current = neighbors[0]
                    this_cycle.append(current)
                    unvisited.remove(current)
                    neighbors = [j for i, j, k in edges.select(current, "*", "*") if (j in unvisited) and (k == an + 1)]

                if len(this_cycle) > 1:
                    if exclude_depot:
                        if not (depot in this_cycle):
                            cycle[an].append(this_cycle)
        return cycle

    # optimize model
    m._vars = e_vars
    m._dvars = p_vars
    m.Params.lazyConstraints = 1
    m.optimize(subtourlim)

    print(status_dict[m.status])

    # solution
    print("objective value: ", m.objVal)

    e_vals = m.getAttr('x', e_vars)
    sol_dict = {}
    for agent in range(len(agents)):
        sol_dict[agent + 1] = {}
        for i, j, k in e_vals.keys():
            if e_vals[i, j, k] > 0.5 and (k == agent + 1):
                sol_dict[k][i] = j

    #print(sol_dict)

    routes = []
    all_paths = []
    for agent in range(len(agents)):
        print("agent: ", agent + 1)
        route = sol_dict[agent + 1]
        i = 0
        path = []
        while True:
            i = route[i]
            if i == 0:
                break
            path.append(i)
        print("path: ", path)
        if len(path) > 1:
            all_paths.append(path)

    return all_paths


