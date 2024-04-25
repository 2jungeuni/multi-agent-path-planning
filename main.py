import random
import numpy as np
import matplotlib.pyplot as plt

from graph import *
from agent import *
from task import *
from alloc import *

if __name__ == "__main__":
    grid_size = 30
    collision_thr = 0.8                                                       # arbitrary collision threshold
    num_agents = 10
    num_tasks = 10

    # make graph
    g = Graph(0, grid_size-1, 0, grid_size-1)                     # main graph
    plt.figure(figsize=(10, 10))                                              # grid map

    for i in range(grid_size):
        for j in range(grid_size):
            v = Vertex(i + j * grid_size, i, j)
            if random.random() >= collision_thr:
                g.set_collision(v)
                plt.scatter(i, j, marker="s", s=300, c='black')
            g.add_node(v)
    g.set_edges()

    # set agents and tasks
    agents = []
    tasks = []
    for i in range(num_agents):
        a = Agent(i)
        a.set_random_loc(g, grid_size)
        agents.append(a)
        g.add_collision(a.x, a.y)
        plt.scatter(a.x, a.y, marker="o", s=300, c="lightblue")
        plt.text(a.x-0.28, a.y-0.25, "A{}".format(i+1), fontsize=10)

    for i in range(num_tasks):
        t = Task(i)
        t.set_random_loc(g, grid_size)
        tasks.append(t)
        g.add_collision(t.x, t.y)
        plt.scatter(t.x, t.y, marker="D", s=150, c="lightgreen")
        plt.text(t.x - 0.28, t.y - 0.25, "T{}".format(i + 1), fontsize=10)



    x = np.linspace(0, grid_size-1, grid_size)
    y = np.linspace(0, grid_size-1, grid_size)

    X, Y = np.meshgrid(x, y)
    #plt.scatter(X, Y, marker="s", s=300)

    for i in range(31):
        plt.axhline(i-0.5, 0, 30, color='lightgray', linestyle='solid', linewidth=1)
        plt.axvline(i-0.5, 0, 30, color='lightgray', linestyle='solid', linewidth=1)
    plt.axhline(30 + 0.5, 0, 30, color='lightgray', linestyle='solid', linewidth=1)
    plt.axvline(30 + 0.5, 0, 30, color='lightgray', linestyle='solid', linewidth=1)

    paths = optimization(g, agents, tasks)

    colors = ['b', 'g', 'r', 'c', 'm', 'y']
    color_count = 0
    for path in paths:
        for count in range(len(path)-1):
            if count == 0:
                frm = g.get_node_frm_xy(agents[path[count]-1].x, agents[path[count]-1].y)
                to = g.get_node_frm_xy(tasks[path[count+1]-11].x, tasks[path[count+1]-11].y)
                path_ = planning(g, frm, to)
                for waypoint in path_[1]:
                    plt.scatter(waypoint[0], waypoint[1], marker="x", s=50, c=colors[color_count])
            else:
                frm = g.get_node_frm_xy(tasks[path[count] - 11].x, tasks[path[count] - 11].y)
                to = g.get_node_frm_xy(tasks[path[count + 1] - 11].x, tasks[path[count + 1] - 11].y)
                path_ = planning(g, frm, to)
                for waypoint in path_[1]:
                    plt.scatter(waypoint[0], waypoint[1], marker="x", s=50, c=colors[color_count])
        color_count += 1

    #plt.show()
    plt.savefig("grid_map_local_planner.png")