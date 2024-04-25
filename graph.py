import numpy as np

# information of nodes
class Vertex(object):
    def __init__(self, idx, x, y):
        self.id = idx
        self.x = x
        self.y = y
        self.cost = np.inf
        self.parent = None
        self.collision = False

    def get_coordinate(self):
        return self.x, self.y

    def set_collision(self):
        self.collision = True

    def unset_collision(self):
        self.collision = False


# information of graph
class Graph(object):
    def __init__(self, min_x, max_x, min_y, max_y):
        self.nodes = {}
        self.coord = {}
        self.edges = {}
        self.collisions = []

        self.min_x = min_x
        self.max_x = max_x
        self.min_y = min_y
        self.max_y = max_y

    def get_node(self, id):
        return self.nodes[id]

    def get_node_frm_xy(self, x, y):
        return self.coord[(x, y)]

    def get_edge(self, id):
        return self.edges[id]

    def get_nodes(self):
        return self.nodes

    def get_edges(self):
        return self.edges

    def get_coordinate(self, v):
        return v.x, v.y

    def get_heuristic(self, v, g):
        return np.sqrt(np.square(v.x - v.x) + np.square(g.y - g.y))

    def get_cost(self, to, frm):
        if to not in self.nodes:
            print("There is no ", to.id, "vertex.")
            return
        if frm not in self.nodes:
            print("There is no ", frm.id, "vertex.")
            return
        return self.edges[to][frm]

    def get_cost_detail(self, to, frm, key):
        if to not in self.nodes:
            print("There is no ", to.id, "vertex.")
            return
        if frm not in self.nodes:
            print("There is no ", frm.id, "vertex.")
            return
        if key not in self.edges[to][frm]:
            print("There is no key.")
            return
        return self.edges[to][frm][key]

    def add_node(self, v):
        self.nodes[v.id] = v
        self.coord[(v.x, v.y)] = v
        self.edges[v.id] = {}

    def add_edge(self, to, frm, speed=0, elevation=0, roughness=0, collision_freq=0, collision=0):
        if to not in self.nodes:
            self.nodes[to.idx] = to
        if frm not in self.nodes:
            self.nodes[frm.idx] = frm

        if (self.nodes[to].collision is False
                and self.nodes[frm].collision is False):
            distance = np.sqrt(np.square(self.nodes[to].x - self.nodes[frm].x)
                               + np.square(self.nodes[to].y - self.nodes[frm].y))
        else:
            distance = np.inf

        cost = {"distance": distance,
                "speed": speed,
                "elevation": elevation,
                "roughness": roughness,
                "collision_freq": collision_freq,
                "collision": collision}

        self.edges[to][frm] = cost

    def add_collision(self, x, y):
        self.collisions.append((x, y))

    def set_collision(self, v):
        v.set_collision()
        self.collisions.append((v.x, v.y))

    def set_edges(self):
        for to in self.nodes:
            left = (self.nodes[to].x - 1, self.nodes[to].y)
            right = (self.nodes[to].x + 1, self.nodes[to].y)
            up = (self.nodes[to].x, self.nodes[to].y - 1)
            down = (self.nodes[to].x, self.nodes[to].y + 1)
            if left in self.coord:
                self.add_edge(to, self.coord[left].id)
                self.add_edge(self.coord[left].id, to)
            if right in self.coord:
                self.add_edge(to, self.coord[right].id)
                self.add_edge(self.coord[right].id, to)
            if up in self.coord:
                self.add_edge(to, self.coord[up].id)
                self.add_edge(self.coord[up].id, to)
            if down in self.coord:
                self.add_edge(to, self.coord[down].id)
                self.add_edge(self.coord[down].id, to)