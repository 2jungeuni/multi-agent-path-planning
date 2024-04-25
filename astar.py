import numpy as np

def planning(graph, s, g, get_path=True):
    open, closed = set(), set()
    s.cost = 0
    open.add(s)

    while True:
        if len(open) == 0:
            return np.inf

        current = min(open, key=lambda o: o.cost + graph.get_heuristic(o, g))

        if current.x == g.x and current.y == g.y:
            #print("find goal")
            g.parent = current.parent
            g.cost = current.cost
            #print("final cost: ", g.cost)
            if get_path:
                path = [(g.x, g.y)]
                parent_ = g.parent
                while parent_ is not s:
                    path.append((parent_.x, parent_.y))
                    parent_ = parent_.parent
                path.append((s.x, s.y))
                path.reverse()
                return g.cost, path
            else:
                return g.cost

        # remove the item from the open set
        open.remove(current)

        # add it to the closed set
        closed.add(current)

        for frm in graph.edges[current.id]:
            v_frm = graph.nodes[frm]
            cost_frm = current.cost + graph.edges[current.id][v_frm.id]["distance"]

            # if the node is not safe, do nothing
            if v_frm.collision:
                continue

            if v_frm in closed:
                continue

            if v_frm not in open:
                open.add(v_frm)
                v_frm.cost = cost_frm
                v_frm.parent = current
            else:
                if v_frm.cost > cost_frm:
                    # this path is the best until now. record it
                    v_frm.cost = cost_frm
                    v_frm.parent = current

