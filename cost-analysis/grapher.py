import igraph as ig
import plotly.graph_objs as go
import plotly.plotly as py
import analysis

class GraphSpec:
    def __init__(self, nodes, edges, good_only):
        self.nodes = nodes
        self.edges = edges
        self.good_only = good_only


def node_gen(knot):
    node = {
        "ranking": knot.ranking,
        "cost": knot.cost,
        "angles": knot.angles
    }
    return node

# Takes in a set of specs and puts them onto the same graph.
def merge_graphs(spec_set):
    new_nodes = []
    new_edges = []
    good_only = spec_set[0].good_only
    for spec in spec_set:
        offset = len(new_nodes)
        new_nodes.extend(spec.nodes)
        for edge in spec.edges:
            edge[0] += offset
            edge[1] += offset
            new_edges.append(edge)
    GraphSpec(new_nodes, new_edges, good_only)


# Takes in the following:
# nodes: a list of knot nodes, generated from nodegen
# edges: a list of edge pairs, in the form of a list of lists/tuples.
# name: file name
# title: title of graph
# and graphs the graph with plotly.
def graph_from_data(spec, file_name, title):
    nodes = spec.nodes
    edges = spec.edges
    good_only = spec.good_only

    for i in range(len(nodes)):
        if [i, i] not in edges:
            edges.append([i, i])

    # Generate an iGraph from the edges, and spread it out accordingly.
    G = ig.Graph(edges, directed=False)
    layout = G.layout('kk', dim=2)

    N = len(nodes)
    rankings = []
    costs = []
    for node in nodes:
        rankings.append(node["ranking"])
        costs.append(node["cost"])

    indices = range(N)
    # If good_only, then only add good nodes/edges to graph.
    if good_only:
        indices = [i for i in indices if nodes[i]["cost"] < 3]
        nodes = [nodes[i] for i in indices]
        edges = [edge for edge in edges if ((edge[0] in indices) and (edge[1] in indices))]

    # Set coordinates for nodes in graph.
    x_coords = [layout[k][0] for k in indices]
    y_coords = [layout[k][1] for k in indices]
    z_coords = [costs[k] for k in indices]

    x_edges = []
    y_edges = []
    z_edges = []

    # Set coordinates for edges in graph.
    for edge in edges:
        x_edges += [layout[edge[0]][0], layout[edge[1]][0], None]
        y_edges += [layout[edge[0]][1], layout[edge[1]][1], None]
        z_edges += [costs[edge[0]], costs[edge[1]], None]

    trace1 = go.Scatter3d(x=x_edges,
                          y=y_edges,
                          z=z_edges,
                          mode='lines',
                          hoverinfo = 'none'
                          )

    trace2 = go.Scatter3d(x=x_coords,
                          y=y_coords,
                          z=z_coords,
                          mode='markers',
                          name='knots',
                          marker=dict(symbol='circle',
                                      size=6,
                                      colorscale='Viridis',
                                      line=dict(color='rgb(50,50,50)', width=0.5)
                                      ),
                          text=z_coords,
                          hoverinfo='text'
                          )

    z_axis = dict(showbackground=True,
                  showline=True,
                  zeroline=True,
                  showgrid=True,
                  showticklabels=True,
                  title='Costs'
                  )

    axis = dict(showbackground=False,
                showline=False,
                zeroline=False,
                showgrid=False,
                showticklabels=False,
                title=''
                )

    layout = go.Layout(
        title=title,
        width=1000,
        height=1000,
        showlegend=False,
        scene=dict(
            xaxis=dict(axis),
            yaxis=dict(axis),
            zaxis=dict(z_axis),
        ),
        margin=dict(
            t=100
        ),
        hovermode='closest',
    )

    data = [trace1, trace2]
    fig = go.Figure(data = data, layout=layout)

    py.plot(fig, filename=file_name)


# Given a set of knots, graph them with appropriate edges included.
def add_edges(knotset):
    nodes = []
    edges = []

    knots = knotset.one_d_knot_list()

    for some_knot in knots:
        nodes.append(node_gen(some_knot))


    for i in range(len(knots)):
        knot1 = knots[i]
        for j in range(i + 1, len(knots)):
            dist = analysis.distance(knot1, knots[j], knotset)
            if dist <= 3 and dist != -1:
                edges.append([i, j])

    spec = GraphSpec(nodes, edges, True)
    return spec


# Returns the spec for a graph adjacent to one knot.
def adjacent_graph(knotset, knot, good_only = False):
    adj_knots = knotset.adjacent_knots(knot)

    # Generate a list of edges, and make a graph based on those edges.
    edges = [(0, i) for i in range(1, len(adj_knots) + 1)]

    # Generate list of nodes from knots.
    nodes = [node_gen(knot)]
    for adj_knot in adj_knots:
        nodes.append(node_gen(adj_knot))

    spec = GraphSpec(nodes, edges, good_only)
    return spec

# Returns the specs for a graph of vertices n away from q knot.
def n_adjacent_graph(knotset, knot, n, good_only = False):
    if n == 0:
        nodes = [node_gen(knot)]
        spec = GraphSpec(nodes, [], good_only)
        return spec
    else:
        edges = []
        all_knots = [knot]
        start = 0
        end = 1
        while n > 0:
            # For all knots from this "adjacency class"
            for index in range(start, end):
                # Find the knots adjacent to this knot, and initialize those
                # knots' adjacency lists as well. In doing so, weed out the ones
                # that are already in the set of all knots.
                curr_knot = all_knots[index]
                adjacents = knotset.adjacent_knots(curr_knot)
                new_knots = []
                for adj in adjacents:
                    if not adj.in_set(all_knots):
                        knotset.adjacent_knots(adj)
                        new_knots.append(adj)

                # Determine if these knots are adjacent to any knot currently
                # in the list. They should at the least be adjacent to curr_knot.
                # If so, add an edge.
                for i in range(len(new_knots)):
                    for j in range(len(all_knots)):
                        first = new_knots[i]
                        second = all_knots[j]
                        if first.is_adjacent(second):
                            edges.append((i+len(all_knots), j))

                # Once finished, simple append this set of knots to end.
                all_knots.extend(new_knots)

            # Update the adjacency class.
            n = n - 1
            start = end
            end = len(all_knots)

        nodes = [node_gen(knot) for knot in all_knots]

        spec = GraphSpec(nodes, edges, good_only)
        return spec


# Graph the top n knots of given parity, with edges between pairs of distances
# less than or equal to d.
# Given a set of knots, graph them with appropriate edges included.
def same_parity_graph(knotset, n, parity, d):
    nodes = []
    edges = []

    knots = knotset.knots[parity]

    for i in range(min(n, len(knots))):
        nodes.append(node_gen(knots[i]))

    for i in range(len(knots)):
        knot1 = knots[i]
        for j in range(i + 1, len(knots)):
            dist = analysis.distance(knot1, knots[j], knotset)
            if dist <= d:
                edges.append([i, j])

    spec = GraphSpec(nodes, edges, True)
    return spec
