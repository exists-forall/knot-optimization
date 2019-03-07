import igraph as ig
import plotly.graph_objs as go
import plotly.plotly as py


def node_gen(knot):
    node = {
        "ranking": knot.ranking,
        "cost": knot.cost,
        "angles": knot.angles
    }
    return node


# Takes in the following:
# nodes: a list of knot nodes, generated from nodegen
# edges: a list of edge pairs, in the form of a list of lists/tuples.
# name: file name
# title: title of graph
# and graphs the graph with plotly.
def graph_from_data(nodes, edges, name, title):

    # Generate an iGraph from the edges, and spread it out accordingly.
    G = ig.Graph(edges, directed=False)
    layout = G.layout('kk', dim=2)


    N = len(nodes)
    rankings = []
    costs = []
    for node in nodes:
        rankings.append(node["ranking"])
        costs.append(node["cost"])

    # Set coordinates for nodes in graph.
    x_coords = [layout[k][0] for k in range(N)]
    y_coords = [layout[k][1] for k in range(N)]
    z_coords = costs

    x_edges = []
    y_edges = []
    z_edges = []
    # Set coordinates for edges in graph.

    for edge in edges:
        x_edges += [x_coords[edge[0]], x_coords[edge[1]], None]
        y_edges += [y_coords[edge[0]], y_coords[edge[1]], None]
        z_edges += [z_coords[edge[0]], z_coords[edge[1]], None]

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
                          text=costs,
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

    py.plot(fig, filename=name)


# Graph adjacent to one knot.
def adjacent_graph(knotset, knot, name):
    adj_knots = knotset.adjacent_knots(knot)

    # Generate a list of edges, and make a graph based on those edges.
    edges = [(0, i) for i in range(1, len(adj_knots) + 1)]

    # Generate list of nodes from knots.
    nodes = [node_gen(knot)]
    for adj_knot in adj_knots:
        nodes.append(node_gen(adj_knot))

    graph_from_data(nodes, edges, name, "Adjacent to best.")


# Graph of vertices n away from q knot.
def n_adjacent_graph(knotset, knot, name, n):
    if n == 0:
        nodes = [node_gen(knot)]
        graph_from_data(nodes, [], name, "Single node.")
    else:
        graph_name = str(n) + "-adjacent"
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

        graph_from_data(nodes, edges, name, graph_name)
