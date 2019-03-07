import igraph as ig
import plotly.graph_objs as go
import plotly.plotly as py


def node_gen(knot):
    node = {
        "ranking": knot.ranking,
        "cost": knot.cost
    }
    return node


def adjacent_graph(knotset, knot, name):
    adj_knots = knotset.adjacent_knots(knot)

    # Generate a list of edges, and make a graph based on those edges.
    edges = [(0, i) for i in range(1, len(adj_knots) + 1)]
    G = ig.Graph(edges, directed=False)

    layout = G.layout('kk', dim=2)

    # Generate list of nodes from knots.
    nodes = [node_gen(knot)]
    for adj_knot in adj_knots:
        nodes.append(node_gen(adj_knot))

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
                          hoverinfo='z'
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
        title="Adjacent Knots!",
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

def adjacent_to_n_graph(, knot, n, name):
