import knot
import grapher
import analysis


def test_knot_retrieval():
    full_set = knot.KnotSet("trefoil_statistics/all_knots.json")
    good_set = knot.KnotSet("trefoil_statistics/top_100.json")

    best_knot = good_set.knots[10][0]
    equivalent = full_set.retrieve_from_angles(best_knot.angles, best_knot.parity)
    print(equivalent.angles)

def knot_plot_test():
    full_set = knot.KnotSet("trefoil_statistics/all_knots.json")
    start_knot = full_set.knots[10][0]
    spec = grapher.adjacent_graph(full_set, start_knot, "test_start")
    grapher.graph_from_data(spec, "knot_plot_test", "test_start.")

def graph_test():
    full_set = knot.KnotSet("trefoil_statistics/all_knots.json")
    start_knot = full_set.knots[10][0]
    spec = grapher.n_adjacent_graph(full_set, start_knot, 2)
    grapher.graph_from_data(spec, "graph_test", "2-adjacent to best.")

def good_graph_test():
    full_set = knot.KnotSet("trefoil_statistics/all_knots.json")
    start_knot = full_set.knots[10][0]
    spec = grapher.n_adjacent_graph(full_set, start_knot, 2, True)
    grapher.graph_from_data(spec, "good_graph_test", "Good 2-adjacent to best.")

def distance_test():
    full_set = knot.KnotSet("trefoil_statistics/all_knots.json")
    start_knot = full_set.knots[10][0]
    adjacents = full_set.adjacent_knots(start_knot)
    for adj_knot in adjacents:
        print(start_knot.angles)
        print(adj_knot.angles)
        print(analysis.distance(start_knot, adj_knot, full_set))

def good_neighborhoods_test():
    full_set = knot.KnotSet("trefoil_statistics/all_knots.json")

    spec = grapher.add_edges(full_set)
    grapher.graph_from_data(spec, "all_neighborhood_test", "All Knots.")


good_neighborhoods_test()
