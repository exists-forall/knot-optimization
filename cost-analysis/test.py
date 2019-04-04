import knot
import grapher


def test_knot_retrieval():
    full_set = knot.KnotSet("trefoil_statistics/all_knots.json")
    good_set = knot.KnotSet("trefoil_statistics/top_100.json")

    best_knot = good_set.knots[10][0]
    equivalent = full_set.retrieve_from_angles(best_knot.angles, best_knot.parity)
    print(equivalent.angles)



def adjacent_costs():
    full_set = knot.KnotSet("trefoil_statistics/all_knots.json")
    start_knot = full_set.knots[10][0]
    adjacents = full_set.adjacent_knots(start_knot)
    for adj_knot in adjacents:
        print(adj_knot.angles, adj_knot.cost)


def good_adjacency_sizes():
    full_set = knot.KnotSet("trefoil_statistics/all_knots.json")
    good_set = knot.KnotSet("trefoil_statistics/top_100.json")

    good_adjacencies = []

    for good_knot in good_set.one_d_knot_list():
        equivalent = full_set.retrieve_from_angles(good_knot.angles, good_knot.parity)
        adj = full_set.adjacent_knots(equivalent)

        count = 0
        for member in adj:
            if member.cost < 3:
                count += 1

        good_adjacencies.append(count)

    print(sum(good_adjacencies)/len(good_adjacencies))


def knot_plot_test():
    full_set = knot.KnotSet("trefoil_statistics/all_knots.json")
    start_knot = full_set.knots[10][0]
    spec = grapher.adjacent_graph(full_set, start_knot, "test_start")
    grapher.graph_from_data(spec, "knot_plot_test", "test_start.")

def graph_test():
    full_set = knot.KnotSet("trefoil_statistics/all_knots.json")
    start_knot = full_set.knots[10][0]
    spec = grapher.n_adjacent_graph(full_set, start_knot, "2-adjacent to best.", 3)
    grapher.graph_from_data(spec, "graph_test", "2-adjacent to best.")

def good_graph_test():
    full_set = knot.KnotSet("trefoil_statistics/all_knots.json")
    start_knot = full_set.knots[10][0]
    spec = grapher.n_adjacent_graph(full_set, start_knot, 2, True)
    grapher.graph_from_data(spec, "good_graph_test", "Good 2-adjacent to best.")

good_graph_test()
