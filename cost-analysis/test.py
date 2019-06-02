import knot
import analysis


def test_knot_retrieval():
    full_set = knot.KnotSet("trefoil_statistics/all_knots.json")
    good_set = knot.KnotSet("trefoil_statistics/top_100.json")

    best_knot = good_set.knots[10][0]
    equivalent = full_set.retrieve_from_angles(best_knot.angles, best_knot.parity)
    print(equivalent.angles)

def cost_count():
    good_set = knot.KnotSet("trefoil_statistics/top_100.json")
    total = 0
    for k in good_set.one_d_knot_list():
        if k.cost < 0.05:
            total += 1
    print(total)

def knot_plot_test():
    full_set = knot.KnotSet("trefoil_statistics/all_knots.json")
    start_knot = full_set.knots[10][0]
    spec = grapher.adjacent_graph(full_set, start_knot, "test_start")
    grapher.graph_from_data(spec, "knot_plot_test", "test_start.")

def graph_test():
    full_set = knot.KnotSet("trefoil_statistics/all_knots.json")
    start_knot = full_set.knots[10][0]
    spec = grapher.n_adjacent_graph(full_set, start_knot, 2)
    grapher.graph_from_data(spec, "best_knot, d=3", "Neighborhood around best knot, d=3.")

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
    good_set = knot.KnotSet("trefoil_statistics/top_100.json")

    spec = grapher.add_edges(good_set)
    grapher.graph_from_data(spec, "good_neighborhood_test", "Good Knots.")

def good_parity_sum():
    good_set = knot.KnotSet("trefoil_statistics/top_100.json")
    for i in range(9):
        print("Parity: " + str(i) + ", Number: " + str(len(good_set.knots[i])))

def all_parity_sum():
    full_set = knot.KnotSet("trefoil_statistics/all_knots.json")
    good_set = knot.KnotSet("trefoil_statistics/top_100.json")
    for i in range(16):
        print("Parity: " + str(i) + ", Top 100: " + str(len(good_set.knots[i])) + ", All: " + str(len(full_set.knots[i])))

def same_parity_graph_test():
    full_set = knot.KnotSet("trefoil_statistics/all_knots.json")
    spec = grapher.same_parity_graph(full_set, 50, 8, 3)
    grapher.graph_from_data(spec, "Parity 8, Edges 3", "50 Best Knots of Parity 8, d=3.")

def within_one_test():
    knot_1 = knot.Knot([1, 3, 0, 5], 0, 0, 0)
    knot_2 = knot.Knot([2, 2, 15, 5], 0, 0, 0)
    print(knot_1.within_one(knot_2))
