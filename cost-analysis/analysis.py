import knot
import plotly.plotly as py
import plotly.graph_objs as go


# Returns the number of moves required to convert one knot to another knot.
# Requires the two knots to be of the same parity.
def distance(knot1, knot2, knotset):
    if knot1.parity != knot2.parity:
        return -1
    knot1copy = knot1.angles[:]
    moves = 0
    for i in range(len(knot1.angles) - 1):
        shift = knot2.angles[i] - knot1copy[i]
        if shift > 8:
            shift -= 16
        if shift < -8:
            shift += 16
        knot1copy[i] = knot1copy[i] + shift
        if i < len(knot1.angles) - 1:
            knot1copy[i+1] = (knot1copy[i+1] - shift) % knotset.total_parity
        moves += abs(shift)
    return moves

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
    all_adjacents = []

    for good_knot in good_set.one_d_knot_list():
        equivalent = full_set.retrieve_from_angles(good_knot.angles, good_knot.parity)
        adj = full_set.adjacent_knots(equivalent)
        for adjacent in adj:
            if not adjacent.in_set(all_adjacents):
                all_adjacents.append(adjacent)

        count = 0
        for member in adj:
            if member.cost < 3:
                count += 1

        good_adjacencies.append(count)

    print(sum(good_adjacencies))
    print(len(all_adjacents))
    print(sum(good_adjacencies)/len(good_adjacencies))


def top_adjacency_sizes():
    good_set = knot.KnotSet("trefoil_statistics/top_100.json")

    good_adjacencies = []
    all_adjacents = []

    for good_knot in good_set.one_d_knot_list():
        adj = good_set.adjacent_knots(good_knot)
        for adjacent in adj:
            if adjacent.cost < 3:
                good_adjacencies.append([good_knot, adjacent])

    print(len(good_adjacencies))

top_adjacency_sizes()

def distance_from_good():
    full_set = knot.KnotSet("trefoil_statistics/all_knots.json")
    good_set = knot.KnotSet("trefoil_statistics/top_100.json")

    distances = []

    for set in full_set.knots:
        if set:
            parity = set[0].parity
            parity_good = good_set.knots[parity]
            for full_knot in set:
                min_dist = 1000000000
                for good_knot in parity_good:
                    min_dist = min(distance(full_knot, good_knot, full_set), min_dist)
                if min_dist != 1000000000:
                    distances.append(min_dist)

    print(max(distances))
