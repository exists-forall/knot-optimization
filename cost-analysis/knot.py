import json


# The fundamental knot object.
class Knot:
    def __init__(self, angles, cost, parity):
        self.angles = angles
        self.cost = cost
        self.parity = parity
        self.adjacent = []


# Set of knots with some functionality for identifying other knots.
class KnotSet:
    def __init__(self, json_file):
        # Matrix with knots sorted by parity.
        with open(json_file) as f:
            data = json.load(f)

        self.total_parity = data["num_angles"]

        self.knots = [[] for _ in range(self.total_parity)]

        for knot in data["knots"]:
            temp_angles = knot["angles"]
            temp_angles.append(round(knot["final_angle"]))
            temp_knot = Knot(knot["angles"], knot["total_cost"], knot["angle_parity"])
            self.knots[temp_knot.parity].append(temp_knot)

    # Returns a list of knot objects exactly one "move" away from current knot.
    def adjacent_knots(self, knot):
        adj_knots = []
        for i in range(len(knot.angles)):
            for shift in [1, -1]:
                new_angles = knot.angles.copy()
                new_angles[i] = (new_angles[i] + shift) % self.total_parity
                if i + 1 == len(knot.angles):
                    new_angles[0] = (new_angles[0] - shift) % self.total_parity
                else:
                    new_angles[i+1] = (new_angles[i+1] - shift) % self.total_parity

                match = self.retrieve_from_angles(new_angles, knot.parity)

                if not match:
                    bad_knot = Knot(new_angles, 3, knot.parity)
                    adj_knots.append(bad_knot)
                else:
                    adj_knots.append(match)

        knot.adjacent = adj_knots
        return adj_knots

    def retrieve_from_angles(self, angle_set, parity):
        for candidate in self.knots[parity]:
            if candidate.angles == angle_set:
                return candidate
        return []

    def one_d_knot_list(self):
        knot_list = []
        for subset in self.knots:
            knot_list.extend(subset)
        return knot_list

# Returns the number of moves required to convert one knot to another knot.
# Requires the two knots to be of the same parity.
def distance(knot1, knot2):
    pass
