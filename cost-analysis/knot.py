import json


# The fundamental knot object.
class Knot:
    def __init__(self, angles, cost, parity):
        self.angles = angles
        self.cost = cost
        self.parity = parity


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

                found = False
                for parity_match in self.knots[knot.parity]:
                    if parity_match.angles == new_angles:
                        adj_knots.append(parity_match)
                        found = True

                if not found:
                    bad_knot = Knot(new_angles, 3, knot.parity)
                    adj_knots.append(bad_knot)

        return adj_knots


# Returns the number of moves required to convert one knot to another knot.
# Requires the two knots to be of the same parity.
def distance(knot1, knot2):
    pass
