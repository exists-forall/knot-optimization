import json

# The fundamental knot object.
class Knot:
    def __init__(self, angles, final_angle, cost, parity):
        self.angles = angles
        self.angles.append(round(final_angle))
        self.cost = cost
        self.parity = parity

    def __init__(self, angles, cost, parity):
        self.angles = angles
        self.cost = cost
        self.parity = parity

# Set of knots with some functionality for identifying other knots.
class KnotSet:
    def __init__(self, json_file):
        # Matrix with knots sorted by parity.
        self.knots = [[] for _ in range(16)]

        with open(json_file) as f:
            data = json.load(f)
        for knot in data["knots"]:
            temp_knot = Knot(knot["angles"], knot["final_angle"], knot["total_cost"], knot["angle_parity"])
            self.knots[temp_knot.parity].append(temp_knot)

    # Returns a list of knot objects exactly one "move" away from current knot.
    def adjacent_knots(self, knot):
        adj_knots = []
        for i in range(len(knot.angles)):
            new_angles = knot.angles.copy()
            new_angles[i] += 1
            if (i == len(knot.angles) - 1):
                new_angles[0] -= 1
            else:
                new_angles[i + 1] -= 1
            for parity_match in self.knots[knot.parity]:
                if parity_match.angles == new_angles:
                    adj_knots.append(parity_match)
                else:
                    bad_knot = Knot(angles, 2, knot.parity)
                    adj_knots.append(bad_knot)

            new_angles = knot.angles.copy()
            new_angles[i] -= 1
            if (i == len(knot.angles) - 1):
                new_angles[0] += 1
            else:
                new_angles[i + 1] += 1
            for parity_match in self.knots[knot.parity]:
                if parity_match.angles == new_angles:
                    adj_knots.append(parity_match)
                else:
                    bad_knot = Knot(angles, 2, knot.parity)
                    adj_knots.append(bad_knot)
        return adj_knots
        
# Returns the number of moves requred to convert one knot to another knot.
# Requires the two knots to be of the same parity.
def distance(knot1, knot2):
    pass
