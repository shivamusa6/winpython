import math
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2


def distance(x1, y1, x2, y2):
    # Manhattan distance
    dist = abs(x1 - x2) + abs(y1 - y2)

    return dist


class CreateDistanceCallback(object):
    """Create callback to calculate distances between points."""

    def __init__(self, locations):
        """Initialize distance array."""
        size = len(locations)
        self.matrix = {}

        for from_node in range(size):
            self.matrix[from_node] = {}
            for to_node in range(size):
                x1 = locations[from_node][0]
                y1 = locations[from_node][1]
                x2 = locations[to_node][0]
                y2 = locations[to_node][1]
                self.matrix[from_node][to_node] = distance(x1, y1, x2, y2)

    def Distance(self, from_node, to_node):
        return int(self.matrix[from_node][to_node])


# Demand callback
class CreateDemandCallback(object):
    """Create callback to get demands at each location."""

    def __init__(self, demands):
        self.matrix = demands

    def Demand(self, from_node, to_node):
        return self.matrix[from_node]


def main():
    # Create the data.
    output = ""
    data = create_data_array()
    locations = data[0]
    demands = data[1]
    num_locations = len(locations)
    depot = 0  # The depot is the start and end point of each route.
    num_vehicles = 5

    # Create routing model.
    if num_locations > 0:
        routing = pywrapcp.RoutingModel(num_locations, num_vehicles, depot)
        search_parameters = pywrapcp.RoutingModel.DefaultSearchParameters()

        # Callback to the distance function.
        dist_between_locations = CreateDistanceCallback(locations)
        dist_callback = dist_between_locations.Distance
        routing.SetArcCostEvaluatorOfAllVehicles(dist_callback)

        # Put a callback to the demands.
        demands_at_locations = CreateDemandCallback(demands)
        demands_callback = demands_at_locations.Demand

        # Add a dimension for demand.
        slack_max = 0
        vehicle_capacity = 100
        fix_start_cumul_to_zero = True
        demand = "Demand"
        routing.AddDimension(demands_callback, slack_max, vehicle_capacity,
                             fix_start_cumul_to_zero, demand)

        # Solve, displays a solution if any.
        assignment = routing.SolveWithParameters(search_parameters)
        if assignment:
            # Display solution.
            # Solution cost.
            print("Total distance of all routes: " + str(assignment.ObjectiveValue()) + "\n")

            for vehicle_nbr in range(num_vehicles):
                index = routing.Start(vehicle_nbr)
                index_next = assignment.Value(routing.NextVar(index))
                route = ''
                route_dist = 0
                route_demand = 0

                while not routing.IsEnd(index_next):
                    node_index = routing.IndexToNode(index)
                    node_index_next = routing.IndexToNode(index_next)
                    route += str(node_index)
                    # Add the distance to the next node.
                    route_dist += dist_callback(node_index, node_index_next)
                    # Add demand.
                    route_demand += demands[node_index_next]
                    index = index_next
                    index_next = assignment.Value(routing.NextVar(index))

                node_index = routing.IndexToNode(index)
                node_index_next = routing.IndexToNode(index_next)
                route += "{0}{1}".format(str(node_index), str(node_index_next))
                route_dist += dist_callback(node_index, node_index_next)
                output += "v{0}r{1}".format(str(vehicle_nbr), route)
        else:
            output = "No solution found."
    else:
        output = "Specify an instance greater than 0."
    return(output)




def create_data_array():
    locations = [[25.989836, 79.450035],
                 [23.223047, 82.870560],
                 [19.186354, 73.191948],
                 [30.525005, 75.890121],
                 [22.422455, 85.760651],
                 [18.106659, 83.395554],
                 [21.190449, 81.284920],
                 [23.597969, 72.969818],
                 [28.590361, 78.571762],
                 [25.369179, 85.530060]
                 ]

    demands = [1, 1, 2, 6, 1, 1, 1, 2, 1, 1]

    data = [locations, demands]
    return data


if __name__ == '__main__':
    main()



numbersList = []

listofNumbers = [1,2,3]
secondListofNumbers = [4,5,6]

numbersList.append(listofNumbers)
numbersList.append(secondListofNumbers)

for number in numbersList:
    print(number)