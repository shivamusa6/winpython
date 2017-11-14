from flask import Flask, jsonify
from flask import request
from ortools.constraint_solver import pywrapcp
import json
import ast

app = Flask(__name__)

location = []


@app.route('/postjson', methods=['POST'])
def postJsonHandler():
    # print(request.is_json)
    content = request.get_json()
    # print(content)
    content = str(content)
    content = ast.literal_eval(content)
    # print(content)
    stri = str(json.dumps(content))
    # print(stri)
    python_obj = json.loads(stri)
    for x in python_obj['location']:
        # print(x["lat"])
        # print(x["lon"])
        loc = [x["lat"], x["lon"]]
        array(loc)
    f_location()
    main()
    return 'jsonposted'


def array(loc):
    location.append(loc)


def f_location():
    # print(location)
    return location


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
    locations = f_location()
    demands = [1, 1, 2, 6, 1, 1, 1, 2, 1, 1]
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
                route2=''
                route_dist = 0
                route_demand = 0

                while not routing.IsEnd(index_next):
                    node_index = routing.IndexToNode(index)
                    node_index_next = routing.IndexToNode(index_next)
                    route += str(locations[node_index]) + " -> "
                    route2+=str([node_index]) +"->"
                    # Add the distance to the next node.
                    route_dist += dist_callback(node_index, node_index_next)
                    # Add demand.
                    route_demand += demands[node_index_next]
                    index = index_next
                    index_next = assignment.Value(routing.NextVar(index))

                node_index = routing.IndexToNode(index)
                node_index_next = routing.IndexToNode(index_next)
                route += "{0} {1}".format(str(node_index), str(node_index_next))
                route_dist += dist_callback(node_index, node_index_next)
                output += "Route for vehicle " + str(vehicle_nbr) + ":\n\n" + route2 + "\nDistance of route " + str(
                    vehicle_nbr) + ": " + str(route_dist) + "\nDemand met by vehicle " + str(vehicle_nbr) + ": " + str(
                    route_demand) + "\n"
                #output1 = str(vehicle_nbr) + ":" + route + ":" + str(route_dist) + ":" + str(route_demand)
                #print(output1)


        else:
            output = "No solution found."
    else:
        output = "Specify an instance greater than 0."
    print(output)
    return output



@app.route('/', methods=['GET'])
def get_tasks():
    tasks = main()
    return jsonify({'tasks': tasks})


app.run(debug=True)