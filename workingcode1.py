#!/usr/bin/env python
# coding: utf-8



import pandas as pd
import math
import numpy as np
import matplotlib.pyplot as plt
from pandas import ExcelWriter
from pandas import ExcelFile
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp




df = pd.read_excel('Christophides.xlsx', sheet_name='CMT5')
df.head()





df['loc'] = [(int(XCOORD),int(YCOORD)) for XCOORD, YCOORD in df[['XCOORD.', 'YCOORD.']].values ]
df.head()





locate = df['loc']
demand = df['DEMAND']
xs = [x[0] for x in locate]
ys = [x[1] for x in locate]
customer = df['CUSTOMER']
plt.scatter(xs, ys,)
for i, label in enumerate(customer):
    plt.text(xs[i], ys[i],label)
plt.figure(figsize=(20,10))
plt.show()




def create_data_model():
    """Stores the data for the problem."""
    data = {}
    data['locations'] = locate
    data['num_vehicles'] = 20
    data['depot'] = 0
    data['dem'] = demand
    return data

def compute_euclidean_distance_matrix(locations):
    """Creates callback to return distance between points."""
    distances = {}
    for from_counter, from_node in enumerate(locations):
        distances[from_counter] = {}
        for to_counter, to_node in enumerate(locations):
            if from_counter == to_counter:
                distances[from_counter][to_counter] = 0
            else:
                # Euclidean distance
                distances[from_counter][to_counter] = (int(math.hypot((from_node[0] - to_node[0]),(from_node[1] - to_node[1]))))
    return distances

def get_routes(solution, routing, manager):
  #Get vehicle routes from a solution and store them in an array
  # Get vehicle routes and store them in a two dimensional array whose
  # i,j entry is the jth location visited by vehicle i along its route.
    routes = []
    for route_nbr in range(routing.vehicles()):
        index = routing.Start(route_nbr)
        route = [manager.IndexToNode(index)]
        while not routing.IsEnd(index):
            index = solution.Value(routing.NextVar(index))
            route.append(manager.IndexToNode(index))
        routes.append(route)
    return routes


def print_solution(data, manager, routing, solution):
    # Display dropped nodes.
    dropped_nodes = 'Dropped nodes:'
    for node in range(routing.Size()):
        if routing.IsStart(node) or routing.IsEnd(node):
            continue
        if solution.Value(routing.NextVar(node)) == node:
            dropped_nodes += ' {}'.format(manager.IndexToNode(node))
    print(dropped_nodes)
    """Prints solution on console."""
    max_route_distance = 0
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        route_distance = 0
        while not routing.IsEnd(index):
            plan_output += ' {} -> '.format(manager.IndexToNode(index))
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id)
        plan_output += '{}\n'.format(manager.IndexToNode(index))
        plan_output += 'Distance of the route: {}\n'.format(route_distance)
        print(plan_output)
        max_route_distance = max(route_distance, max_route_distance)
    print('Maximum of the route distances: {}'.format(max_route_distance))
         

def main():
    """Solve the CVRP problem."""
    # Instantiate the data problem.
    data = create_data_model()

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['locations']),
                                           data['num_vehicles'], data['depot'])
    
    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)
    
    distance_matrix = compute_euclidean_distance_matrix(data['locations'])
    
    
    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    #Add Distance constraint.
    dimension_name = 'Distance'
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        230,  # vehicle maximum travel distance
        True,  # start cumul to zero
        dimension_name)
    #distance_dimension = routing.GetDimensionOrDie(dimension_name)
    
    
    #create demand call back for locations
    def demand_callback(from_index):
    #Returns the demand of the node
    # Convert from routing variable Index to demands NodeIndex.
        from_node = manager.IndexToNode(from_index)
        return data['dem'][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    
    #Add capacity constraint
    dimension_name2 = 'Capacity'
    routing.AddDimension(
        demand_callback_index,
        0,  # null capacity slack
        1150,  # vehicle maximum capacities
        True,  # start cumul to zero
    'Capacity')
    capacity_dimension = routing.GetDimensionOrDie(dimension_name2)
    
    #penalty = 80
    #for node in range(1, len(data['locations'])):
      #  routing.AddDisjunction([manager.NodeToIndex(node)], penalty)

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.AUTOMATIC)
    search_parameters.local_search_metaheuristic = (
    routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    #search_parameters.time_limit.seconds = 180
    #search_parameters.log_search = True
   
    #solution = routing.Solve()
    #search_parameters.local_search_metaheuristic = (
    #routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    search_parameters.time_limit.seconds = 600
    search_parameters.log_search = True
    
       # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)
    
    routes = get_routes(solution, routing, manager)
    # Display the routes.
    for i, route in enumerate(routes):
        print('Route', i, route)
        
    solver_initiation = routing.solver()
    
    Monitor = pywrapcp.SearchMonitor(solver_initiation)
    
  
    objective_value = solution.ObjectiveValue()
    print(objective_value)
    
    number_of_branches = solver_initiation.Branches()
    print(number_of_branches)
    
    solution_count = solver_initiation.Solutions()
    print(solution_count)
    
    
    
    constraints_added = solver_initiation.Constraints()
    print(constraints_added)
    
    added_neighbours = solver_initiation.AcceptedNeighbors()
    print(added_neighbours)
    
    memory_usage = solver_initiation.MemoryUsage()
    print(memory_usage)
    
    solution_status = routing.status()
    print(solution_status)
    
    # Print solution on console.
    if solution:
           print_solution(data, manager, routing, solution)
            
    else:
      print('no solution found')

if __name__ == '__main__':
    main()







