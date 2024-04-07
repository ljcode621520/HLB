from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import numpy as np

# 货架与货架之间的距离 包含充电桩
rack_distance = [[0, 26, 35, 26, 33, 20, 27, 37, 22, 27, 28, 35, 29, 31, 14, 11, 30, 21, 33, 16, 28], [26, 0, 9, 8, 7, 6, 9, 11, 18, 15, 6, 9, 3, 5, 14, 15, 4, 15, 9, 12, 12], [35, 9, 0, 9, 2, 15, 8, 2, 13, 8, 13, 2, 8, 4, 21, 24, 11, 14, 2, 19, 7] , [26, 8, 9, 0, 7, 6, 1, 11, 10, 7, 14, 9, 9, 5, 12, 15, 12, 7, 7, 10, 4], [33, 7, 2, 7, 0, 13, 6, 4, 11, 8, 13, 2, 8, 4, 19, 22, 11, 12, 2, 17, 5], [20, 6, 15, 6, 13, 0, 7, 17, 16, 13, 8, 15, 9, 11, 12, 9, 10, 13, 13, 10, 10], [27, 9, 8, 1, 6, 7, 0, 10, 9, 6, 15, 8, 10, 6, 13, 16, 13, 6, 6, 11, 3], [37, 11, 2, 11, 4, 17, 10, 0, 15, 10, 11, 2, 8, 6, 23, 26, 9, 16, 4, 21, 9], [22, 18, 13, 10, 11, 16, 9, 15, 0, 5, 24, 13, 19, 15, 8, 11, 22, 3, 11, 6, 6], [27, 15, 8, 7, 8, 13, 6, 10, 5, 0, 21, 10, 16, 12, 13, 16, 19, 6, 6, 11, 3], [28, 6, 13, 14, 13, 8, 15, 11, 24, 21, 0, 11, 5, 9, 20, 17, 2, 21, 15, 18, 18], [35, 9, 2, 9, 2, 15, 8, 2, 13, 10, 11, 0, 6, 4, 21, 24, 9, 14, 4, 19, 7], [29, 3, 8, 9, 8, 9, 10, 8, 19, 16, 5, 6, 0, 4, 15, 18, 3, 16, 10, 13, 13], [31, 5, 4, 5, 4, 11, 6, 6, 15, 12, 9, 4, 4, 0, 17, 20, 7, 12, 6, 15, 9], [14, 14, 21, 12, 19, 12, 13, 23, 8, 13, 20, 21, 15, 17, 0, 3, 18, 7, 19, 2, 14], [11, 15, 24, 15, 22, 9, 16, 26, 11, 16, 17, 24, 18, 20, 3, 0, 19, 10, 22, 5, 17] , [30, 4, 11, 12, 11, 10, 13, 9, 22, 19, 2, 9, 3, 7, 18, 19, 0, 19, 13, 16, 16], [21, 15, 14, 7, 12, 13, 6, 16, 3, 6, 21, 14, 16, 12, 7, 10, 19, 0, 12, 5, 7] , [33, 9, 2, 7, 2, 13, 6, 4, 11, 6, 15, 4, 10, 6, 19, 22, 13, 12, 0, 17, 5] , [16, 12, 19, 10, 17, 10, 11, 21, 6, 11, 18, 19, 13, 15, 2, 5, 16, 5, 17, 0, 12], [28, 12, 7, 4, 5, 10, 3, 9, 6, 3, 18, 7, 13, 9, 14, 17, 16, 7, 5, 12, 0]]
# 货架与拣选台之间的来回距离
demands = [0, 20, 10, 40, 14, 24, 28, 10, 12, 14, 18, 22, 18, 18, 24, 16, 18, 16, 14, 18, 24]
# 机器人个数
num_vehicles = 2
# 考虑货架与拣选台的距离
distance_matrix = np.array(rack_distance)+np.array(demands)
distance_matrix.tolist()

def create_data_model():
    """Stores the data for the problem."""
    data = {}
    data['distance_matrix'] = distance_matrix
    data['demands'] = demands
    data['vehicle_capacities'] = [200] * num_vehicles
    data['num_vehicles'] = num_vehicles
    data['depot'] = 0
    return data

def print_solution(data, manager, routing, solution):
    total_distance = 0
    list_vehicle_id = []
    list_node = []

    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        route_distance = 0
        route_load = 0
        while not routing.IsEnd(index):
            list_vehicle_id.append(vehicle_id)
            node_index = manager.IndexToNode(index)
            list_node.append(int(manager.IndexToNode(index)))
            route_load += data['demands'][node_index]
            plan_output += ' {0} -> '.format(node_index)
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id)
        plan_output += ' {0} \n'.format(manager.IndexToNode(index))
        plan_output += 'Distance of the route: {}m\n'.format(route_distance)
        print(plan_output)
        total_distance += route_distance
    print('Total distance of all routes: {}m'.format(total_distance))
    return data

def main():
    # Instantiate the data problem.
    data = create_data_model()

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                           data['num_vehicles'], data['depot'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Capacity constraint.
    def demand_callback(from_index):
        """Returns the demand of the node."""
        # Convert from routing variable Index to demands NodeIndex.
        from_node = manager.IndexToNode(from_index)
        return data['demands'][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(
        demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data['vehicle_capacities'],  # vehicle maximum capacities
        True,  # start cumul to zero
        'Capacity')

    # Add Distance constraint.
    dimension_name = 'Distance'
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        800,  # vehicle maximum travel distance
        True,  # start cumul to zero
        dimension_name)

    # 通过软约束来平衡车辆的负载
    demand_dimension = routing.GetDimensionOrDie('Capacity')
    penalty_coef = 1000
    average_demand = sum(data['demands']) // 2
    for vehicle_id in range(manager.GetNumberOfVehicles()):
        index = routing.End(vehicle_id)
        demand_dimension.SetCumulVarSoftUpperBound(index, average_demand, penalty_coef)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    search_parameters.time_limit.FromSeconds(1)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)
    # Print solution on console.
    if solution:
        data = print_solution(data, manager, routing, solution)


if __name__ == '__main__':
    main()