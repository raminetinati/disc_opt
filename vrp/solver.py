#!/usr/bin/python
# -*- coding: utf-8 -*-

import math
from collections import namedtuple
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

Customer = namedtuple("Customer", ['index', 'demand', 'x', 'y'])

def length(customer1, customer2):
    return math.sqrt((customer1.x - customer2.x)**2 + (customer1.y - customer2.y)**2)

def parse_inp(input_data):
    lines = input_data.split('\n')

    parts = lines[0].split()
    customer_count = int(parts[0])
    vehicle_count = int(parts[1])
    vehicle_capacity = int(parts[2])

    customers = []
    for i in range(1, customer_count + 1):
        line = lines[i]
        parts = line.split()
        customers.append(Customer(i - 1, int(parts[0]), float(parts[1])*100.0, float(parts[2])*100.0))

    # the depot is always the first customer in the input
    depot = customers[0]
    return depot, customers, vehicle_count, vehicle_capacity

class CreateDistanceCallback(object):
    """Create callback to calculate distances between points."""
    @staticmethod
    def distance(x1, y1, x2, y2):
        return math.sqrt((x1-x2)**2 + (y1 - y2)**2)
    def __init__(self, customers):
        """Initialize distance array."""
        size = len(customers)
        self.matrix = {}

        for from_node in xrange(size):
            self.matrix[from_node] = {}
            for to_node in xrange(size):
                x1 = customers[from_node].x
                y1 = customers[from_node].y
                x2 = customers[to_node].x
                y2 = customers[to_node].y
                self.matrix[from_node][to_node] = self.distance(x1, y1, x2, y2)

    def Distance(self, from_node, to_node):
        return int(self.matrix[from_node][to_node])

# Demand callback
class CreateDemandCallback(object):
    """Create callback to get demands at each location."""

    def __init__(self, customers):
        self.matrix = [_.demand for _ in customers]

    def Demand(self, from_node, to_node):
        return self.matrix[from_node]


def solve_vrp(input_data):
    depot, customers, num_vehicles, vehicle_capacity = parse_inp(input_data)
    num_locastions = len(customers)
    routing = pywrapcp.RoutingModel(num_locastions, num_vehicles, 0)
    search_parameters = pywrapcp.RoutingModel_DefaultSearchParameters()
    search_parameters.time_limit_ms = 600*1000
    dist_between_locations = CreateDistanceCallback(customers)
    dist_callback = dist_between_locations.Distance
    routing.SetArcCostEvaluatorOfAllVehicles(dist_callback)
   # Put a callback to the demands.
    demands_at_locations = CreateDemandCallback(customers)
    demands_callback = demands_at_locations.Demand

    slack_max = 0

    fix_start_cumul_to_zero = True
    demand = "Demand"
    routing.AddDimension(demands_callback, slack_max, vehicle_capacity,
                         fix_start_cumul_to_zero, demand)

    # Solve, displays a solution if any.
    assignment = routing.SolveWithParameters(search_parameters)
    if assignment:
        output = ("%.2f" % (assignment.ObjectiveValue()/100.0)) + " 0\n"
        for ii in range(num_vehicles):
            index = routing.Start(ii)
            index_next = assignment.Value(routing.NextVar(index))
            route = ''
            route_dist = 0
            route_demand = 0

            while not routing.IsEnd(index_next):
                node_index = routing.IndexToNode(index)
                node_index_next = routing.IndexToNode(index_next)
                route += str(node_index) + " "
                # Add the distance to the next node.
                route_dist += dist_callback(node_index, node_index_next)
                # Add demand.
                route_demand += customers[node_index_next].demand

                index = index_next
                index_next = assignment.Value(routing.NextVar(index))

            node_index = routing.IndexToNode(index)
            node_index_next = routing.IndexToNode(index_next)
            route += str(node_index) + " " + str(node_index_next)
            output += route + '\n'
            print "Route for vehicle " + str(ii) + ":\n\n" + route + "\n"
            print "Distance of route " + str(ii) + ": " + str(route_dist)
            print "Demand met by vehicle " + str(ii) + ": " + str(route_demand) + "\n"
        return output
    else:
        return solve_simp(input_data)

def solve_it(input_data):
    return solve_vrp(input_data)

def solve_simp(input_data):
    # Modify this code to run your optimization algorithm
    depot, customers, vehicle_count, vehicle_capacity = parse_inp(input_data)
    # parse the input


    # build a trivial solution
    # assign customers to vehicles starting by the largest customer demands
    vehicle_tours = []
    
    remaining_customers = set(customers)
    remaining_customers.remove(depot)
    
    for v in range(0, vehicle_count):
        # print "Start Vehicle: ",v
        vehicle_tours.append([])
        capacity_remaining = vehicle_capacity
        while sum([capacity_remaining >= customer.demand for customer in remaining_customers]) > 0:
            used = set()
            order = sorted(remaining_customers, key=lambda customer: -customer.demand)
            for customer in order:
                if capacity_remaining >= customer.demand:
                    capacity_remaining -= customer.demand
                    vehicle_tours[v].append(customer)
                    # print '   add', ci, capacity_remaining
                    used.add(customer)
            remaining_customers -= used

    # checks that the number of customers served is correct
    assert sum([len(v) for v in vehicle_tours]) == len(customers) - 1

    # calculate the cost of the solution; for each vehicle the length of the route
    obj = 0
    for v in range(0, vehicle_count):
        vehicle_tour = vehicle_tours[v]
        if len(vehicle_tour) > 0:
            obj += length(depot,vehicle_tour[0])
            for i in range(0, len(vehicle_tour)-1):
                obj += length(vehicle_tour[i],vehicle_tour[i+1])
            obj += length(vehicle_tour[-1],depot)

    # prepare the solution in the specified output format
    outputData = '%.2f' % obj + ' ' + str(0) + '\n'
    for v in range(0, vehicle_count):
        outputData += str(depot.index) + ' ' + ' '.join([str(customer.index) for customer in vehicle_tours[v]]) + ' ' + str(depot.index) + '\n'

    return outputData


import sys

if __name__ == '__main__':
    import sys
    if len(sys.argv) > 1:
        file_location = sys.argv[1].strip()
        with open(file_location, 'r') as input_data_file:
            input_data = input_data_file.read()
        print(solve_it(input_data))
    else:

        print('This test requires an input file.  Please select one from the data directory. (i.e. python solver.py ./data/vrp_5_4_1)')

