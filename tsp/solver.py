#!/usr/bin/python
# -*- coding: utf-8 -*-

import math
from collections import namedtuple

from jedi.evaluate.dynamic import search_params
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2
import time
Point = namedtuple("Point", ['x', 'y'])


class Points(object):
    def __init__(self, inp_data):
        lines = inp_data.split('\n')

        self.node_count = int(lines[0])

        self.points = []
        for i in range(1, self.node_count + 1):
            line = lines[i]
            parts = line.split()
            self.points.append(Point(float(parts[0]), float(parts[1])))
        # self.dist = [[0]*self.node_count for _ in range(self.node_count)]
        # for ii in range(self.node_count):
        #     for jj in range(ii):
        #         self.dist[ii][jj] = math.sqrt((self.points[ii].x - self.points[jj].x)**2
        #                                       + (self.points[ii].y - self.points[jj].y)**2)*100
        #         self.dist[jj][ii] = self.dist[ii][jj]

    def distance(self, p1, p2):
        return math.sqrt((self.points[p1].x - self.points[p2].x)**2
                         + (self.points[p1].y - self.points[p2].y)**2)*1000

    def __len__(self):
        return self.node_count


def solve_it(inp_data):
    # Modify this code to run your optimization algorithm

    # parse the input
    inp = Points(inp_data)
    tsp_size = len(inp)
    num_routes = 1

    routing = pywrapcp.RoutingModel(tsp_size, num_routes, 0)
    dist_callback = inp.distance
    routing.SetArcCostEvaluatorOfAllVehicles(dist_callback)
    search_params = pywrapcp.RoutingModel.DefaultSearchParameters()
    search_params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    search_params.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.SIMULATED_ANNEALING)
    search_params.time_limit_ms = 400000
    start_time = time.time()
    assignment = routing.SolveWithParameters(search_params)
    time_taken = time.time() - start_time
    opt = 1 if time_taken < 999 else 0
    if assignment:
        index = routing.Start(0)
        route = ''
        while not routing.IsEnd(index):
            route += str(index) + ' '
            index = assignment.Value(routing.NextVar(index))
        output_data = '%.2f' % (assignment.ObjectiveValue()/1000.0) + ' ' + str(opt) + '\n'
        output_data += route
    else:
        output_data = ''
    return output_data


if __name__ == '__main__':
    import sys
    if len(sys.argv) > 1:
        file_location = sys.argv[1].strip()
        with open(file_location, 'r') as input_data_file:
            input_data = input_data_file.read()
        print(solve_it(input_data))
    else:
        print('This test requires an input file.  Please select one from the data directory. (i.e. python solver.py'
              './data/tsp_51_1)')

