#!/usr/bin/python
# -*- coding: utf-8 -*-
from ortools.constraint_solver import pywrapcp
import time


class Graph(object):

    def __init__(self, inp):
        lines = inp.split('\n')

        first_line = lines[0].split()
        self.node_count = int(first_line[0])
        self.edge_count = int(first_line[1])
        self.neighbors = dict((_, set()) for _ in range(self.node_count))
        self.edges = []
        self.solver = None
        self.values = range(self.node_count)
        self.vars = None
        self.goal = None
        self.phase = None
        self.limit = None
        for i in range(1, self.edge_count + 1):
            line = lines[i]
            parts = [int(_) for _ in line.split()]
            self.edges.append((parts[0], parts[1]))
            self.neighbors[parts[0]].add(parts[1])
            self.neighbors[parts[1]].add(parts[0])

    def create_solver(self, limit=-1):
        if limit == -1:
            limit = max(len(_) for _ in self.neighbors.values())

        self.solver = pywrapcp.Solver("Graph-color")
        nodes = sorted(self.neighbors.items(), key=lambda x: len(x[1]), reverse=True)
        self.vars = dict((item[0], self.solver.IntVar(0, min(ii, limit),
                         "n%d" % item[0])) for ii, item in enumerate(nodes))
        self.goal = self.solver.Minimize(self.solver.Max(self.vars.values()), 1)
        self.phase = self.solver.Phase(self.vars.values(),
                                       self.solver.CHOOSE_MIN_SIZE_HIGHEST_MIN, self.solver.ASSIGN_MIN_VALUE)
        for edge in self.edges:
            self.solver.Add(self.vars[edge[0]] != self.vars[edge[1]])
        #self.add_cliques()
        self.limit = self.solver.TimeLimit(1000*1000)

    def add_cliques(self):
        st_time = time.time()
        all_cliques = set()
        for edge in self.edges:
            curr_clique = set(edge)
            curr_nodes = self.neighbors[edge[0]] | self.neighbors[edge[1]]
            while curr_nodes:
                node = curr_nodes.pop()
                if node in curr_clique:
                    continue
                if all(node in self.neighbors[_] for _ in curr_clique):
                    curr_clique.add(node)
                    curr_nodes |= self.neighbors[node]
            if len(curr_clique) > 2:
                if tuple(curr_clique) in all_cliques:
                    continue
                all_cliques.add(tuple(curr_clique))
                self.solver.AllDifferent([self.vars[_] for _ in curr_clique])
            if (time.time() - st_time) > 10:
                break

    def solve(self):
        time_st = time.time()
        collector = self.solver.LastSolutionCollector()
        collector.Add(self.vars.values())
        self.solver.Solve(self.phase, collector, self.goal, self.limit)
        time_stop = time.time()
        print "time taken:", time_stop - time_st
        self.values = range(self.node_count)

        new_values = [collector.Value(0, _[1]) for _ in sorted(self.vars.items())]
        print "solution : ", str(max(new_values))
        if max(self.values) > max(new_values):
            self.values = new_values

        output = str(max(self.values)+1) + " " + "0\n"
        return output + " ".join(str(x) for x in self.values)


def solve_it(inp):
    # Modify this code to run your optimization algorithm

    # parse the input

    # build a trivial solution
    # every node has its own color
    graph = Graph(inp)
    graph.create_solver()
    return graph.solve()


if __name__ == '__main__':
    import sys
    if len(sys.argv) > 1:
        file_location = sys.argv[1].strip()
        with open(file_location, 'r') as input_data_file:
            input_data = input_data_file.read()
        print(solve_it(input_data))
    else:
        print('This test requires an input file.'  
              'Please select one from the data directory. (i.e. python solver.py ./data/gc_4_1)')
