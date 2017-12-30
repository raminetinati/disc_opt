#!/usr/bin/python
# -*- coding: utf-8 -*-

from __future__ import print_function
from collections import namedtuple
import math
import heapdict
from ortools.linear_solver import pywraplp

Point = namedtuple("Point", ['x', 'y'])
Facility = namedtuple("Facility", ['index', 'setup_cost', 'capacity', 'location'])
Customer = namedtuple("Customer", ['index', 'demand', 'location'])
MAX_COST = 1e99

def length(point1, point2):
    return math.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)


def solve_mip(facs, custs):
    solver = pywraplp.Solver('SolveFac', pywraplp.Solver.CBC_MIXED_INTEGER_PROGRAMMING)
    fac_vars = [solver.IntVar(0.0, 1.0, 'y_' + str(ii)) for ii in range(len(facs))]
    cust_vars = []
    for ii, fac in enumerate(facs):
        cust_vars.append([solver.IntVar(0.0, 1.0, 'x_' + str(ii) + "_" + str(jj))
                          for jj in range(len(custs))])

    # customer only goes to 1 facitily
    for ii, cust in enumerate(custs):
        con = solver.Constraint(1,1)
        for jj, fac in enumerate(facs):
            con.SetCoefficient(cust_vars[jj][ii], 1)

    #facility open constraint
    for ii,fac in enumerate(facs):
        for jj, cust in enumerate(custs):
            con = solver.Constraint(0, 1)
            con.SetCoefficient(cust_vars[ii][jj],-1)
            con.SetCoefficient(fac_vars[ii], 1)


    #capacity constraint
    for ii, fac in enumerate(facs):
        con = solver.Constraint(0, fac.capacity)
        for jj, cust in enumerate(custs):
            con.SetCoefficient(cust_vars[ii][jj], cust.demand)
    #objective
    objective = solver.Objective()
    for ii, fac in enumerate(facs):
        objective.SetCoefficient(fac_vars[ii], fac.setup_cost)
    for ii, fac in enumerate(facs):
        for jj, cust in enumerate(custs):
            objective.SetCoefficient(cust_vars[ii][jj],
                                     length(fac.location, cust.location))
    objective.SetMinimization()
    solver.set_time_limit(3600*1000)
    print('Starting solver')
    result_status = solver.Solve();
    out = str(solver.Objective().Value())

    if result_status == pywraplp.Solver.OPTIMAL:
        out += ' 1\n'
    else:
        if result_status != pywraplp.Solver.FEASIBLE:
            return simple_soln(custs, facs)
        out += ' 0\n'
    cust_vals = ['0']*len(custs)
    for ii, var_vals in enumerate(cust_vars):
        for jj,val in enumerate(var_vals):
            if val.solution_value() == 1:
                cust_vals[jj] = str(ii)
    out += " ".join(cust_vals)
    return out


def parse_inp(input_data):
    # parse the input
    lines = input_data.split('\n')

    parts = lines[0].split()
    facility_count = int(parts[0])
    customer_count = int(parts[1])

    facilities = []
    for i in range(1, facility_count + 1):
        parts = lines[i].split()
        facilities.append(Facility(i - 1, float(parts[0]), int(parts[1]), Point(float(parts[2]), float(parts[3]))))

    customers = []
    for i in range(facility_count + 1, facility_count + 1 + customer_count):
        parts = lines[i].split()
        customers.append(Customer(i - 1 - facility_count, int(parts[0]), Point(float(parts[1]), float(parts[2]))))
    return facilities, customers


def simple_soln(customers, facilities):
    solution = [-1]*len(customers)
    capacity_remaining = [f.capacity for f in facilities]

    facility_index = 0
    for customer in customers:
        if capacity_remaining[facility_index] >= customer.demand:
            solution[customer.index] = facility_index
            capacity_remaining[facility_index] -= customer.demand
        else:
            facility_index += 1
            assert capacity_remaining[facility_index] >= customer.demand
            solution[customer.index] = facility_index
            capacity_remaining[facility_index] -= customer.demand

    used = [0]*len(facilities)
    for facility_index in solution:
        used[facility_index] = 1

    # calculate the cost of the solution
    obj = sum([f.setup_cost*used[f.index] for f in facilities])
    for customer in customers:
        obj += length(customer.location, facilities[solution[customer.index]].location)

    # prepare the solution in the specified output format
    output_data = '%.2f' % obj + ' ' + str(0) + '\n'
    output_data += ' '.join(map(str, solution))
    return output_data


class CustomerC(object):
    def __init__(self, index, demand, location):
        self.demand = demand
        self.location =location
        self.curr_fac = None
        self.index = index
        self.prop_fac = None
        self.curr_cost = MAX_COST
        self.prop_cost = MAX_COST
        self.facs = {}
    def comp_costs(self, facs):
        self.facs = facs
        assert type(facs) == dict
        costs = [(length(self.location, fac.location) + fac.cost, fac) for fac in facs.values()]
        self.costHeap = heapdict.heapdict()
        for cost, fac in costs:
            self.costHeap[fac.index] = cost

        for fac in facs.values():
            fac.cust_costs[self.index] = self.costHeap[fac.index]
        self.prop_fac, self.prop_cost = self.costHeap.peekitem()
        facs[self.prop_fac].prop_custs.add(self)

    def update_cost(self, index, delta, cust_heap):

        if delta < 0:
            if (self.facs[index].capacity - self.facs[index].filled_cap) >= self.demand:
                self.costHeap[index] += delta
            while True:
                best_ind, cost = self.costHeap.peekitem()
                if best_ind != self.prop_fac:
                    fac = self.facs[best_ind]
                    if self.demand <= (fac.capacity - fac.filled_cap):
                        self.prop_fac = best_ind
                        self.prop_cost = cost
                        cust_heap[self.index] = (cost, self)
                        self.facs[best_ind].prop_custs.add(self)
                        break
                    else:
                        self.costHeap[best_ind] = MAX_COST

                else:
                    break
        else:
            self.costHeap[index] += delta
            if index == self.prop_fac:
                while True:
                    best_ind, cost = self.costHeap.peekitem()
                    if (best_ind != index):
                        fac = self.facs[best_ind]
                        if self.demand <= (fac.capacity - fac.filled_cap):
                            self.prop_fac = best_ind
                            self.prop_cost = cost
                            self.facs[best_ind].prop_custs.add(self)
                            cust_heap[self.index] = (self.prop_cost, self)
                            break
                        else:
                            self.costHeap[best_ind] = MAX_COST
                    else:
                        break
    def find_fac(self):
        raise ValueError, "this should not be called"

class FacilityC(object):
    def __init__(self, index, location, capacity, cost):
        self.capacity = capacity
        self.cost = cost
        self.location = location
        self.index = index
        self.is_open = False
        self.customers = {}
        self.cust_costs = {}
        self.filled_cap = 0
        self.prop_custs = set()
    def open(self, custs, cust_heap):
        self.is_open = True
        for cust in custs.values():
            cust.update_cost(self.index, -self.cost, cust_heap)

    def close(self, cust_heap, custs):
        if not self.is_open:
            raise ValueError("trying to close an already closed facility")
        if self.customers:
            raise ValueError("Cannot close a facility that is not empty")
        self.is_open = False
        for cust in custs.values():
            cust.update_cost(self.index , self.cost, cust_heap)

    def add_cust(self, cust, cust_heap):
        if (self.capacity - self.filled_cap) < cust.demand:
            raise ValueError("customer demand cannot be filled")
        self.prop_custs.remove(cust)
        self.customers[cust.index] = cust
        self.filled_cap += cust.demand
        cap_left = self.capacity - self.filled_cap
        rem_custs = set()
        for cust in self.prop_custs:
            if cust.demand > cap_left:
                cust.update_cost(self.index, MAX_COST-cust.prop_cost, cust_heap)
                rem_custs.add(cust)
        self.prop_custs -= rem_custs
    def rem_cust(self, cust, cust_heap, custs):
        self.customers.pop(cust.index)
        self.filled_cap -= cust.demand
        if not self.customers:
            self.close(cust_heap, custs)
        else:
            for cust in custs.values():
                if cust.index not in self.customers:
                    if cust.demand <= (self.capacity - self.filled_cap):
                        if cust.prop_cost > self.cust_costs[cust.index]:
                            cust.prop_cost = self.cust_costs[cust.index]
                            cust.prop_fac = self.index
                            cust_heap[cust.index] = (cust.prop_cost, cust)
                            self.prop_custs.add(cust)


class FacLoc(object):
    def __init__(self, data):
        facs, custs, = parse_inp(data)
        self.facs = dict((fac.index, FacilityC(fac.index, fac.location, fac.capacity, fac.setup_cost))
                     for fac in facs)
        self.custs = dict((cust.index, CustomerC(cust.index, cust.demand, cust.location)) for cust
                      in custs)
        [cust.comp_costs(self.facs) for cust in self.custs.values()]
    def compute_cost(self):
        cost = 0
        for fac in self.facs.values():
            if fac.is_open:
                cost += fac.cost
        for cust in self.custs.values():
            fac = cust.curr_fac
            cost += length(fac.location, cust.location)
        return cost
    def compute_out(self):
        output_data = '%.2f' % self.compute_cost() + ' ' + str(0) + '\n'
        output_data += ' '.join(map(str, (cust.curr_fac.index for ind, cust in sorted(self.custs.items()))))
        return output_data

    def greedy_fill(self):
        self.cust_heap = heapdict.heapdict()
        for cust in self.custs.values():
            self.cust_heap[cust.index] = (cust.prop_cost, cust)
        while self.cust_heap:
            print("Len = ", len(self.cust_heap))
            index, cust = self.cust_heap.popitem()
            prop_cost, cust = cust
            fac = self.facs[cust.prop_fac]
            if not fac.is_open:
                fac.open(self.custs, self.cust_heap)
            if cust.curr_fac is not fac:
                if ((fac.capacity - fac.filled_cap) >= cust.demand):
                    fac.add_cust(cust, self.cust_heap)
                    if cust.curr_fac is not None:
                        cust.curr_fac.rem_cust(cust, self.cust_heap, self.custs)
                    cust.curr_fac = fac
                    cust.curr_cost = prop_cost
                else:
                    raise ValueError, "Trying to send customer to facility without enough capacity"

        for fac in self.facs.values():
            if (fac.filled_cap > fac.capacity) or (fac.filled_cap < 0):
                raise  ValueError, "Inconsistent state"
            test_cap = sum(cust.demand for cust in fac.customers.values())
            assert test_cap == fac.filled_cap
            for cust in fac.customers.values():
                if cust.curr_fac is not fac:
                    raise ValueError, "!Inconsistent state"

def solve_it(input_data):
    # Modify this code to run your optimization algorithm
    facilities, customers = parse_inp(input_data)

    # build a trivial solution
    # pack the facilities one by one until all the customers are served
    if (len(customers) * len(facilities) < 200*800+1):
        output_data = solve_mip(facilities,customers)
    else:
        fac_loc = FacLoc(input_data)
        fac_loc.greedy_fill()
        output_data = fac_loc.compute_out()
    return output_data



if __name__ == '__main__':
    import sys
    if len(sys.argv) > 1:
        file_location = sys.argv[1].strip()
        with open(file_location, 'r') as input_data_file:
            input_data = input_data_file.read()
        print(solve_it(input_data))
    else:
        print('This test requires an input file.  Please select one from the data directory.',
               '(i.e. python solver.py ./data/fl_16_2)')

