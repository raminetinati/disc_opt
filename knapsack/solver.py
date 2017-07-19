#!/usr/bin/python
# -*- coding: utf-8 -*-
import numpy
import math
from collections import namedtuple
Item = namedtuple("Item", ['index', 'value', 'weight'])
import sparse_arr

def solve_dyn_sparse(capacity, items):
    items = sorted(items, key=lambda x : x.weight/x.value)
    table = [sparse_arr.SparseArr(capacity)]
    for jj, item in enumerate(items, 1):
        table.append(table[-1].successor(item.weight, item.value))
        print jj
        print len(table[-1])
    res = str(table[-1][capacity]) + " 1" + "\n"
    index = capacity
    res_item = []
    for ii in range(len(items), 0, -1):
        if table[ii][index] == table[ii-1][index]:
            res_item. append((items[ii-1].index, '0'))
        else:
            res_item.append((items[ii-1].index, '1'))
            index -= items[ii - 1].weight
    res_item = " ".join(str(x[1]) for x in sorted(res_item))
    return res + res_item


MAX_MEM = 500000000


def solv_dyn_approx(capacity, items):
    if capacity * len(items) > MAX_MEM:
        approx = True
        cap_sc = capacity*1.0*len(items)/MAX_MEM
        capacity = int(math.floor(capacity/cap_sc))
        for ii, item in enumerate(items):
            items[ii] = Item(item.index, item.value, int(math.ceil(item.weight/cap_sc)))
    else:
        approx = False
    table = numpy.zeros((capacity+1, len(items)+1), dtype=numpy.long)
    for jj, item in enumerate(items, 1):
        print jj
        for ii in range(1, capacity+1):
            if ii < item.weight:
                table[ii, jj] = table[ii, jj-1]
            else:
                table[ii, jj] = max(table[ii, jj-1], table[ii-item.weight, jj - 1] + item.value)
    res = str(table[capacity, len(items)]) + " " + str(int( not approx)) + "\n"
    index = capacity
    res_item = ''
    for ii in range(len(items), 0, -1):
        if table[index, ii] == table[index, ii-1]:
            res_item = '0 ' + res_item
        else:
            res_item = '1 ' + res_item
            index -= items[ii - 1].weight
    return res + res_item


def solv_br_bound():
    pass
def parse(input_data):
    lines = input_data.split('\n')

    firstLine = lines[0].split()
    item_count = int(firstLine[0])
    capacity = int(firstLine[1])

    items = []

    for i in range(1, item_count + 1):
        line = lines[i]
        parts = line.split()
        items.append(Item(i - 1, int(parts[0]), int(parts[1])))
    return capacity, items, item_count

def solve_it(input_data):
    # Modify this code to run your optimization algorithm
    capacity, items, item_count = parse(input_data)
    # parse the input

    # a trivial greedy algorithm for filling the knapsack
    # it takes items in-order until the knapsack is full
    #print solv_dyn_approx(capacity, items)
    #print solve_dyn_sparse(capacity, items)
    #return solve_dyn_sparse(capacity, items)
    return solve_dyn_sparse(capacity, items)


if __name__ == '__main__':
    import sys
    if len(sys.argv) > 1:
        file_location = sys.argv[1].strip()
        with open(file_location, 'r') as input_data_file:
            input_data = input_data_file.read()
        print(solve_it(input_data))
    else:
        print('This test requires an input file.  Please select one from the data directory. (i.e. python solver.py ./data/ks_4_0)')

