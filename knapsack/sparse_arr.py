import math
import numpy
import bisect
from  itertools import izip
class SparseArr(object):
    def __init__(self, capacity):
        self.capacity = capacity
        self.trans = [0]
        self.vals = [0]


    def __getitem__(self, item):
        ind = bisect.bisect_right(self.trans, item)
        if ind ==0:
            return self.vals[0]

        return self.vals[ind-1]


    def __len__(self):
        try:
            return len(self.trans)
        except AttributeError:
            return self.capacity

    def add_trans(self, ind, val):
        self.trans.append(ind)
        self.vals.append(val)

    def __iter__(self):
        return iter(izip(self.trans, self.vals))

    def successor(self, wt, val):
        trans = sorted(set(self.trans + [_ + wt for _ in self.trans]))
        trans = [_ for _ in trans if _ <= self.capacity]
        ret = SparseArr(self.capacity)
        last_val = 0
        tran_ind = 0
        tran_m_wt_ind=0
        for tran in trans:
            try:
                while(self.trans[tran_ind] <= tran):
                    tran_ind += 1
            except IndexError:
                tran_ind = len(self.trans)
            if tran >= wt:
                try:
                    while self.trans[tran_m_wt_ind] <= tran -wt:
                        tran_m_wt_ind += 1
                except IndexError:
                    tran_m_wt_ind = len(self.trans)

                new_val = max(self.vals[tran_ind - 1], self.vals[tran_m_wt_ind - 1] + val)
            else:
                new_val = self.vals[tran_ind - 1]

            if new_val > last_val:
                ret.add_trans(tran, new_val)
                last_val = new_val
        return ret
    def __repr__(self):
        return "sp_arr:\n" + "\n".join(str(_) for _ in izip(self.trans, self.vals))

if __name__ == "__main__":
    sp = SparseArr(100)
    sp.add_trans(10,20)
    sp2 = sp.successor(5,15)
    print sp
    print sp2
    print sp2[0], sp2[3], sp2[5], sp2[10], sp2[100]

