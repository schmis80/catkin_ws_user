import numpy as np
from scipy.spatial import KDTree()

def read_lanes(lane_file):
    arr = []
    with open(lane_file) as l_file:
        for line in l_file:
            if line.startswith('1.'):
                x, y = line.split('\t')[1:3]
                yield x, y

def build_kdtree(fname):
    
    
