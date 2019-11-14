#!/usr/bin/env python
import sys, getopt
from tf.transformations import *
#import transformations
import numpy as np

if __name__ == '__main__':

    options, remainder = getopt.getopt(sys.argv[1:], 'x:y:z:R:P:Y:v')

    x, y, z, R, P, Y = 0., 0., 0., 0., 0., 0.

    for opt, arg in options:
        if opt in ('-x'):
            x = float(arg)
        if opt in ('-y'):
            y = float(arg)
        if opt in ('-z'):
            z = float(arg)
        if opt in ('-R'):
            R = float(arg)
        if opt in ('-P'):
            P = float(arg)
        if opt in ('-Y'):
            Y = float(arg)
        if opt in ('-v'):
            verbose = True
        else:
            verbose = False

    origin, xaxis, yaxis, zaxis = [0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]
    Rx = rotation_matrix(R, xaxis)
    Ry = rotation_matrix(P, yaxis)
    Rz = rotation_matrix(Y, zaxis)
    R = concatenate_matrices(Rx, Ry, Rz)
    T = translation_matrix([x, y, z])
    M = np.dot(T,R)

    M_inv = np.linalg.inv(M)

    if verbose:
    	print('M:')
    	print(M)
    	print('M_inv:')
    	print(M_inv)

    # get quaternion in x,y,z,w
    q_inv = quaternion_from_matrix(M_inv)
    # get translation
    x_inv, y_inv, z_inv = M_inv[0,-1], M_inv[1,-1], M_inv[2,-1]

    print("%0.4f %0.4f %0.4f %0.4f %0.4f %0.4f %0.4f" %(x_inv, y_inv, z_inv, q_inv[0], q_inv[1], q_inv[2], q_inv[3]))
