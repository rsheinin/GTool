# -*- coding: utf-8 -*-
"""
Created on Mon Jul 03 20:22:39 2017

@author: gbaruch
"""

import sys
import csv
import numpy as np

def quaternion_from_matrix(matrix):
    """Return quaternion from rotation matrix."""
    M = np.array(matrix, dtype=np.float64, copy=False)

    # symmetric matrix K
    K = np.array([[M[0, 0]-M[1, 1]-M[2, 2],  0.0,                      0.0,                     0.0],
                  [M[0, 1]+M[1, 0],          M[1, 1]-M[0, 0]-M[2, 2],  0.0,                     0.0],
                  [M[0, 2]+M[2, 0],          M[1, 2]+M[2, 1],          M[2, 2]-M[0, 0]-M[1, 1], 0.0],
                  [M[2, 1]-M[1, 2],          M[0, 2]-M[2, 0],          M[1, 0]-M[0, 1],         M[0, 0]+M[1, 1]+M[2, 2]]])
    K /= 3.0
    # quaternion is eigenvector of K that corresponds to largest eigenvalue
    w, V = np.linalg.eigh(K)
    q = V[:, np.argmax(w)]
    if q[3] < 0.0:
        np.negative(q, q)
    return q

def read_csv_floats(filename, header=False):
    rows = []
    dropped_count = 0
    with open(filename, 'rb') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        if header:
            reader.next() # skip header
        for row in reader:
            try:
                float_row = [float(r) for r in row if len(r) > 0]
                rows.append(float_row)
            except ValueError:#cells with 'nan' and the like
                dropped_count = dropped_count + 1
                if dropped_count % 100 == 0:
                    print 'dropped_count: ', dropped_count 
                if dropped_count > 10000:
                    print 'Over 10K invalid lines. Aborting.'
                    raise
                pass

    return np.array(rows)

def parse_gt_csv(rows, drop_zeros=False):

    dropped_count = 0
    T_axis = [7, 11, 15]
    R_axis = [4, 5, 6, 8, 9, 10, 12, 13, 14]
    for row in rows:
        if drop_zeros:
            if np.count_nonzero(row[4:16]) < 6:
                dropped_count = dropped_count + 1
                if dropped_count % 100 == 0:
                    print 'dropped_count: ', dropped_count
                if dropped_count > 10000:
                    print 'Over 10K lines or zeros. Aborting.'
                    return
                continue
            if row[3] != 1:
                continue
        timestamp = row[1] / 1000 #ms to sec
        R = row[R_axis]

        quat = quaternion_from_matrix(np.reshape(R, (3, 3)))
        T = row[T_axis]/1000. #meters to mm
        yield np.concatenate([[timestamp], T, quat])

def write_tum(output_tum, output_rows):
    with open(output_tum, 'wb') as out_gt:
        writer = csv.writer(out_gt, delimiter=' ')
        for row in output_rows:
            writer.writerow(['{:.9f}'.format(i) for i in row])

def main(csv_filename, tum_filename):
    '''Reading the input csv, converting and writing to the output file'''
    in_rows = read_csv_floats(csv_filename, True)
    out_rows = parse_gt_csv(in_rows, True)
    write_tum(tum_filename, out_rows)

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print 'Usage: ', sys.argv[0], ' <in_gt.csv> <out_gt.tum>'
        sys.exit(1)
    main(sys.argv[1], sys.argv[2])
