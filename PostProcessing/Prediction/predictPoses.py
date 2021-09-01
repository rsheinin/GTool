import argparse
import configparser

import math
import os
import sys

import copy

sys.path.append(os.path.join(os.path.dirname(__file__), '..\\..\\Common\\python\\PosesReader'))

from PosesReader import Format_File, Poses_Container, Poses_Utils


def mult_quat( r, q):
    '''

    :param r: first quaternioun
    :param q: second quaternioun
    :return: multiple results of r and q
    '''
    qr=[0,0,0,0]
    qr[0] = r[3] * q[0] + r[0] * q[3] + r[1] * q[2] - r[2] * q[1]
    qr[1] = r[3] * q[1] + r[1] * q[3] + r[2] * q[0] - r[0] * q[2]
    qr[2] = r[3] * q[2] + r[2] * q[3] + r[0] * q[1] - r[1] * q[0]
    qr[3] = r[3] * q[3] - r[0] * q[0] - r[1] * q[1] - r[2] * q[2]
    return qr

def quaternionExp(v):
    w = [v[0]/2.0, v[1]/2.0, v[2]/2.0 ]
    th2 = w[0]*w[0] + w[1]*w[1] + w[2]*w[2]
    th = math.sqrt(th2)
    c = math.cos(th)
    s = 1.0- (1.0 / 6.0 * th2) if th2 < math.sqrt(120* sys.float_info.epsilon) else math.sin(th)/th
    Q = [ s*w[0], s*w[1], s*w[2], c ]
    return Q

def normalize(q):
    '''

    :param q: quaternioun
    :return: the normalize quaternioun
    '''
    norm = math.sqrt(pow(q[0], 2)+pow(q[1], 2)+pow(q[2], 2)+pow(q[3], 2))
    return [q[0]/norm, q[1]/norm, q[2]/norm, q[3]/norm]

def predict_poses(poses, predict_time_in_ms, time_factor_to_ms):
    '''

    :param poses_container: poses container, contain the old poses
    :param predict_time_in_sec: time to predict the poses (in ms)
    :return: new poses list after prediction
    '''

    # change the prediction time to the format units.
    # !!!!!!!!!!!! important !!!!!!!!!!!!- corrently the velocity & the accel are m/s, m/s^2, r/s, r/s^2 so we should use prediction in seconds.
    predict_in_sec = predict_time_in_ms * 0.001
    predict_in_format = predict_time_in_ms / time_factor_to_ms
    new_poses = []
    for pose in poses:

        new_pose = pose
        # update the timestamp to the new one
        new_pose['timestamp']= pose['timestamp']+ predict_in_format
        # translation
        new_pose['x'] = predict_in_sec * (predict_in_sec / 2.0 * pose['accel_x'] + pose['vel_x']) + pose['x']
        new_pose['y'] = predict_in_sec * (predict_in_sec / 2.0 * pose['accel_y'] + pose['vel_y']) + pose['y']
        new_pose['z'] = predict_in_sec * (predict_in_sec / 2.0 * pose['accel_z'] + pose['vel_z']) + pose['z']

        # rotation
        W = [ predict_in_sec * (predict_in_sec/2.0 * pose['ang_accel_x'] + pose['ang_vel_x']),
              predict_in_sec * (predict_in_sec/2.0 * pose['ang_accel_y'] + pose['ang_vel_y']),
              predict_in_sec * (predict_in_sec/2.0 * pose['ang_accel_z'] + pose['ang_vel_z'])]

        quatW = quaternionExp(W)
        new_quat = normalize(mult_quat(quatW,[pose['qx'], pose['qy'], pose['qz'], pose['qw']]))

        new_pose['qx'] = new_quat[0]
        new_pose['qy'] = new_quat[1]
        new_pose['qz'] = new_quat[2]
        new_pose['qw'] = new_quat[3]

        new_poses.append(new_pose)

    return new_poses





def parseArguments():
    parser = argparse.ArgumentParser(
        description="metrics runner")

    parser.add_argument("--in_file_path", type=str, default=None, help="input file path")
    parser.add_argument("--in_config_file", type=str, default=None, help="config file of the given data")
    parser.add_argument("--out_file_path", type=str, default='predicted_poses.txt', help="output file path")
    parser.add_argument("--out_config_file", type=str, default=None, help="config file for output data")
    parser.add_argument("--prediction_time_in_ms", type=float, default=0.04, help="prediction time in milliseconds")

    args = parser.parse_args()

    if not os.path.isfile(args.in_file_path):
        print("Error: given in_file_path is wrong.", args.in_file_path )
        exit(-1)
    if not os.path.isfile(args.in_config_file):
        print("Error: given in_config_file is wrong.", args.in_config_file)
        exit(-1)
    if args.out_config_file and not os.path.isfile(args.out_config_file):
        print("Error: given out_config_file is wrong.", args.in_config_file)
        exit(-1)
    return args


if __name__ == "__main__":
    args = parseArguments()

    in_config = configparser.ConfigParser()
    in_config.read(args.in_config_file)
    in_format = Format_File(in_config)

    # load poses file
    poses_container = Poses_Container(filename = args.in_file_path, format = in_format)
    poses_container.read_pose_file()

    print('using prediction time of ' ,args.prediction_time_in_ms, ' ms')
    time_factor_to_ms = 1 if 'time_factor_to_ms' not in poses_container.format.general_definition else float(poses_container.format.general_definition['time_factor_to_ms'])
    poses_container.set_poses(predict_poses(poses = poses_container.get_poses(), predict_time_in_ms=args.prediction_time_in_ms, time_factor_to_ms=time_factor_to_ms))

    out_format = copy.deepcopy(in_format)
    if args.out_config_file:
        out_config = configparser.ConfigParser()
        out_config.read(args.out_config_file)
        out_format = Format_File(out_config)

    poses_container.write_pose_file(output_filename = args.out_file_path, format= out_format)
    print('predicted poses writen into ', args.out_file_path)
