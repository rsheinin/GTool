import cv2
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
from scipy import interpolate
from scipy.optimize import minimize
import scipy.signal

import os, os.path as osp
import re
import pickle
import itertools

from glob import glob
from datetime import datetime
from collections import defaultdict, OrderedDict


class Utils:
    @staticmethod
    def load_pkl(path):
        with open(path, 'rb') as f:
            data = pickle.load(f)

        return data

    @staticmethod
    def save_pkl(path, data):
        with open(path, 'wb') as f:
            pickle.dump(data, f)

    @staticmethod
    def homogeneous(r, t=[0, 0, 0]):
        # return Utils.homogeneous_nd(np.asarray(r), np.asarray(t))[0]
        return np.concatenate([np.concatenate([np.asarray(r), np.asarray(t)[..., None]], axis=-1),
                               np.asarray([0, 0, 0, 1])[None, ...]])

    @staticmethod
    def homogeneous_nd(r_, t_, n_=None):
        n = n_ or len(r_)
        r = np.asarray(r_)
        t = np.asarray(t_)[..., None]
        return np.concatenate([np.concatenate([r, t], axis=2), np.array([[0, 0, 0, 1]] * n)[:, None, :]], axis=1)

    @staticmethod
    def inv(m):
        return np.linalg.inv(m)

    @staticmethod
    def euler2matrix(eu, order='xyz', degrees=True):
        return R.from_euler(order, eu, degrees=degrees).as_matrix()

    @staticmethod
    def matrix2euler(m, order='xyz', degrees=True):
        return R.from_matrix(m[..., :3, :3]).as_euler(order, degrees=degrees)

    @staticmethod
    def matrix2mag(m):
        return np.linalg.norm(R.from_matrix(m[..., :3, :3]).as_rotvec(), axis=1)

    @staticmethod
    def trans_dist(m):
        return np.linalg.norm(m[..., :3, 3], axis=1)

    @staticmethod
    def RMSe(dist):
        d = dist[~np.isnan(dist)]
        n = len(d)
        return np.sqrt(np.sum((d ** 2) / n))
        return (((d ** 2) / len(d)).sum()) ** (1 / 2)

    @staticmethod
    def average_transformation(pose_list):
        p_ = np.asarray(pose_list)
        rot = R.from_matrix(p_[..., :3, :3]).mean().as_matrix()
        trans = np.mean(p_[..., :3, 3], axis=0)
        return Utils.homogeneous(rot, trans)




class ImageChessboardPose:

    def init_from_folder(self, path):
        self.board_h, self.board_w, self.board_mm_h, self.board_mm_w = \
            Utils.load_pkl(osp.join(path, 'board_params.pkl'))
        # self.board_h, self.board_w, self.board_mm_h, self.board_mm_w = \
        #     pd.read_csv(osp.join(path, 'board_params.csv'), header=None).values.flatten()
        self.board_points3d = self.make_points3d(self.board_h, self.board_w, self.board_mm_h, self.board_mm_w)

        self.color_cam_mat = Utils.load_pkl(osp.join(path, 'cam.pkl'))
        self.color_cam_dist = Utils.load_pkl(osp.join(path, 'dist.pkl'))

    def get_pose(self, image):
        pose = self.image2pose(image, self.color_cam_mat, self.color_cam_dist,
                               self.board_h, self.board_w, self.board_points3d, draw_coord=True)
        cv2.imshow('im pose', image)
        cv2.waitKey(1)
        return pose
        return self.image2pose(image, self.color_cam_mat, self.color_cam_dist,
                               self.board_h, self.board_w, self.board_points3d, draw_coord=True)

    @staticmethod
    def reprojection_error(imgpoints, objpoints, rvecs, tvecs, mtx, dist):
        imgpoints2, _ = cv2.projectPoints(objpoints, rvecs, tvecs, mtx, dist)
        error = cv2.norm(imgpoints, imgpoints2, cv2.NORM_L2) / len(imgpoints2)

        return error

    @staticmethod
    def make_points3d(board_h, board_w, sq_h, sq_w):
        objp = np.zeros((board_w * board_h, 3), np.float32)
        objp[:, :2] = np.mgrid[0:board_w, 0:board_h].T.reshape(-1, 2)
        objp[:, 0] *= sq_w
        objp[:, 1] *= sq_h

        return objp

    @staticmethod
    def draw_axis(img, corners, imgpts):
        corner = tuple(np.round(corners[0].ravel()).astype('int'))
        img = cv2.line(img, corner, tuple(imgpts[0].ravel().astype('int')), (255, 0, 0), 2)
        img = cv2.line(img, corner, tuple(imgpts[1].ravel().astype('int')), (0, 255, 0), 2)
        img = cv2.line(img, corner, tuple(imgpts[2].ravel().astype('int')), (0, 0, 255), 2)
        return img

    @staticmethod
    def image2pose(im, mtx, dist, board_h, board_w, points3d, draw_corners=False, draw_coord=False, coord_scale=50):

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)

        # Find the chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, (board_w, board_h), None)

        # If found, add object points, image points
        if ret == True:

            # Draw and display the corners
            if draw_corners:
                im = cv2.drawChessboardCorners(im, (board_w, board_h), corners, ret)

            points2d = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

            # Find the rotation and translation vectors.
            # _, rvecs, tvecs, inliers = cv2.solvePnPRansac(points3d, points2d, mtx, dist)
            _, rvecs, tvecs = cv2.solvePnP(points3d, points2d, mtx, dist)
            reproj_error = ImageChessboardPose.reprojection_error(points2d, points3d, rvecs, tvecs, mtx, dist)

            if draw_coord:
                # project 3D points to image plane
                axis = np.float32([[coord_scale, 0, 0], [0, coord_scale, 0], [0, 0, -coord_scale]]).reshape(-1, 3)
                axis_imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)

                im = ImageChessboardPose.draw_axis(im, corners, axis_imgpts)

            pose = np.eye(4)
            pose[:3, :3] = cv2.Rodrigues(rvecs)[0]
            pose[:3, 3] = tvecs.flatten()

            return pose, reproj_error, corners

        return None, None, None


class DataReader:
    def __init__(self, capture_folder, keys=['ts', 'color', 'accel', 'accel_ts', 'gyro', 'gyro_ts']):

        self.folder = capture_folder
        for k, v in self.iterate_frames_(capture_folder, keys).items():
            setattr(self, k, v)

    @staticmethod
    def iterate_frames_(folder, keys):
        files = [x[:-4] for x in os.listdir(folder) if re.match('\d+\.pkl', x)]
        files.sort()
        pkls = [os.path.join(folder, f'{x}.pkl') for x in files]

        data = defaultdict(lambda: [])

        for pkl in pkls:
            for key, val in zip(*DataReader.read_frame_(pkl, keys)):
                data[key] += [val]

        return {k: np.asarray(v) for k, v in data.items()}
        # todo: convertto np?
        return data

    @staticmethod
    def read_frame_(pkl_path, keys):
        data = Utils.load_pkl(pkl_path)

        return keys, [data[key] if key in data else None for key in keys]


class OptiReader:
    ROTMAT = ['r_0', 'r_1', 'r_2', 'r_3', 'r_4', 'r_5', 'r_6', 'r_7', 'r_8']
    QUAT = ['qx', 'qy', 'qz', 'qw']
    TRANS = ['x', 'y', 'z']
    ROTMAT_TRANS = ['r_0', 'r_1', 'r_2', 'x', 'r_3', 'r_4', 'r_5', 'y', 'r_6', 'r_7', 'r_8', 'z']

    def __init__(self, csv_path):
        self.df = self.filter(pd.read_csv(csv_path))

        self.ts = self.df.timestamp.values

        self.pose = np.column_stack([
            self.df[self.ROTMAT_TRANS].astype(np.float).values,
            np.asarray([[0, 0, 0, 1]] * len(self.df))
        ]).reshape((-1, 4, 4))
        self.error = self.df.error.values

        if True:
            idx = np.where(self.ts[1:] != self.ts[:-1])[0]
            self.ts = self.ts[idx]
            self.pose = self.pose[idx]
            self.error = self.error[idx]

    def filter(self, df):
        t_error = 0.1
        markers_in_scence_col_idx = 22
        markers_in_scence_col = df.columns[markers_in_scence_col_idx]
        markers_col = df.columns[markers_in_scence_col_idx - 1]
        rb_n_markers = 8

        df = df[df.tracked == 1]
        df = df[df.error < t_error]
        df = df[(df[markers_in_scence_col] <= rb_n_markers)]
        # df = df[(df[markers_in_scence_col] == df[markers_col])]

        # df = df.drop(np.where(df.timestamp.iloc[1:].values == df.timestamp.iloc[:-1].values))

        return df


class Trajectory:
    def __init__(self, timestamp, pose_list):
        self.timestamp = np.asarray(timestamp)
        self.pose = np.asarray(pose_list)

    def rel(self, idx=0, orig=None):
        eye = orig if orig is not None else self.pose[idx]
        return Utils.inv(eye) @ self.pose

    def project(self, calib=np.eye(4), orig=np.eye(4)):
        return Trajectory(self.timestamp, Utils.inv(calib) @ self.rel(orig=orig) @ calib)
        return Trajectory(self.timestamp, orig @ Utils.inv(calib) @ self.rel() @ calib)

    def interpolate(self, ts, doTrnasInterp=True):
        slerp = Slerp(self.timestamp, R.from_matrix(self.pose[:, :3, :3]))
        valid_ts = ts[(ts > self.timestamp.min()) & (ts < self.timestamp.max())]
        interp_r = slerp(valid_ts)
        interp_rotmat = interp_r.as_matrix()

        if doTrnasInterp:
            ### todo: interpolate.interp3d(self.ts)
            interp_tx = interpolate.interp1d(self.timestamp, self.pose[:, 0, 3])
            interp_ty = interpolate.interp1d(self.timestamp, self.pose[:, 1, 3])
            interp_tz = interpolate.interp1d(self.timestamp, self.pose[:, 2, 3])

            interp_t = np.column_stack([interp_tx(valid_ts), interp_ty(valid_ts), interp_tz(valid_ts)])

            return Trajectory(valid_ts, Utils.homogeneous_nd(interp_rotmat, interp_t))

        return Trajectory(valid_ts, interp_rotmat)


def calc_imu_rotation(accel_ts, accel, gyro_ts, gyro):
    gyro_sec_diff = np.diff(np.asarray(gyro_ts)) / 1000
    gyro_euler = (np.asarray(gyro)[1:] * gyro_sec_diff[..., None])

    z = np.arctan2(accel[:, 1], accel[:, 2])
    x = np.arctan2(accel[:, 0], (accel[:, 1] ** 2 + accel[:, 2] ** 2) ** 0.5)
    accel_euler = np.asarray([_ for _ in zip(x, itertools.repeat(0), z)])

    alpha = 0.99
    alpha = 0.90
    alpha = 0.98
    accel_alpha = np.asarray([alpha, 1, alpha])
    accel_beta = np.asarray([1 - alpha, 0, 1 - alpha])

    gyro_xyz_idx = [2, 1, 0]
    gyro_coeff = np.asarray([-1, -1, 1])

    def process_accel(euler_accel, theta=None):
        if theta is None:
            return euler_accel

        return theta * accel_alpha + euler_accel * accel_beta

    def process_gyro(gyro_euler, theta=None):
        if theta is None:
            return theta
        return theta + gyro_euler[gyro_xyz_idx] * gyro_coeff

    process = \
        [(ta, a, process_accel, 'a') for ta, a in zip(accel_ts, accel_euler)] + \
        [(tg, g, process_gyro, 'g') for tg, g in zip(gyro_ts[1:], gyro_euler)]

    process.sort(key=lambda x: x[0])

    imu_euler_rad_rot = {'ag': [], 'g': [], 'a': []}
    imu_rot_time = {'ag': [], 'g': [], 'a': []}
    theta = None
    for ts, data, func, dt in process:
        theta = func(data, theta)

        imu_euler_rad_rot['ag'] += [theta]
        imu_euler_rad_rot[dt] += [theta]

        imu_rot_time['ag'] += [ts]
        imu_rot_time[dt] += [ts]

    return imu_euler_rad_rot, imu_rot_time


def temporal_diff(ts1, traj1, ts2, traj2):
    def func_ta(x, t1, t2, d1, d2):
        slerp = Slerp(t2, R.from_matrix(d2[:, :3, :3]))
        new_ts = np.asarray(t1) + x
        mask = (new_ts > t2[0]) & (new_ts < t2[-1])
        interp = slerp(new_ts[mask]).as_matrix()

        return np.linalg.norm(Utils.matrix2mag(d1[mask]) - Utils.matrix2mag(interp))

        a = Utils.matrix2mag(d1[mask])
        b = Utils.matrix2mag(interp)

        s, e = 200, 400
        a = a[s:e]
        b = b[s:e]

        return np.linalg.norm((a - a.max()) - (b - b.max()))
        return Utils.RMSe((a - a.max()) - (b - b.max()))
        return Utils.RMSe(Utils.matrix2mag(d1[mask]) - Utils.matrix2mag(interp))

    res = minimize(func_ta, 0, (ts1 - ts1[0], ts2 - ts2[0], traj1, traj2), method='Nelder-Mead')  # good!!! # [3347.734]

    # # ts1[i] - ts1[0] + res.x = ts2[i] - t2[0]
    # ts1[i] = ts2[i] + (ts1[0] - ts2[0] - res.x)

    return ts1[0] - ts2[0] - res.x  # time to add to t2 clock


def temporal_diff2(ts1, traj1, ts2, traj2):
    def func_ta(x, t1, t2, d1, d2):
        bias, scale = x
        new_ts = t2 * scale + bias
        slerp = Slerp(new_ts, R.from_matrix(d2[:, :3, :3]))
        mask = (t1 > new_ts[0]) & (t1 < new_ts[-1])
        interp = slerp(t1[mask]).as_matrix()

        return np.linalg.norm(Utils.matrix2mag(d1[mask]) - Utils.matrix2mag(interp))

        return np.linalg.norm((a - a.max()) - (b - b.max()))
        return Utils.RMSe((a - a.max()) - (b - b.max()))
        return Utils.RMSe(Utils.matrix2mag(d1[mask]) - Utils.matrix2mag(interp))

    res = minimize(func_ta, [0, 1], (ts1 - ts1[0], ts2 - ts2[0], traj1, traj2),
                   method='Nelder-Mead')  # good!!! # [3347.734]

    # # # # ts1 - ts1[0] = (ts2 - ts2[0]) * scale + bias
    # # # ts1[i] - ts1[0] = ts2[i]*scale - ts2[0]*scale + bias
    # # ts1[i] = ts2[i]*scale - ts2[0]*scale + bias + ts1[0]
    # ts1[i] = ts2[i]*scale + bias + ts1[0] - ts2[0]*scale

    bias, scale = res.x
    return bias + ts1[0] - ts2[0] * scale, scale


def temporal_diff3(ts1, traj1, ts2, traj2):
    def func_ta(x, t1, t2, d1, d2):
        bias, scale = x
        slerp = Slerp(t2, R.from_matrix(d2[:, :3, :3]))
        new_ts = np.asarray(t1) + bias
        mask = (new_ts > t2[0]) & (new_ts < t2[-1])
        interp = slerp(new_ts[mask]).as_matrix()

        return np.linalg.norm(Utils.matrix2mag(d1[mask]) * scale - Utils.matrix2mag(interp))

    res = minimize(func_ta, [0, 1], (ts1 - ts1[0], ts2 - ts2[0], traj1, traj2),
                   method='Nelder-Mead')  # good!!! # [3347.734]

    # # ts1[i] - ts1[0] + res.x = ts2[i] - t2[0]
    # ts1[i] = ts2[i] + (ts1[0] - ts2[0] - res.x)

    print(res.x)
    bias, scale = res.x
    return ts1[0] - ts2[0] - bias  # time to add to t2 clock


def temporal_diff4(ts1, traj1, ts2, traj2):
    # fix imu drift and scale
    def func_ta1(x, t1, t2, d1, d2):
        time_bias, mag_scale, drift_bias, drift_scale = x

        new_t2 = t2 + time_bias
        mask = (t1 > new_t2[0]) & (t1 < new_t2[-1])
        interp = interpolate.interp1d(new_t2, d2)(t1[mask])
        new_d1 = d1 * mag_scale + (t1 - t1[0]) * drift_scale + drift_bias

        return np.linalg.norm(new_d1[mask] - interp)
        return Utils.RMSe(new_d1[mask] - interp)

    # time bias and scale
    def func_ta2(x, t1, t2, d1, d2):
        time_bias, time_scale = x

        new_t2 = t2 * time_scale + time_bias
        mask = (t1 > new_t2[0]) & (t1 < new_t2[-1])
        interp = interpolate.interp1d(new_t2, d2)(t1[mask])

        return np.linalg.norm(d1[mask] - interp)
        return Utils.RMSe(d1[mask] - interp)

    d1_ = Utils.matrix2mag(traj1)
    d2_ = scipy.signal.medfilt(Utils.matrix2mag(traj2), 9)
    # res = minimize(func_ta1, [0, 1, 0, 1], (ts1-ts1[0], ts2-ts2[0], d1_, d2_), method='Nelder-Mead')    # good!!! # [3347.734]
    # # res = minimize(func_ta1, [0, 1, 0, 1], (ts1-ts1[0] + 500, ts2-ts2[0], d1_, d2_), method='Nelder-Mead')    # good!!! # [3347.734]
    # # # res = minimize(func_ta1, [0, 1, 0, 1], (ts1[:15*30]-ts1[0] + 1000, ts2-ts2[0], d1_[:15*30], d2_), method='Nelder-Mead')    # good!!! # [3347.734]
    res = minimize(func_ta1, [0, 1, 0, 1], (1000 + ts1 - ts1[0], ts2 - ts2[0], d1_, d2_),
                   method='Nelder-Mead')  # good!!! # [3347.734]

    t1, t2 = ts1 - ts1[0], ts2 - ts2[0]
    t1, t2 = ts1 - ts1[0] + 1000, ts2 - ts2[0]
    time_bias, mag_scale, drift_bias, drift_scale = res.x
    new_t2 = t2 + time_bias
    new_d1 = d1_ * mag_scale + (ts1 - ts1[0]) * drift_scale + drift_bias

    if False:
        plt.figure()
        plt.scatter(t1, d1_, s=2, label='imu')
        plt.scatter(t1, new_d1, s=2, label='imu fix')
        plt.scatter(new_t2, d2_, s=2, label='opti')
        plt.legend()

    s = 10

    f1 = s * 1
    v1_ = (d1_[f1:] - d1_[:-f1]) / (ts1[f1:] - ts1[:-1 * f1])

    f2 = s * 8
    v2_ = (d2_[f2:] - d2_[:-f2]) / (ts2[f2:] - ts2[:-f2])

    v_thresh = 0.014 * 0.001
    start1 = np.argmax(v1_ > v_thresh) + f1
    start2 = np.argmax(v2_ > v_thresh) + f2

    sec = 15
    end1 = min(len(ts1) - 1, start1 + sec * 30)
    end2 = min(len(ts2) - 1, start2 + sec * 240)

    # res = minimize(func_ta2, [time_bias, 1], (ts1[start1:end1]-ts1[0], ts2[start2:end2]-ts2[0], new_d1[start1:end1], d2_[start2:end2]), method='Nelder-Mead')
    res = minimize(func_ta2, [time_bias, 1], (ts1[start1:end1] - ts1[0], ts2[start2:end2] - ts2[0], new_d1, d2_),
                   method='Nelder-Mead')

    time_bias, time_scale = res.x

    new_t2 = t2 * time_scale + time_bias

    # # ts1[i] - ts1[0] = (t2[i] * time_scale + time_bias) - ts2[0]
    # ts1[i] = t2[i] * time_scale + time_bias - ts2[0] + ts1[0]

    # bias + ts1[0] - ts2[0] * scale
    return time_bias - ts2[0] * time_scale + ts1[0], time_scale  # time to add to t2 clock


# def temporal_diff4(ts1, traj1, ts2, traj2):
#
#     def func_ta(x, t1, t2, d1, d2):
#
#         time_bias, time_scale, mag_scale, drift_bias, drift_scale = x
#
#         # new_t2 = t2 * time_scale + time_bias
#         new_t2 = t2 + time_bias
#         mask = (t1 > new_t2[0]) & (t1 < new_t2[-1])
#         interp = interpolate.interp1d(new_t2, d2)(t1[mask])
#         new_d1 = d1 * mag_scale + (t1 - t1[0]) * drift_scale + drift_bias
#
#         return np.linalg.norm(new_d1[mask] - interp)
#         return Utils.RMSe(new_d1[mask] - interp)
#
#
#     d1_ = Utils.matrix2mag(traj1)
#     d2_ = scipy.signal.medfilt(Utils.matrix2mag(traj2), 9)
#
#
#     s = 10
#
#     f1 = s*1
#     v1_ = (d1_[f1:]-d1_[:-f1]) / (ts1[f1:]-ts1[:-1*f1])
#     a1_ = (v1_[f1:]-v1_[:-f1]) / (ts1[2*f1:]-ts1[:-2*f1])
#     ad1_ = (a1_[f1:]-a1_[:-f1]) / (ts1[3*f1:]-ts1[:-3*f1])
#
#     f2 = s*8
#     v2_ = (d2_[f2:]-d2_[:-f2]) / (ts2[f2:]-ts2[:-f2])
#     a2_ = (v2_[f2:]-v2_[:-f2]) / (ts2[2*f2:]-ts2[:-(2*f2)])
#     ad2_ = (a2_[f2:]-a2_[:-f2]) / (ts2[3*f2:]-ts2[:-(3*f2)])
#
#     v_thresh = 0.014 * 0.001
#     start1 = np.argmax(v1_ > v_thresh) + f1
#     start2 = np.argmax(v2_ > v_thresh) + f2
#
#     sec = 2
#     end1 = min(len(ts1)-1, start1 + sec*30)
#     end2 = min(len(ts2)-1, start2 + sec*240)
#
#
#
#     res = minimize(func_ta, [0, 1, 1, 0, 1], (ts1-ts1[0], ts2-ts2[0], d1_, d2_), method='Nelder-Mead')    # good!!! # [3347.734]
#     res = minimize(func_ta, [0, 1, 1, 0, 1], (ts1[start1:end1]-ts1[0], ts2[start2:end2]-ts2[0], d1_[start1:end1], d2_[start2:end2]), method='Nelder-Mead')
#
#     # # ts1[i] - ts1[0] + res.x = ts2[i] - t2[0]
#     # ts1[i] = ts2[i] + (ts1[0] - ts2[0] - res.x)
#
#
#
#     print(res.x, func_ta(res.x, ts1-ts1[0], ts2-ts2[0], d1_, d2_))
#
#     time_bias, time_scale, mag_scale, drift_bias, drift_scale = res.x
#     t1 = ts1-ts1[0]
#     t2 = ts2-ts2[0]
#
#     d1 = d1_
#     d2 = d2_
#
#     new_t2 = t2 * time_scale + time_bias
#     new_t2 = t2 + time_bias
#     mask = (t1 > new_t2[0]) & (t1 < new_t2[-1])
#     interp = interpolate.interp1d(new_t2, d2)(t1[mask])
#     new_d1 = d1 * mag_scale + (ts1 - ts1[0]) * drift_scale + drift_bias
#
#     return res.x  # time to add to t2 clock


# def temporal_sync(ts1, rot1, ts2, rot2):
#     #
#     return 0.   # time diff


post_process_S, post_process_M = 300, 600
p1_sec, p2_sec, p3_sec = 3.5, 7.5, 5


def find_static_phase(t, d, interval_sec=0.5, seq_min_sec_len=0.5, d_ker=3):

    sec2msec = 1000

    t_with_delta_for_v = t + interval_sec * sec2msec
    mask_v = t_with_delta_for_v < t.max()
    t_with_delta_for_v = t_with_delta_for_v[mask_v]
    interp_d_for_v = interpolate.interp1d(t, d)(t_with_delta_for_v)
    v = (interp_d_for_v - d[mask_v]) / interval_sec

    t_with_delta_for_a = t_with_delta_for_v + interval_sec * 1000
    mask_a = t_with_delta_for_a < t_with_delta_for_v.max()
    t_with_delta_for_a = t_with_delta_for_a[mask_a]
    interp_v_for_a = interpolate.interp1d(t_with_delta_for_v, v)(t_with_delta_for_a)
    a = (interp_v_for_a - v[mask_a]) / interval_sec


    a *= 1/interval_sec     # mm per sec^2
    a_t_diff = t_with_delta_for_a[1:]-t_with_delta_for_a[:-1]
    # med filter 1/d_ker sec, making sure is odd
    ker_size = int(sec2msec / a_t_diff.mean() // d_ker)
    a_filter = scipy.signal.medfilt(a, ker_size - (1 - ker_size % 2))
    a_thresh = 0.01
    stat_mask_a_based = np.abs(a_filter) < a_thresh
    ta_stat_delta = t_with_delta_for_a[stat_mask_a_based]
    ta_map_stat = np.cumsum(1 - (ta_stat_delta[1:] - ta_stat_delta[:-1] < (seq_min_sec_len * sec2msec)))
    if False:
        a_stat_delta = a[stat_mask_a_based]
        plt.figure()
        plt.scatter(t_with_delta_for_a, a, s=2, label='a')
        plt.scatter(ta_stat_delta[1:], a_stat_delta[1:], c=ta_map_stat, s=5, label='a_stat', cmap='Dark2', alpha=0.8)
        plt.legend()
        plt.show()

    val, idx, cts = np.unique(ta_map_stat, return_index=True, return_counts=True)
    # [ta_stat_delta[1:][i] for i, c in zip(idx, cts)]
    # [ta_stat_delta[1:][i+c-1] for i, c in zip(idx, cts)]
    # [ta_stat_delta[1:][i+c-1]-ta_stat_delta[1:][i] for i, c in zip(idx, cts)]

    start = ta_stat_delta[1:][idx]
    end = ta_stat_delta[1:][idx + cts - 1]
    m = (end - start) > (2.5 * sec2msec)
    # m = ta_stat_delta[idx + cts] - ta_stat_delta[idx] > 2.5 * sec2msec
    # m = t_with_delta_for_a[idx + cts] - t_with_delta_for_a[idx] > 2.5 * sec2msec
    # m = t_with_delta_for_a[idx + cts] - t_with_delta_for_a[idx] > 1 * sec2msec

    ret = [(s, e) for s, e in zip(start[m], end[m])]

    if True:
        plt.figure()
        plt.scatter(t, d, s=2, label='d')
        for i, (s, e) in enumerate(ret):
            plt.scatter(t[(t > s) & (t < e)], d[(t > s) & (t < e)], s=2 , label=f'stat {i}')
        plt.legend()
        plt.show()

    return ret


def sync_time_phase1(ts1, data1d1, ts2, data1d2, stat_ts):
    # fix imu drift and scale
    def imu_drift_loss(x, t1, t2, d1, d2):
        time_bias, mag_scale, drift_bias, drift_scale = x

        new_t2 = t2 + time_bias
        mask = (t1 > new_t2[0]) & (t1 < new_t2[-1])
        interp = interpolate.interp1d(new_t2, d2)(t1[mask])
        new_d1 = d1 * mag_scale + (t1 - t1[0]) * drift_scale + drift_bias

        return np.linalg.norm(new_d1[mask] - interp)
        return Utils.RMSe(new_d1[mask] - interp)

    t1_, t2_ = ts1, ts2
    d1_, d2_ = data1d1, data1d2

    t1_mask = (t1_ > stat_ts[0][0]) & (t1_ < stat_ts[1][1])
    t2_mask = (t2_ > stat_ts[0][0]) & (t2_ < stat_ts[1][1])

    res = minimize(imu_drift_loss, [0, 1, 0, 1],
                   (t1_[t1_mask], t2_[t2_mask], d1_[t1_mask], d2_[t2_mask]),
                   method='Nelder-Mead')

    time_bias, mag_scale, drift_bias, drift_scale = res.x
    new_t2 = t2_ + time_bias
    new_d1 = d1_ * mag_scale + (ts1 - ts1[0]) * drift_scale + drift_bias

    if True:
        plt.figure()
        plt.scatter(t1_, d1_, s=2, label='imu')
        plt.scatter(t1_, new_d1, s=2, label='imu fix')
        plt.scatter(new_t2, d2_, s=2, label='opti')
        plt.legend()

        # plt.figure()
        # plt.scatter(t1_ + ts1[0] - init_diff, d1_, s=2, label='imu')
        # plt.scatter(t1_ + ts1[0] - init_diff, new_d1, s=2, label='imu fix')
        # plt.scatter(new_t2 + ts1[0] - init_diff, d2_, s=2, label='opti')
        # plt.legend()
        # plt.show()

    print('time sync phase 1 res = ', res.x, ', error = ', imu_drift_loss(res.x, t1_, t2_, d1_, d2_))
    return t1_, new_d1, new_t2, d2_
    return t1_ + ts1[0] - init_diff, new_d1, new_t2 + ts1[0] - init_diff, d2_


def sync_time_phase2(ts1, data1d1, ts2, data1d2, stat_ts, interval_sec=10, velocity_mm_per_ms=0.014, seq_sec=15):
    # todo: interval_sec is not the right name probably

    # time bias and scale
    def time_scale_and_bias_loss(x, t1, t2, d1, d2):

        time_bias, time_scale = x

        # new_t2 = t2 * time_scale + time_bias
        new_t2 = t2[0] + (t2 - t2[0]) * time_scale + time_bias
        #### new_t2 = t2[0] - t2[0] * time_scale + t2 * time_scale + time_bias
        mask = (t1 > new_t2[0]) & (t1 < new_t2[-1])
        interp = interpolate.interp1d(new_t2, d2)(t1[mask])

        return np.linalg.norm(d1[mask] - interp)
        return Utils.RMSe(d1[mask] - interp)

    # time bias
    def time_bias_loss(x, t1, t2, d1, d2):

        time_bias, = x

        # new_t2 = t2 * time_scale + time_bias
        new_t2 = t2[0] + (t2 - t2[0]) * 1 + time_bias
        #### new_t2 = t2[0] - t2[0] * time_scale + t2 * time_scale + time_bias
        mask = (t1 > new_t2[0]) & (t1 < new_t2[-1])
        interp = interpolate.interp1d(new_t2, d2)(t1[mask])

        return np.abs(d1[mask] - interp).mean()
        return Utils.RMSe(d1[mask] - interp)
        return np.linalg.norm(d1[mask] - interp)

    d1_, d2_ = data1d1, data1d2
    t1_, t2_ = ts1, ts2


    t1_mask = (t1_ > stat_ts[0][1]) & (t1_ < stat_ts[1][0])
    t2_mask = (t2_ > stat_ts[0][1]) & (t2_ < stat_ts[1][0])


    # fps1, fps2 = 30, 240
    # # todo: use ts vec to find pairs (or add nans when missing data instead of frame drop
    # f1 = interval_sec * fps1 // fps1
    # v1_ = (d1_[f1:] - d1_[:-f1]) / (t1_[f1:] - t1_[:-1 * f1])
    #
    # f2 = interval_sec * fps2 // fps1
    # v2_ = (d2_[f2:] - d2_[:-f2]) / (t2_[f2:] - t2_[:-f2])
    #
    # # todo: use ts vec to find pairs (or add nans when missing data instead of frame drop
    # v_thresh = velocity_mm_per_ms * 0.001
    # start1 = np.argmax(v1_ > v_thresh) + f1
    # start2 = np.argmax(v2_ > v_thresh) + f2
    #
    # # end1 = min(len(ts1) - 1, start1 + seq_sec * 30)
    # end1 = min(len(ts1) - 1, start1 + seq_sec * 30, post_process_S + post_process_M)
    # end2 = min(len(ts2) - 1, start2 + seq_sec * 240)
    #
    # # # option a: check diff and remove non fit dist
    # # # option b: search the nearest time in vector (maybe)
    # # ts_diff1 = t1_[f1:] - t1_[:-1*f1]
    # # mask1 = ts_diff1 < (fps1 * interval_sec)
    # # mask1 = ts_diff1 < 350

    if False:
        res = minimize(time_scale_and_bias_loss, [0, 1],
                       (t1_[start1:end1], t2_[start2:end2], d1_[start1:end1], d2_[start2:end2]), method='Nelder-Mead')
        time_bias, time_scale = res.x
        func = time_scale_and_bias_loss
    elif True:
        # # res = minimize(time_bias_loss, [0], (t1_[start1:end1], t2_[start2:end2], d1_[start1:end1], d2_[start2:end2]),
        # #                method='Nelder-Mead')
        # res = minimize(time_bias_loss, [0], (t1_[t1_mask], t2_[t2_mask], d1_[t1_mask], d2_[t2_mask]),
        #                method='Nelder-Mead')
        # res = minimize(time_bias_loss, [0], (t1_[t1_mask], t2_[t2_mask], d1_[t1_mask], d2_[t2_mask]),
        #                method='L-BFGS-B')
        # res = minimize(time_bias_loss, [0], (t1_[t1_mask], t2_[t2_mask], d1_[t1_mask], d2_[t2_mask]),
        #                method='Nelder-Mead', bounds=[(-60, 100)])
        res = minimize(time_bias_loss, [0], (t1_[t1_mask], t2_[t2_mask], d1_[t1_mask], d2_[t2_mask]),
                        method='Nelder-Mead')
        time_bias, = res.x
        time_scale = 1
        func = time_bias_loss

        print('time sync phase 2 res = ', res.x, ', all error = ', func(res.x, t1_, t2_, d1_, d2_))
    else:
        min_r = 99999
        min_i = None
        for i in range(-100, 100, 1):
            r = time_bias_loss([i], t1_[t1_mask], t2_[t2_mask], d1_[t1_mask], d2_[t2_mask])
            if r < min_r:
                min_r = r
                min_i = i

        time_bias = min_i
        time_scale = 1
        func = time_bias_loss

        print('time sync phase 2 res = ', min_i, ', all error = ', func([min_i], t1_, t2_, d1_, d2_))

    new_t2 = t2_[0] + (t2_ - t2_[0]) * time_scale + time_bias

    if True:
        plt.figure()
        plt.scatter(t1_, d1_, s=2, label='imu fix')
        plt.scatter(new_t2, d2_, s=2, label='opti')
        plt.legend()
        plt.show()

    # print('time sync phase 2 res = ', res.x, ', fit error = ',
    #       func(res.x, t1_[start1:end1], t2_[start2:end2], d1_[start1:end1], d2_[start2:end2]))
    # print('time sync phase 2 res = ', res.x, ', all error = ', func(res.x, t1_, t2_, d1_, d2_))
    return t1_, d1_, new_t2, d2_


def time_sync(ts1, traj1, ts2, traj2, init_diff=1000, interval_sec=10, velocity_mm_per_ms=0.014, seq_sec=15):
    # todo: replace the missing ts with nans in opti data
    t1, d1, t2, d2 = ts1, Utils.matrix2mag(traj1), ts2, Utils.matrix2mag(traj2)

    offset = init_diff - ts1[0]
    t1_, t2_ = ts1 + offset, ts2 - ts2[0]
    d1_, d2_ = d1, scipy.signal.medfilt(d2, 9)

    d2_stat = find_static_phase(t2_, d2_)
    if len(d2_stat) < 2:
        print('could not found static phase > 2.5 sec')
        return None, None

    t1__, d1__, t2__, d2__ = sync_time_phase1(t1_, d1_, t2_, d2_, d2_stat)
    t1___, d1___, t2___, d2___ = sync_time_phase2(t1__ - offset, d1__, t2__ - offset, d2__,
                                                  d2_stat - offset, interval_sec, velocity_mm_per_ms, seq_sec)

    return t2___, d2_stat - offset


def debug_data(data):
    plt.plot(data.ts[1:] - data.ts[:-1], marker='o')
    plt.show()


def post_process(capture_folder, ghc_path, doEval=False, evalDataPath=None):
    print('start post process on', capture_folder)

    data = DataReader(capture_folder)

    debug_data(data)

    imu_rot_rad_euler, imu_time = calc_imu_rotation(data.accel_ts, data.accel, data.gyro_ts, data.gyro)
    imu_idx = 'ag'  # 'a'   # 'g'   #
    imu_idx = 'g'  # 'ag'  # 'a'   #
    imu = Trajectory(imu_time[imu_idx], Utils.euler2matrix(imu_rot_rad_euler[imu_idx], degrees=False))

    opti_data = OptiReader(osp.join(capture_folder, 'opti_pose_list.csv'))
    opti = Trajectory(opti_data.ts.copy(), opti_data.pose.copy())

    new_opti_ts, stat = time_sync(imu.timestamp, imu.rel(), opti.timestamp, opti.rel())
    if new_opti_ts is None:
        return

    ghc = Utils.load_pkl(ghc_path)
    gt_est = Trajectory(new_opti_ts, opti_data.pose.copy()).project(ghc).interpolate(data.ts)
    offset = 0
    offset = 1000 / 30



    data_ts_mask = data.ts > (stat[1][1] - 1000)
    opti_orig_mask = (new_opti_ts > stat[1][0] + 200) & (new_opti_ts < stat[1][1] - 200)
    opti_orig_mat = Utils.average_transformation(opti_data.pose[opti_orig_mask])
    gt_est = Trajectory(new_opti_ts + offset, opti_data.pose.copy()).project(ghc, orig=opti_orig_mat).interpolate(data.ts[data_ts_mask])
    # gt_est = Trajectory(new_opti_ts + offset, opti_data.pose.copy()).project(ghc).interpolate(data.ts)

    if doEval:
        print('start eval using', evalDataPath)
        im_pose = ImageChessboardPose()
        im_pose.init_from_folder(evalDataPath)

        color_pose = [im_pose.get_pose(c.copy())[0] for c in data.color]
        color_pose = np.asarray([Utils.inv(p) if p is not None else np.eye(4) * np.nan for p in color_pose])

        cpt_orig_mask = (data.ts > stat[1][0] + 200) & (data.ts < stat[1][1] - 200)
        cpt_orig_mat = Utils.average_transformation(color_pose[cpt_orig_mask])
        cpt = Trajectory(data.ts[data_ts_mask], color_pose[data_ts_mask])
        cpt_rel = cpt.rel(orig=cpt_orig_mat)
        cpt_gt_diff = Utils.inv(cpt_rel) @ gt_est.pose

        print('RMSE error (rot)', Utils.RMSe(Utils.matrix2mag(cpt_gt_diff)) * 180 / np.pi)
        print('RMSE error (trans)', Utils.RMSe(Utils.trans_dist(cpt_gt_diff)))

        if True:
            plt.figure()
            # plt.scatter(imu.timestamp + offset, Utils.matrix2mag(imu.rel()), s=2, label='imu')
            plt.scatter(cpt.timestamp, Utils.matrix2mag(cpt_rel), s=2, label='rot color pose')
            plt.scatter(gt_est.timestamp, Utils.matrix2mag(gt_est.pose), s=2, label='rot est')
            plt.legend()

            plt.figure()
            plt.scatter(cpt.timestamp, Utils.trans_dist(cpt_rel), s=2, label='trans color pose')
            plt.scatter(gt_est.timestamp, Utils.trans_dist(gt_est.pose), s=2, label='trans est')
            plt.legend()

            plt.show()

        if False:
            for i in range(0, 100, 10):
                ggt_est = Trajectory(new_opti_ts + (1000 / 30)+i, opti_data.pose.copy()).project(ghc).interpolate(data.ts)
                print(i, 'RMSE error (rot)',
                      Utils.RMSe(Utils.matrix2mag(Utils.inv(cpt.rel()) @ ggt_est.pose)) * 180 / np.pi)
                print(i, 'RMSE error (trans)', Utils.RMSe(Utils.trans_dist(Utils.inv(cpt.rel()) @ ggt_est.pose)))

                plt.figure()
                plt.scatter(cpt.timestamp, Utils.matrix2mag(cpt.rel()), s=2, label='rot color pose')
                plt.scatter(gt_est.timestamp, Utils.matrix2mag(ggt_est.rel()), s=2, label=f'rot est {i}')
                plt.legend()
                plt.show()


        if False:
            plt.figure()
            plt.scatter(cpt.timestamp, cpt.rel()[:, 0, 3], s=2, label='tx color pose')
            plt.scatter(gt_est.timestamp, gt_est.rel()[:, 0, 3], s=2, label='tx est')
            plt.scatter(cpt.timestamp, cpt.rel()[:, 1, 3], s=2, label='ty color pose')
            plt.scatter(gt_est.timestamp, gt_est.rel()[:, 1, 3], s=2, label='ty est')
            plt.scatter(cpt.timestamp, cpt.rel()[:, 2, 3], s=2, label='tz color pose')
            plt.scatter(gt_est.timestamp, gt_est.rel()[:, 2, 3], s=2, label='tz est')
            plt.legend()

        cv2.destroyAllWindows()

    # todo: drop gt_est to


def post_process1(capture_folder, ghc_path, doEval=False, evalDataPath=None):
    data = DataReader(capture_folder)
    imu_rot_rad_euler, imu_time = calc_imu_rotation(data.accel_ts, data.accel, data.gyro_ts, data.gyro)
    imu_idx = 'g'  # 'ag'  # 'a'   #
    imu_idx = 'ag'  # 'a'   # 'g'   #
    imu = Trajectory(imu_time[imu_idx], Utils.euler2matrix(imu_rot_rad_euler[imu_idx], degrees=False))

    opti_data = OptiReader(osp.join(capture_folder, 'opti_pose_list.csv'))
    opti = Trajectory(opti_data.ts.copy(), opti_data.pose.copy())

    opti2 = Trajectory(opti_data.ts.copy(), opti_data.pose.copy())
    opti3 = Trajectory(opti_data.ts.copy(), opti_data.pose.copy())
    opti4 = Trajectory(opti_data.ts.copy(), opti_data.pose.copy())

    time_diff = temporal_diff(imu.timestamp, imu.rel(), opti.timestamp, opti.rel())
    print('ta diff', time_diff)
    time_diff2 = temporal_diff2(imu.timestamp, imu.rel(), opti.timestamp, opti.rel())
    print('ta diff 2', time_diff2)
    time_diff3 = temporal_diff3(imu.timestamp, imu.rel(), opti.timestamp, opti.rel())
    print('ta diff 3', time_diff3)
    time_diff4 = temporal_diff4(imu.timestamp, imu.rel(), opti.timestamp, opti.rel())
    print('ta diff 4', time_diff4)
    # todo A: RMSe
    # todo B: learn scale
    ghc = Utils.load_pkl(ghc_path)
    opti.timestamp += time_diff
    gt_estimation = opti.project(ghc).interpolate(data.ts)
    # todo: est = chg @ opti.rel(orig=avg of X first static poses) @ ghc # todo next: avg between X static poses
    opti2.timestamp = opti2.timestamp * time_diff2[1] + time_diff2[0]
    gt_estimation2 = opti2.project(ghc).interpolate(data.ts)

    opti3.timestamp += time_diff3
    gt_estimation3 = opti3.project(ghc).interpolate(data.ts)

    opti4.timestamp = opti4.timestamp * time_diff4[1] + time_diff4[0]
    gt_estimation4 = opti4.project(ghc).interpolate(data.ts)

    if doEval:
        im_pose = ImageChessboardPose()
        im_pose.init_from_folder(evalDataPath)

        color_pose = [im_pose.get_pose(c.copy())[0] for c in data.color]
        color_pose = np.asarray([Utils.inv(p) if p is not None else np.eye(4) * np.nan for p in color_pose])

        cpt_inv = Trajectory(data.ts, Utils.inv(color_pose))
        cpt = Trajectory(data.ts, color_pose)
        nandist = Utils.trans_dist(Utils.inv(cpt.rel()[1:]) @ gt_estimation.pose)
        nandist = Utils.trans_dist(Utils.inv(cpt.rel()[4:]) @ gt_estimation.pose)
        dist = nandist[~np.isnan(nandist)]

        rmse = ((dist ** 2).sum() / np.count_nonzero(~np.isnan(dist))) ** 1 / 2

        print('rmse error', rmse)

        # nandist2 = Utils.trans_dist(Utils.inv(cpt.rel()[1:]) @ gt_estimation2.pose)
        # dist2 = nandist2[~np.isnan(nandist2)]
        #
        # rmse2 = ((dist2 ** 2).sum() / np.count_nonzero(~np.isnan(dist2))) ** 1/2
        #
        # print('rmse error 2', rmse2)

        nandist3 = Utils.trans_dist(Utils.inv(cpt.rel()[1:]) @ gt_estimation3.pose)
        dist3 = nandist3[~np.isnan(nandist3)]

        rmse3 = ((dist3 ** 2).sum() / np.count_nonzero(~np.isnan(dist3))) ** 1 / 2

        print('rmse error 3', rmse3)

        nandist4 = Utils.trans_dist(Utils.inv(cpt.rel()[1:]) @ gt_estimation4.pose)
        dist4 = nandist4[~np.isnan(nandist4)]

        rmse4 = ((dist4 ** 2).sum() / np.count_nonzero(~np.isnan(dist4))) ** 1 / 2

        print('rmse error 4', rmse4)

        ggg = Trajectory(opti_data.ts * time_diff4[1] + time_diff4[0] + 100, opti_data.pose.copy()).project(
            ghc).interpolate(
            data.ts)

        ddd

        x = 11

        ...

    # dump gt data to pkl

    x = 11

    # read cam data
    #   calc imu rot (relative)
    # read opti data
    #   remove relative large error and spikes (cpp code available)
    # read ghc
    # calc est # verify the need
    # calc ta
    # project + interpolate
    # asset - optional:  calc cam pose and trans error
    # dump new pkl
    return False


if __name__ == '__main__':
    with np.printoptions(suppress=True, precision=3):
        # post_process(r'F:\AMR\record\20211114-151800',
        # post_process(r'F:\AMR\record\20211114-162227',
        post_process(r'F:\AMR\record\20211114-172436-p-good',
                     r'F:\AMR\calib-dc\calib-20211114-134246\gHc_nelder-mead-from-scratch.pkl',
                     True, r'D:\AMR\gtool\ClientSample\src\gt_raw')

        # # # # post_process(r'D:\mevolve\data\amr\record\time_calib\20211111-141653',
        # # # post_process(r'D:\mevolve\data\amr\record\time_calib\20211111-141343',
        # # post_process(r'D:\mevolve\data\amr\record\time_calib\20211111-140817',
        # # post_process(r'D:\mevolve\data\amr\record\time_calib\20211111-143359', # -31.849 diff
        # # post_process(r'D:\mevolve\data\amr\record\time_calib\20211111-144525',
        # # post_process(r'D:\mevolve\data\amr\record\time_calib\20211111-151303',
        # post_process(r'D:\mevolve\data\amr\record\time_calib\20211111-150012',
        #              r'D:\mevolve\data\amr\calib-dc\ghc_opt2021-11-03-03-10-42.pkl',
        #              True, r'D:\mevolve\data\amr\record\gt_raw')

        # post_process(r'F:\AMR\record\20211109-154840',
        #              r'D:\AMR\gtool\ClientSample\src\gt_raw\ghc_opt2021-11-03-03-10-42.pkl', True,
        #              r'D:\AMR\gtool\ClientSample\src\gt_raw')
        #
        # # post_process(r'C:\Users\ntuser\record\20211109-142305',
        # #              r'D:\AMR\gtool\ClientSample\src\gt_raw\ghc_opt2021-11-03-03-10-42.pkl', True,
        # #              r'D:\AMR\gtool\ClientSample\src\gt_raw')
        # #
        # # # post_process(r'C:\Users\ntuser\record\20211109-141224',
        # # #              r'D:\AMR\gtool\ClientSample\src\gt_raw\ghc_opt2021-11-03-03-10-42.pkl', True,
        # # #              r'D:\AMR\gtool\ClientSample\src\gt_raw')
        # # #
        # # # # post_process(
        # # # #     r'D:\mevolve\data\amr\record\omer_time_calib_type_1\20211109-111409',
        # # # #     r'D:\mevolve\data\amr\calib-dc\ghc_opt2021-11-03-03-10-42.pkl',
        # # # #     True, r'D:\mevolve\data\amr\record\gt_raw'
        # # # # )

    # post_process1(
    #     # # # # r'D:\mevolve\data\amr\record\20211108-130024',
    #     r'D:\mevolve\data\amr\record\20211108-131542', #'sin' like init phase
    #     # # # r'D:\mevolve\data\amr\record\20211108-142256',
    #     # r'D:\mevolve\data\amr\record\20211109-093652',
    #     # r'D:\mevolve\data\amr\record\omer_time_calib_type_1\20211109-111409',
    #     r'D:\mevolve\data\amr\calib-dc\ghc_opt2021-11-03-03-10-42.pkl',
    #     True, r'D:\mevolve\data\amr\record\gt_raw'
    # )
    # # post_process1(
    # #     r'D:\mevolve\data\amr\record\20211101-125437',
    # #     r'D:\mevolve\data\amr\calib-dc\ghc_opt2021-11-03-03-10-42.pkl',
    # #     True, r'D:\mevolve\data\amr\record\gt_raw'
    # # )
