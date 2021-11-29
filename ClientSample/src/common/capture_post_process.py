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
import tqdm

from multiprocessing import Pool
from collections import defaultdict, OrderedDict

from common.Utils import Utils, ImageChessboardPose


post_process_S, post_process_M = 300, 600
p1_sec, p2_sec, p3_sec = 3.5, 7.5, 5

debug_plot = True
sum_plot = True


class DataReader:
    def __init__(self, capture_folder, keys=['ts', 'color', 'accel', 'accel_ts', 'gyro', 'gyro_ts']):

        self.folder = capture_folder
        data, pkl_path = self.iterate_frames_(capture_folder, keys)
        for k, v in data.items():
            setattr(self, k, v)
        self.pkls = pkl_path

    @staticmethod
    def iterate_frames_(folder, keys):
        files = [x[:-4] for x in os.listdir(folder) if re.match('\d+\.pkl', x)]
        files.sort()
        pkls = [os.path.join(folder, f'{x}.pkl') for x in files]

        data = defaultdict(lambda: [])

        for pkl in pkls:
            for key, val in zip(*DataReader.read_frame_(pkl, keys)):
                data[key] += [val]

        return {k: np.asarray(v) for k, v in data.items()}, pkls
        # todo: convert to np?
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

        intr = 1000 / 30
        mask = [(self.ts > t - intr / 2) & (self.ts < t + intr / 2) for t in self.ts]
        self.med_pose = np.asarray([Utils.median_transformation(self.pose[m]) for m in mask])
        self.avg_pose = np.asarray([Utils.average_transformation(self.pose[m]) for m in mask])

        # p_rel = Utils.inv(self.pose[0]) @ self.pose
        # p_euler = Utils.matrix2euler(p_rel)
        # p_trans = p_rel[..., :3, 3]
        #
        # ap_rel = Utils.inv(self.avg_pose[0]) @ self.avg_pose
        # ap_euler = Utils.matrix2euler(ap_rel)
        # ap_trans = ap_rel[..., :3, 3]

        # self.avg_pose = self.pose

        if True:
            idx = np.where(self.ts[1:] != self.ts[:-1])[0]
            self.ts = self.ts[idx]
            self.pose = self.pose[idx]
            self.avg_pose = self.avg_pose[idx]
            self.med_pose = self.med_pose[idx]
            self.error = self.error[idx]

    def filter(self, df):
        t_error = 0.1
        t_error = 0.001
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
    if debug_plot:
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

    if debug_plot:
        plt.figure()
        plt.scatter(t, d, s=2, label='d')
        for i, (s, e) in enumerate(ret):
            plt.scatter(t[(t > s) & (t < e)], d[(t > s) & (t < e)], s=2 , label=f'stat {i}')
        plt.legend()
        plt.show()

    return ret


def sync_time_phase1(ts1, data1d1, ts2, data1d2, stat_ts):
    # fix imu drift and scale

    def imu_drift_bias_scale_loss(x, t1, t2, d1, d2):
        drift_bias, drift_scale = x

        mask = (t1 > t2[0]) & (t1 < t2[-1])
        interp = interpolate.interp1d(t2, d2)(t1[mask])
        new_d1 = d1 + (t1 - t1[0]) * drift_scale + drift_bias

        return Utils.RMSe(new_d1[mask] - interp)

    def imu_scale_loss(x, t1, t2, d1, d2):
        time_bias, mag_scale = x

        new_t2 = t2 + time_bias
        mask = (t1 > new_t2[0]) & (t1 < new_t2[-1])
        interp = interpolate.interp1d(new_t2, d2)(t1[mask])
        new_d1 = d1 * mag_scale

        return np.linalg.norm(new_d1[mask] - interp)
        return Utils.RMSe(new_d1[mask] - interp)

    def imu_scale_loss2(x, t1, t2, d1, d2):
        time_bias, mag_scale, y_bias = x

        new_t2 = t2 + time_bias
        mask = (t1 > new_t2[0]) & (t1 < new_t2[-1])
        interp = interpolate.interp1d(new_t2, d2)(t1[mask])
        new_d1 = d1 * mag_scale + y_bias

        return np.linalg.norm(new_d1[mask] - interp)
        return Utils.RMSe(new_d1[mask] - interp)


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

    if True:
        t1_mask = (t1_ > stat_ts[0][0]) & (t1_ < stat_ts[1][1])
        t2_mask = (t2_ > stat_ts[0][0]) & (t2_ < stat_ts[1][1])

        res = minimize(imu_drift_loss, [0, 1, 0, 1],
                       (t1_[t1_mask], t2_[t2_mask], d1_[t1_mask], d2_[t2_mask]),
                       method='Nelder-Mead')
        time_bias, mag_scale, drift_bias, drift_scale = res.x
        new_t2 = t2_ + time_bias
        new_d1 = d1_ * mag_scale + (ts1 - ts1[0]) * drift_scale + drift_bias


        res_out = [drift_bias, drift_scale, np.nan, np.nan, np.nan, time_bias, mag_scale]

        tmp = imu_drift_loss(res.x, t1_, t2_, d1_, d2_)
        print('imu_drift_loss res = ', res.x, ', error all = ', tmp)
        res_out += [tmp]

        tmp = imu_drift_loss(res.x, t1_[t1_mask], t2_[t2_mask], d1_[t1_mask], d2_[t2_mask])
        print('imu_drift_loss res = ', res.x, ', error fit = ', tmp)
        res_out += [tmp]

        t1_mask = (t1_ > stat_ts[1][1] + 1000)
        t2_mask = (t2_ > stat_ts[1][1] + 1000)
        tmp = imu_drift_loss(res.x, t1_[t1_mask], t2_[t2_mask], d1_[t1_mask], d2_[t2_mask])
        print('imu_drift_loss res = ', res.x, ', error test = ', tmp)
        res_out += [tmp]


    else:
        t1_mask = (t1_ > stat_ts[1][0]) & (t1_ < stat_ts[1][1])
        t2_mask = (t2_ > stat_ts[1][0]) & (t2_ < stat_ts[1][1])
        # t1_mask = (t1_ > stat_ts[0][0]) & (t1_ < stat_ts[0][1])
        # t2_mask = (t2_ > stat_ts[0][0]) & (t2_ < stat_ts[0][1])

        # res = minimize(imu_drift_bias_scale_loss, [0, 1],
        res = minimize(imu_drift_bias_scale_loss, [d2_[t2_mask][0] - d1_[t1_mask][0], 1],
                       (t1_[t1_mask], t2_[t2_mask], d1_[t1_mask], d2_[t2_mask]),
                       method='Nelder-Mead')

        drift_bias, drift_scale = res.x
        new_d1 = d1_ + (t1_ - t1_[t1_mask][0]) * drift_scale + drift_bias

        res_out = [*res.x]

        tmp = imu_drift_bias_scale_loss(res.x, t1_, t2_, d1_, d2_)
        print('imu_drift_bias_scale_loss res = ', res.x, ', error all = ', tmp)
        res_out += [tmp]

        tmp = imu_drift_bias_scale_loss(res.x, t1_[t1_mask], t2_[t2_mask], d1_[t1_mask], d2_[t2_mask])
        print('imu_drift_bias_scale_loss res = ', res.x, ', error fit = ', tmp)
        res_out += [tmp]

        t1_mask = (t1_ > stat_ts[1][1] + 1000)
        t2_mask = (t2_ > stat_ts[1][1] + 1000)
        tmp = imu_drift_bias_scale_loss(res.x, t1_[t1_mask], t2_[t2_mask], d1_[t1_mask], d2_[t2_mask])
        print('imu_drift_bias_scale_loss res = ', res.x, ', error test = ', tmp)
        res_out += [tmp]

        if True:
            t1_mask = (t1_ > stat_ts[0][0]) & (t1_ < stat_ts[1][1])
            t2_mask = (t2_ > stat_ts[0][0]) & (t2_ < stat_ts[1][1])

            res = minimize(imu_scale_loss, [0, 1],
                           (t1_[t1_mask], t2_[t2_mask], new_d1[t1_mask], d2_[t2_mask]),
                           method='Nelder-Mead')

            res_out += [*res.x]

            time_bias, mag_scale = res.x
            new_t2 = t2_ + time_bias
            new_d1 = new_d1 * mag_scale

            tmp = imu_scale_loss(res.x, t1_, t2_, d1_, d2_)
            print('imu_scale_loss res = ', res.x, ', error all = ', tmp)
            res_out += [tmp]

            tmp = imu_scale_loss(res.x, t1_[t1_mask], t2_[t2_mask], d1_[t1_mask], d2_[t2_mask])
            print('imu_scale_loss res = ', res.x, ', error fit = ', tmp)
            res_out += [tmp]

            t1_mask = (t1_ > stat_ts[1][1] + 1000)
            t2_mask = (t2_ > stat_ts[1][1] + 1000)
            tmp = imu_scale_loss(res.x, t1_[t1_mask], t2_[t2_mask], d1_[t1_mask], d2_[t2_mask])
            print('imu_scale_loss res = ', res.x, ', error test = ', tmp)
            res_out += [tmp]

        else:
            t1_mask = (t1_ > stat_ts[0][1] + 1000) & (t1_ < stat_ts[1][0])
            t2_mask = (t2_ > stat_ts[0][1] + 1000) & (t2_ < stat_ts[1][0])

            res = minimize(imu_scale_loss2, [0, 1, 0],
                           (t1_[t1_mask], t2_[t2_mask], new_d1[t1_mask], d2_[t2_mask]),
                           method='Nelder-Mead')
            print('imu_scale_loss2 res = ', res.x, ', error all = ', imu_scale_loss2(res.x, t1_, t2_, d1_, d2_))
            time_bias, mag_scale, y_bias = res.x
            new_t2 = t2_ + time_bias
            new_d1 = new_d1 * mag_scale + y_bias

    if debug_plot:
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

    # print('time sync phase 1 res = ', res.x, ', error all = ', imu_drift_loss(res.x, t1_, t2_, d1_, d2_))
    # print('time sync phase 1 res = ', res.x, ', error fit = ', imu_drift_loss(res.x, t1_[t1_mask], t2_[t2_mask], d1_[t1_mask], d2_[t2_mask]))
    # t1_mask = (t1_ > stat_ts[1][1])
    # t2_mask = (t2_ > stat_ts[1][1])
    # print('time sync phase 1 res = ', res.x, ', error test = ', imu_drift_loss(res.x, t1_[t1_mask], t2_[t2_mask], d1_[t1_mask], d2_[t2_mask]))
    return (t1_, new_d1, new_t2, d2_), (res_out, time_bias)
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

        res_out = [*res.x]

        tmp = time_bias_loss(res.x, t1_, t2_, d1_, d2_)
        print('time_bias_loss res = ', res.x, ', error all = ', tmp)
        res_out += [tmp]

        tmp = time_bias_loss(res.x, t1_[t1_mask], t2_[t2_mask], d1_[t1_mask], d2_[t2_mask])
        print('time_bias_loss res = ', res.x, ', error fit = ', tmp)
        res_out += [tmp]

        t1_mask = (t1_ > stat_ts[1][1] + 1000)
        t2_mask = (t2_ > stat_ts[1][1] + 1000)
        tmp = time_bias_loss(res.x, t1_[t1_mask], t2_[t2_mask], d1_[t1_mask], d2_[t2_mask])
        print('time_bias_loss res = ', res.x, ', error test = ', tmp)
        res_out += [tmp]

        # print('time sync phase 2 res = ', res.x, ', all error = ', func(res.x, t1_, t2_, d1_, d2_))
        # print('time sync phase 2 res = ', res.x, ', fit error = ', func(res.x, t1_[t1_mask], t2_[t2_mask], d1_[t1_mask], d2_[t2_mask]))
        # t1_mask = (t1_ > stat_ts[1][1])
        # t2_mask = (t2_ > stat_ts[1][1])
        # print('time sync phase 2 res = ', res.x, ', test error = ', func(res.x, t1_[t1_mask], t2_[t2_mask], d1_[t1_mask], d2_[t2_mask]))
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

    if debug_plot:
        plt.figure()
        plt.scatter(t1_, d1_, s=2, label='imu fix')
        plt.scatter(new_t2, d2_, s=2, label='opti')
        plt.legend()
        plt.show()

    # print('time sync phase 2 res = ', res.x, ', fit error = ',
    #       func(res.x, t1_[start1:end1], t2_[start2:end2], d1_[start1:end1], d2_[start2:end2]))
    # print('time sync phase 2 res = ', res.x, ', all error = ', func(res.x, t1_, t2_, d1_, d2_))
    return (t1_, d1_, new_t2, d2_), (res_out, time_bias)


def time_sync(ts1, traj1, ts2, traj2, init_diff=1000, interval_sec=10, velocity_mm_per_ms=0.014, seq_sec=15):
    # todo: replace the missing ts with nans in opti data
    t1, d1, t2, d2 = ts1, Utils.matrix2mag(traj1), ts2, Utils.matrix2mag(traj2)

    offset = init_diff - ts1[0]
    t1_, t2_ = ts1 + offset, ts2 - ts2[0]
    d1_, d2_ = d1, scipy.signal.medfilt(d2, 9)

    d2_stat = find_static_phase(t2_, d2_)
    if len(d2_stat) < 2:
        print('could not found static phase > 2.5 sec')
        return None, None, None


    (t1__, d1__, t2__, d2__), (res_out_1, time_bias_1) = sync_time_phase1(t1_, d1_, t2_, d2_, d2_stat)
    (t1___, d1___, t2___, d2___), (res_out_2, time_bias_2) = sync_time_phase2(t1__ - offset, d1__, t2__ - offset, d2__,
                                                  d2_stat - offset, interval_sec, velocity_mm_per_ms, seq_sec)

    time_bias = time_bias_2 + time_bias_1
    print('total time bias', time_bias)
    return t2___, d2_stat - offset, res_out_1 + res_out_2 + [time_bias]


def debug_data(data):
    plt.plot(data.ts[1:] - data.ts[:-1], marker='o')
    plt.show()


def post_process(capture_folder, ghc_path, doEval=False, evalDataPath=None, doDump=False, outfile=None):
    print('start post process on', capture_folder)

    data = DataReader(capture_folder)

    # debug_data(data)

    imu_rot_rad_euler, imu_time = calc_imu_rotation(data.accel_ts, data.accel, data.gyro_ts, data.gyro)
    imu_idx = 'ag'  # 'a'   # 'g'   #
    imu_idx = 'g'  # 'ag'  # 'a'   #
    imu = Trajectory(imu_time[imu_idx], Utils.euler2matrix(imu_rot_rad_euler[imu_idx], degrees=False))

    opti_data = OptiReader(osp.join(capture_folder, 'opti_pose_list.csv'))
    opti = Trajectory(opti_data.ts.copy(), opti_data.pose.copy())

    Utils.inv(opti.interpolate(opti.timestamp + 500, True).pose) @ opti.pose

    new_opti_ts, stat, res_out = time_sync(imu.timestamp, imu.rel(), opti.timestamp, opti.rel())
    if new_opti_ts is None:
        return

    ghc = Utils.load_pkl(ghc_path)
    offset = 0
    offset = 1000 / 30



    data_ts_mask = data.ts > (stat[1][1] - 1000)
    opti_orig_mask = (new_opti_ts > stat[1][0] + 200) & (new_opti_ts < stat[1][1] - 200)
    opti_orig_mat = Utils.average_transformation(opti_data.pose[opti_orig_mask])

    gt_est = Trajectory(new_opti_ts + offset, opti_data.pose.copy()).project(ghc, orig=opti_orig_mat).interpolate(data.ts[data_ts_mask])
    gt_avg_est = Trajectory(new_opti_ts + offset, opti_data.avg_pose.copy()).project(ghc, orig=opti_orig_mat).interpolate(data.ts[data_ts_mask])
    gt_med_est = Trajectory(new_opti_ts + offset, opti_data.med_pose.copy()).project(ghc, orig=opti_orig_mat).interpolate(data.ts[data_ts_mask])


    if outfile is not None:
        outfile.write(','.join([capture_folder] + [f'{s:.06}' for s in res_out]))

    if doDump is True:
        gt_out = 'gt'
        os.makedirs(os.path.join(data.folder, gt_out), exist_ok=True)
        #todo: avg, med...........
        for pkl_path, gt_ts, gt_pose in zip(np.asarray(data.pkls)[data_ts_mask], gt_est.timestamp, gt_est.pose):
            pkl = Utils.load_pkl(pkl_path)
            if pkl['ts'] == gt_ts: # if all goes good should be always true
                pkl['gt_pose'] = gt_pose.copy()
                pkl['gt_pose'][..., :3, 3] *= 0.001
                # print(gt_ts, pkl['gt_pose'][..., :3, 3])
                Utils.save_pkl((lambda x: os.path.join(x[0], gt_out, x[1]))(os.path.split(pkl_path)), pkl)
            else:
                print('****** error', pkl_path, pkl['ts'], gt_ts)

    if doEval:
        print('start eval using', evalDataPath)
        im_pose = ImageChessboardPose()
        im_pose.init_from_folder(evalDataPath)

        if False:
                color_pose = [im_pose.get_pose(c.copy(), show=True, draw_coord=True)[0] for c in data.color]
                color_pose = np.asarray([Utils.inv(p) if p is not None else np.eye(4) * np.nan for p in color_pose])
        else:
            if False:
                with Pool(10) as p:
                    color_pose = p.starmap(im_pose.get_pose, tqdm.tqdm([(c,) for c in data.color]))
            else:
                color_pose = []
                for c in tqdm.tqdm(data.color):
                    color_pose += [im_pose.get_pose(c)]
            color_pose = np.asarray([Utils.inv(p[0]) if p[0] is not None else np.eye(4) * np.nan for p in color_pose])

        cpt_orig_mask = (data.ts > stat[1][0] + 200) & (data.ts < stat[1][1] - 200)
        cpt_orig_mat = Utils.average_transformation(color_pose[cpt_orig_mask])
        cpt = Trajectory(data.ts[data_ts_mask], color_pose[data_ts_mask])
        cpt_rel = cpt.rel(orig=cpt_orig_mat)

        cpt_gt_diff = Utils.inv(cpt_rel) @ gt_est.pose
        rot_rmse = Utils.RMSe(Utils.matrix2mag(cpt_gt_diff)) * 180 / np.pi
        trans_rmse = Utils.RMSe(Utils.trans_dist(cpt_gt_diff))
        print('RMSE error (rot)', rot_rmse)
        print('RMSE error (trans)', trans_rmse)

        cpt_gt_avg_diff = Utils.inv(cpt_rel) @ gt_avg_est.pose
        rot_avg_rmse = Utils.RMSe(Utils.matrix2mag(cpt_gt_avg_diff)) * 180 / np.pi
        trans_avg_rmse = Utils.RMSe(Utils.trans_dist(cpt_gt_avg_diff))
        print('RMSE error (rot) (avg opti)', rot_avg_rmse)
        print('RMSE error (trans) (avg opti)', trans_avg_rmse)

        cpt_gt_med_diff = Utils.inv(cpt_rel) @ gt_med_est.pose
        rot_med_rmse = Utils.RMSe(Utils.matrix2mag(cpt_gt_med_diff)) * 180 / np.pi
        trans_med_rmse = Utils.RMSe(Utils.trans_dist(cpt_gt_med_diff))
        print('RMSE error (rot) (median opti)', rot_med_rmse)
        print('RMSE error (trans) (median opti)', trans_med_rmse)

        if outfile is not None:
            outfile.write(f',{trans_rmse:.06},{rot_rmse:.06},{trans_avg_rmse:.06},{rot_avg_rmse:.06},{trans_med_rmse:.06},{rot_med_rmse:.06}')

        if sum_plot:
            plt.figure()
            # plt.scatter(imu.timestamp + offset, Utils.matrix2mag(imu.rel()), s=2, label='imu')
            plt.scatter(cpt.timestamp, Utils.matrix2mag(cpt_rel), s=2, label='rot color pose')
            plt.scatter(gt_est.timestamp, Utils.matrix2mag(gt_est.pose), s=2, label='rot est')
            plt.scatter(gt_est.timestamp, Utils.matrix2mag(gt_avg_est.pose), s=2, label='rot (avg) est')
            plt.scatter(gt_est.timestamp, Utils.matrix2mag(gt_med_est.pose), s=2, label='rot (med) est')
            plt.legend()

            plt.figure()
            plt.scatter(cpt.timestamp, Utils.trans_dist(cpt_rel), s=2, label='trans color pose')
            plt.scatter(gt_est.timestamp, Utils.trans_dist(gt_est.pose), s=2, label='trans est')
            plt.scatter(gt_est.timestamp, Utils.trans_dist(gt_avg_est.pose), s=2, label='trans (avg) est')
            plt.scatter(gt_est.timestamp, Utils.trans_dist(gt_med_est.pose), s=2, label='trans (med) est')
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
        if outfile is not None:
            outfile.write('\n')
            outfile.flush()

    # todo: drop gt_est to


def write_log_title(outfile):
    outfile.write(','.join([
        'data_path',
        'imu_drift_bias[I]',
        'imu_drift_scale[I]',
        'error_imu_drift_all[I]',
        'error_imu_drift_fit[I]',
        'error_imu_drift_test[I]',
        'time_bias[I]',
        'imu_scale[I]',
        'error_time_bias_imu_scale_all[I]',
        'error_time_bias_imu_scale_fit[I]',
        'error_time_bias_imu_scale_test[I]',
        'time_bias[II]',
        'error_time_bias_all[II]',
        'error_time_bias_fit[II]',
        'error_time_bias_test[II]',
        'total_time_bias',
        'trans RMSE',
        'rot RMSE',
        'trans (avg) RMSE',
        'rot (avg) RMSE',
        'trans (median) RMSE',
        'rot (median) RMSE',
    ]) + '\n')


from glob import glob
if __name__ == '__main__':
    outfile = r'\\optitrack.ger.corp.intel.com\GTService\25.11.21\20211125__old_ta__ag.csv'
    input_list = glob(r'\\optitrack.ger.corp.intel.com\GTService\25.11.21\20211125-*')

    with open(outfile, 'w') as ofile:
        write_log_title(ofile)
        # for f in [r'D:\mevolve\data\amr\from_inbal_for_correl\20211118-151916']:
        for i, f in enumerate(input_list):
            print('---------->', i)
            try:
                post_process(f,
                             r'\\optitrack.ger.corp.intel.com\GTService\25.11.21\calib-20211125-101528\gHc_nelder-mead-from-guess.pkl',
                             doEval=True, doDump=False,
                             evalDataPath=r'\\optitrack.ger.corp.intel.com\GTService\25.11.21\calib-20211125-101528',
                             outfile=ofile)
            except:
                ofile.write('~~~~~~~~ error~~~~~~~~~~\n')
            print('=========>', i)


    exit(0)

    post_process(r'\\optitrack.ger.corp.intel.com\GTService\25.11.21\20211125-103334',
                 r'\\optitrack.ger.corp.intel.com\GTService\25.11.21\calib-20211125-101528\gHc_nelder-mead-from-guess.pkl',
                 doEval=True, doDump=False,
                 evalDataPath=r'\\optitrack.ger.corp.intel.com\GTService\25.11.21\calib-20211125-101528',
                 outfile=None)
    exit(0)


    input_list = [
        r'D:\mevolve\data\amr\from_inbal_for_correl\20211118-105350',
        r'D:\mevolve\data\amr\from_inbal_for_correl\20211118-105708',
        r'D:\mevolve\data\amr\from_inbal_for_correl\20211118-113728',
        r'D:\mevolve\data\amr\from_inbal_for_correl\20211118-114235',
        r'D:\mevolve\data\amr\from_inbal_for_correl\20211118-114622',
        r'D:\mevolve\data\amr\from_inbal_for_correl\20211118-115006',
        r'D:\mevolve\data\amr\from_inbal_for_correl\20211118-115242',
        r'D:\mevolve\data\amr\from_inbal_for_correl\20211118-115544',
        r'D:\mevolve\data\amr\from_inbal_for_correl\20211118-130026',
        # r'D:\mevolve\data\amr\from_inbal_for_correl\20211118-130505',
        r'D:\mevolve\data\amr\from_inbal_for_correl\20211118-130623',
        r'D:\mevolve\data\amr\from_inbal_for_correl\20211118-131002',
        r'D:\mevolve\data\amr\from_inbal_for_correl\20211118-131249',
        r'D:\mevolve\data\amr\from_inbal_for_correl\20211118-131553',
        r'D:\mevolve\data\amr\from_inbal_for_correl\20211118-132008',
        r'D:\mevolve\data\amr\from_inbal_for_correl\20211118-132425',
        r'D:\mevolve\data\amr\from_inbal_for_correl\20211118-132825',
        r'D:\mevolve\data\amr\from_inbal_for_correl\20211118-133313',
        r'D:\mevolve\data\amr\from_inbal_for_correl\20211118-133736',
        r'D:\mevolve\data\amr\from_inbal_for_correl\20211118-134025',
        r'D:\mevolve\data\amr\from_inbal_for_correl\20211118-134258',
        r'D:\mevolve\data\amr\from_inbal_for_correl\20211118-134514',
        r'D:\mevolve\data\amr\from_inbal_for_correl\20211118-134755',
        r'D:\mevolve\data\amr\from_inbal_for_correl\20211118-135013',
        r'D:\mevolve\data\amr\from_inbal_for_correl\20211118-135239',
        r'D:\mevolve\data\amr\from_inbal_for_correl\20211118-135510',
        r'D:\mevolve\data\amr\from_inbal_for_correl\20211118-135730',
        r'D:\mevolve\data\amr\from_inbal_for_correl\20211118-135943',
        r'D:\mevolve\data\amr\from_inbal_for_correl\20211118-145501',
        r'D:\mevolve\data\amr\from_inbal_for_correl\20211118-145748',
        r'D:\mevolve\data\amr\from_inbal_for_correl\20211118-150233',
        r'D:\mevolve\data\amr\from_inbal_for_correl\20211118-150504',
        r'D:\mevolve\data\amr\from_inbal_for_correl\20211118-150753',
        r'D:\mevolve\data\amr\from_inbal_for_correl\20211118-151400',
        r'D:\mevolve\data\amr\from_inbal_for_correl\20211118-151916',
        r'D:\mevolve\data\amr\from_inbal_for_correl\20211118-152159'
    ]

    I = 'old'
    II = 'old'
    filter = 'no'
    # with open(f'I-{I}_II-{II}_{filter}Filter.csv', 'a') as ofile:
    with open(f'I-{I}_II-{II}_{filter}Filter.csv', 'w') as ofile:
        write_log_title(ofile)
        # for f in [r'D:\mevolve\data\amr\from_inbal_for_correl\20211118-151916']:
        for i, f in enumerate(input_list):
            print('---------->', i)
            try:
                post_process(f, r'D:\mevolve\data\amr\from_inbal_for_correl\publish\gHc_nelder-mead-from-guess.pkl',
                         doEval=True, doDump=False,
                         evalDataPath=r'D:\mevolve\data\amr\from_inbal_for_correl\publish',
                         outfile=ofile)
            except:
                ofile.write('~~~~~~~~ error~~~~~~~~~~\n')
            print('=========>', i)



    # with np.printoptions(suppress=True, precision=3):
    #
    #     # # post_process(r'D:\mevolve\data\amr\from_inbal_for_correl\20211118-130623',
    #     # post_process(r'D:\mevolve\data\amr\from_inbal_for_correl\20211118-132825',
    #     post_process(r'D:\mevolve\data\amr\from_inbal_for_correl\20211118-151916',
    #                  r'D:\mevolve\data\amr\calib-dc\ghc_opt2021-11-03-03-10-42.pkl',
    #                  True, r'D:\mevolve\data\amr\record\gt_raw')
    #
    #
    #     # post_process(r'D:\mevolve\data\amr\record\time_calib\20211111-150012',
    #     #              r'D:\mevolve\data\amr\calib-dc\ghc_opt2021-11-03-03-10-42.pkl',
    #     #              True, r'D:\mevolve\data\amr\record\gt_raw')
    #
    #
    #     # post_process(r'C:\Users\rsheinin\record\20211116-142717',
    #     # # post_process(r'\\mevolve-win\wbm-trial-01-60deg-noirnoise\for_Riki\20211118-105708',
    #     #              r'..\capture_tool\gt_raw\gHc_nelder-mead-from-scratch.pkl',
    #     #              True, r'..\capture_tool\gt_raw')
    #
    #
    #     # # post_process(r'F:\AMR\record\20211114-151800',
    #     # # post_process(r'F:\AMR\record\20211114-162227',
    #     # post_process(r'F:\AMR\record\20211114-172436-p-good',
    #     #              r'F:\AMR\calib-dc\calib-20211114-134246\gHc_nelder-mead-from-scratch.pkl',
    #     #              True, r'D:\AMR\gtool\ClientSample\src\gt_raw')
    #
    #     # # # # post_process(r'D:\mevolve\data\amr\record\time_calib\20211111-141653',
    #     # # # post_process(r'D:\mevolve\data\amr\record\time_calib\20211111-141343',
    #     # # post_process(r'D:\mevolve\data\amr\record\time_calib\20211111-140817',
    #     # # post_process(r'D:\mevolve\data\amr\record\time_calib\20211111-143359', # -31.849 diff
    #     # # post_process(r'D:\mevolve\data\amr\record\time_calib\20211111-144525',
    #     # # post_process(r'D:\mevolve\data\amr\record\time_calib\20211111-151303',
    #     # post_process(r'D:\mevolve\data\amr\record\time_calib\20211111-150012',
    #     #              r'D:\mevolve\data\amr\calib-dc\ghc_opt2021-11-03-03-10-42.pkl',
    #     #              True, r'D:\mevolve\data\amr\record\gt_raw')
    #
    #     # post_process(r'F:\AMR\record\20211109-154840',
    #     #              r'D:\AMR\gtool\ClientSample\src\gt_raw\ghc_opt2021-11-03-03-10-42.pkl', True,
    #     #              r'D:\AMR\gtool\ClientSample\src\gt_raw')
    #     #
    #     # # post_process(r'C:\Users\ntuser\record\20211109-142305',
    #     # #              r'D:\AMR\gtool\ClientSample\src\gt_raw\ghc_opt2021-11-03-03-10-42.pkl', True,
    #     # #              r'D:\AMR\gtool\ClientSample\src\gt_raw')
    #     # #
    #     # # # post_process(r'C:\Users\ntuser\record\20211109-141224',
    #     # # #              r'D:\AMR\gtool\ClientSample\src\gt_raw\ghc_opt2021-11-03-03-10-42.pkl', True,
    #     # # #              r'D:\AMR\gtool\ClientSample\src\gt_raw')
    #     # # #
    #     # # # # post_process(
    #     # # # #     r'D:\mevolve\data\amr\record\omer_time_calib_type_1\20211109-111409',
    #     # # # #     r'D:\mevolve\data\amr\calib-dc\ghc_opt2021-11-03-03-10-42.pkl',
    #     # # # #     True, r'D:\mevolve\data\amr\record\gt_raw'
    #     # # # # )
