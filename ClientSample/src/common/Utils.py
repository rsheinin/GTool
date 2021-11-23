import cv2
import os, os.path as osp
import pickle
import shutil

import numpy as np
from scipy.spatial.transform import Rotation as R

from common.ServiceClient.OptiClient import OptiClient


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

    @staticmethod
    def median_transformation(pose_list):
        p_ = np.asarray(pose_list)
        rot = Utils.euler2matrix(np.median(Utils.matrix2euler(p_), axis=0))
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

    def init_from_data(self, board_h, board_w, board_mm_h, board_mm_w, color_cam_mat, color_cam_dist):
        self.board_h, self.board_w, self.board_mm_h, self.board_mm_w = \
            board_h, board_w, board_mm_h, board_mm_w

        self.board_points3d = self.make_points3d(self.board_h, self.board_w, self.board_mm_h, self.board_mm_w)

        self.color_cam_mat = color_cam_mat
        self.color_cam_dist = color_cam_dist

    def get_pose(self, image, show=False, draw_corners=False, draw_coord=False, coord_scale=50):
        pose = self.image2pose(image, self.color_cam_mat, self.color_cam_dist,
                               self.board_h, self.board_w, self.board_points3d,
                               draw_coord=draw_coord, draw_corners=draw_corners, coord_scale=coord_scale)
        if show:
            cv2.imshow('im pose', image)
            cv2.waitKey(1)
        return pose

    def draw_board(self, image, pose, color):
        return self.draw_points3d(image, pose, self.board_points3d, self.color_cam_mat, self.color_cam_dist, color)


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
    def draw_points3d(image, pose, points3d, mtx, dist, color):
        rvec = cv2.Rodrigues(pose[..., :3, :3])[0]
        tvec = pose[..., :3, 3]

        imgpts, jac = cv2.projectPoints(points3d, rvec, tvec, mtx, dist)

        for pt in imgpts:
            im = cv2.circle(image, pt[0].astype('int'), 3, color)

        return im

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


class OptitrackClient:
    def __init__(self,
                 server='optitrack.ger.corp.intel.com',
                 ot_server='127.0.0.1',
                 ot_client='127.0.0.1',
                 rigid_body='AMR_101',
                 shared_folder=r'\\optitrack.ger.corp.intel.com\GTService\output\OptitrackResults\PosesFiles',
                 log=True):
        self.server = server
        self.ot_server = ot_server
        self.ot_client = ot_client
        self.rigid_body = rigid_body
        self.shared_folder = shared_folder

        self.opti = self.create_instance(server, ot_server, ot_client, log)
        self.rel_path = None

    @staticmethod
    def create_instance(server, ot_server, ot_client, log=True):
        return OptiClient(server, ot_server, ot_client, log)

    def rb_init(self):
        self.rel_path = os.path.join(self.opti.request_init(self.rigid_body), 'Optitrack', f'{self.rigid_body}.csv')

    def in_range(self):
        return self.opti.request_is_in_range().read() == b'true'

    def start(self):
        self.opti.request_start()

    def stop(self):
        self.opti.request_stop()
        self.opti.request_shutdown()

    def copy_csv(self, dst_dir):
        shutil.copy(os.path.join(self.shared_folder, self.rel_path),
                    os.path.join(dst_dir, 'opti_pose_list.csv'))
