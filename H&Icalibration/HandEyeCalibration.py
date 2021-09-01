import numpy as np
import math
import os
import sys
import csv
import copy
from DataHolder import DataHolder
from CalibrationUtils import Filter, FilterFormat, RelativePoses
import CalibrationUtils
from CalibrationConstants import CalibrationConstants
sys.path.append(os.path.join(os.path.dirname(__file__), '..\\..\\..\\3rdparty\\python'))
import transformations as tm
sys.path.append(os.path.join(os.path.dirname(__file__), '..\\Common\\python\\PosesReader'))
from PosesReader import Format_File, Poses_Container, Poses_Utils


class Calibrator:

    def __init__(self, gt_data, src_data, ghc):

        self.ghc_mat = [ghc[i:i + 4] for i in range(0, len(ghc), 4)]
        self.ghc_mat.append([0, 0, 0, 1])
        self.ghc_mat = np.array(self.ghc_mat, dtype=np.float64)

        self.score = sys.float_info.max

        for holder in [gt_data, src_data]:
            holder.container.poses = sorted(holder.container.poses, key=lambda k: k['timestamp']) if not CalibrationUtils.is_sorted(holder.container.poses, key=lambda x: x['timestamp']) else holder.container.poses #sort poses by timestamps if not sorted

        src_data.add_sixdof_poses()
        DataHolder.apply_list_on_holders([gt_data, src_data], [DataHolder.time_to_ms, DataHolder.translation_to_mm, DataHolder.remove_duplicates, DataHolder.add_magnitude, DataHolder.add_rotation_matrix])

        # update containers formats accordingly to changes
        Poses_Utils.apply_updates_on_formats([gt_data.container.format.general_definition, src_data.container.format.general_definition], {'time_factor_to_ms': 1, 'translation_factor_to_mm': 1})

        DataHolder.remove_unique_poses_by_key([gt_data, src_data])

        #Filter.apply_filters([FilterFormat(holders=[gt_data], func=Filter.filter_by_error), FilterFormat(holders=[gt_data], func=Filter.filter_by_status), FilterFormat(holders=[src_data], func=Filter.filter_by_confidence)])
        Filter.apply_filters([FilterFormat(holders=[gt_data, src_data], func=Filter.filter_by_validity)])

        self.rel_rotation = RelativePoses(gt_data, src_data, CalibrationConstants.interval_in_ms_r)
        self.rel_rotation.rel_gt.name += "_r"
        self.rel_rotation.rel_src.name += "_r"
        self.rel_translation = RelativePoses(gt_data, src_data, CalibrationConstants.interval_in_ms_t)
        self.rel_translation.rel_gt.name += "_t"
        self.rel_translation.rel_src.name += "_t"
        self.rel_score = RelativePoses(gt_data, src_data, CalibrationConstants.interval_in_ms_s)
        self.rel_score.rel_gt.name += "_s"
        self.rel_score.rel_src.name += "_s"
        self.gt_file_name = gt_data.container.file_name
        self.src_file_name = src_data.container.file_name
        if any(len(rel_holder.container.poses) < CalibrationConstants.min_relative_matrices_num for rel_holder in [self.rel_rotation.rel_gt, self.rel_rotation.rel_src, self.rel_translation.rel_gt, self.rel_translation.rel_gt, self.rel_score.rel_gt, self.rel_score.rel_gt]):
            raise Exception("not enough relative matrices")

    def Tsai_algorithm(self, t_xyz=None):
        '''
            calculate transformation matrix between 2 systems according to Tsai algorithm
            :param t_xyz: known x, y, and z translation
            this function runs Tsai algorithm on relative matrices
        '''

        self.ghc_mat = self.Tsai_rotation(self.rel_rotation.rel_gt, self.rel_rotation.rel_src)
        self.ghc_mat = self.Tsai_translation(self.rel_translation.rel_gt, self.rel_translation.rel_src, self.ghc_mat, t_xyz)

    @staticmethod
    def Tsai_rotation(rel_gt, rel_src):
        '''
            calculate rotation part in transformation matrix between 2 systems according to Tsai algorithm
            :param rel_gt: relative matrices based ground-truth data
            :param rel_src: relative matrices based source data
            :return: GHC matrix with rotation part
        '''

        size = len(rel_src.container.poses) - 1  # ? to reduce 1?
        A = np.zeros(shape=(3 * size, 3))
        B = np.zeros(shape=(3 * size, 1))

        mask = Filter.intersect_masks([rel_gt.mask, rel_src.mask])

        print("positive poses in mask for Tsai rotation:", Filter.count_positive_poses(mask), "from", len(mask), "poses")

        ind = 0
        for ij in range(size):
            if mask[ij] == 1:

                Pgij = np.dot(2, Calibrator.rot2quat(rel_gt.container.poses[ij]['rotmat']))# ... and the corresponding quaternion
                Pcij = np.dot(2, Calibrator.rot2quat(rel_src.container.poses[ij]['rotmat']))

                skewA = Calibrator.skew_func(Pgij + Pcij)

                A[ind, 0:3] = skewA[0:3]
                A[(ind + 1), 0:3] = skewA[3:6]
                A[(ind + 2), 0:3] = skewA[6:9]
                ind = ind + 3
                B[(ind - 3):ind, 0] = Pcij - Pgij  # right-hand side

        Pcg_ = np.linalg.lstsq(A, B,rcond=-1)[0] #Solve the equation A*Pcg_ = B

        Pcg = 2 * Pcg_ / np.sqrt(1 + np.dot(Pcg_.transpose(), Pcg_))

        return Calibrator.quat2rot(Pcg / 2)  # GHC

    @staticmethod
    def Tsai_translation(rel_gt, rel_src, ghc, t_xyz=None):
        '''
                calculate translation part in transformation matrix between 2 systems according to Tsai algorithm
                :param rel_gt: relative matrices based ground-truth data
                :param rel_src: relative matrices based source data
                :param ghc: GHC matrix with computed rotation part
                :param t_xyz: known x, y, and z translation if exist
                :return: GHC matrix with translation part, based given rotation matrix
        '''

        trans_mat = np.eye(4)

        # Add given translation
        if t_xyz is not None:
            trans_mat[0, 3] = t_xyz[0]
            trans_mat[1, 3] = t_xyz[1]
            trans_mat[2, 3] = t_xyz[2]
        else:
            mask = Filter.intersect_masks([rel_gt.mask, rel_src.mask])
            print("positive poses in mask for Tsai translation:", Filter.count_positive_poses(mask), "from", len(mask),
                  "poses")
            size = len(rel_src.container.poses) - 1  # ? to reduce 1?
            A = np.zeros(shape=(3 * size, 3))
            B = np.zeros(shape=(3 * size, 1))
            ind = 0
            for ij in range(size):
                if mask[ij] == 1:

                    Hij_gt = rel_gt.container.poses[ij]['rotmat']
                    Hij_src = rel_src.container.poses[ij]['rotmat']

                    A[ind:ind + 3, 0:3] = np.subtract(Hij_gt[0:3, 0:3], np.eye(3))[0:3, 0:3]
                    B[ind:ind + 3, 0] = np.dot(ghc[0:3, 0:3], Hij_src[0:3, 3]) - Hij_gt[0:3, 3]  #right-hand side
                    ind = ind + 3

            Tcg = np.linalg.lstsq(A, B, rcond=-1)[0]
            trans_mat[0, 3] = Tcg[0]
            trans_mat[1, 3] = Tcg[1]
            trans_mat[2, 3] = Tcg[2]

        return np.dot(ghc, trans_mat)

    @staticmethod
    def skew_func(v):
        s = [0, -v[2], v[1],
             v[2], 0, -v[0],
             -v[1], v[0], 0]
        return s

    @staticmethod
    def quat2rot(q):
        '''
            convert quaternion to rotation matrix
            :param q: quaternion in 3 dimensions
            :return: rotation matrix
        '''
        p = np.dot(q.transpose(), q)

        if p > 1:
            np.disp('Warning: quat2rot: quaternion greater than 1: ', p)

        w = np.sqrt(1 - p)  # w = np.cos(np.theta / 2)

        rot = np.eye(4)

        arr = np.array([p, p, p])

        q1 = Calibrator.skew_func(q)
        q2 = np.array(q1)
        q2 = q2.reshape(3, 3)

        rot[0:3, 0:3] = 2 * np.dot(q, q.transpose()) + 2 * w * q2 + np.eye(3) - 2 * np.diagflat(arr)
        return rot

    @staticmethod
    def rot2quat(R):
        '''
            convert rotation matrix to quaternion
            :param R: rotation matrix
            :return: quaternion in 3 dimensions
        '''
        w4 = 2 * np.sqrt(1 + np.trace(R[0:3, 0:3]))

        quat1 = (R[2, 1] - R[1, 2]) / w4
        quat2 = (R[0, 2] - R[2, 0]) / w4
        quat3 = (R[1, 0] - R[0, 1]) / w4

        q = [quat1, quat2, quat3]
        return q

    def calculate_score(self, final=False):
        '''
            calculate score of current GHC matrix, based RPE
            :param final: True for final score, False for intermediate score
            :return: score of GHC matrix, based RPE
        '''
        if not final:
            for relative_matrices in [self.rel_rotation, self.rel_translation]:
                relative_matrices.score = Calibrator.calculate_RPE_avg(relative_matrices.rel_gt, relative_matrices.rel_src, self.ghc_mat)
                Filter.apply_filters([FilterFormat(holders=[relative_matrices.rel_gt], func=Filter.filter_by_RPE,
                                                   params={"RPE_avg": relative_matrices.score})])
            self.score = self.rel_rotation.score
            return
        self.score = Calibrator.calculate_RPE_avg(self.rel_score.rel_gt, self.rel_score.rel_src, self.ghc_mat)

    def calculate_RRE(self):
        return Calibrator.calculate_RRE_avg(self.rel_score.rel_gt, self.rel_score.rel_src, self.ghc_mat)

    @staticmethod
    def calculate_RPE_avg(rel_gt, rel_src, ghc):
        '''
            calculate RPE between GT relative matrices and source relative matrices
            :param rel_gt: relative matrices based GT data
            :param rel_src: relative matrices based source data
            :param ghc: transformation matrix between GT world to source world
            :return: average of RPE
        '''
        euclidean_diff = []
        for rel_gt_pose, rel_src_pose in zip(rel_gt.container.poses, rel_src.container.poses):
            projected_rel_gt = np.dot(np.dot(np.linalg.inv(ghc), rel_gt_pose['rotmat']), ghc)
            diff = np.linalg.norm(rel_src_pose['rotmat'][0:3, 3] - projected_rel_gt[0:3, 3])
            rel_gt_pose['dist'] = math.fabs(diff)
            euclidean_diff.append(diff)
        RPE_avg = np.mean(euclidean_diff)
        return RPE_avg

    @staticmethod
    def calculate_RRE_avg(rel_gt, rel_src, ghc):
        '''
            calculate RRE between GT relative matrices and source relative matrices
            :param rel_gt: relative matrices based GT data
            :param rel_src: relative matrices based source data
            :param ghc: transformation matrix between GT world to source world
            :return: average of RRE
        '''
        magnitude_diff = []
        for rel_gt_pose, rel_src_pose in zip(rel_gt.container.poses, rel_src.container.poses):
            projected_rel_gt = np.dot(np.dot(np.linalg.inv(ghc), rel_gt_pose['rotmat']), ghc)
            projected_rel_gt_q = tm.quaternion_from_matrix(projected_rel_gt)
            m_quat = tm.quaternion_multiply([rel_src_pose['qw'], rel_src_pose['qx'], rel_src_pose['qy'], rel_src_pose['qz']], tm.quaternion_inverse(projected_rel_gt_q))
            diff = min(math.pi*2 - (2 * math.acos(max(min(m_quat[0], 1), -1))),2 * math.acos(max(min(m_quat[0], 1), -1)))#m_quat[0] = qw
            magnitude_diff.append(diff)
        RRE_avg = np.mean(magnitude_diff)
        return math.degrees(RRE_avg)

    def project_rel_gt(self):
        '''
            project GT relative poses according to current GHC matrix
            :return: GT relative matrices projected to source world
        '''
        return Calibrator.project_relative_poses(self.rel_score.rel_gt, self.ghc_mat)

    @staticmethod
    def project_relative_poses(rel_gt, ghc):
        '''
            project poses according to given GHC matrix
            :param rel_gt: relative matrices based GT data
            :param ghc: transformation matrix between GT world to source world
            :return: GT relative matrices projected according to GHC matrix
        '''
        projected_holder = copy.deepcopy(rel_gt)
        for pose in projected_holder.container.poses:
            pose['rotmat'] = np.dot(np.dot(np.linalg.inv(ghc), pose['rotmat']), ghc)
            q = tm.quaternion_from_matrix(pose['rotmat'])
            pose['qw'], pose['qx'], pose['qy'], pose['qz'] = q[0], q[1], q[2], q[3]
            pose['x'], pose['y'], pose['z'] = pose['rotmat'][0:3, 3]
        return projected_holder

    def print_results(self, GT_file_path, outdir):
        '''
            print calibration results to file and to screen
            :param GT_file_path: path to GT data location
            :param outdir: path to output directory
        '''

        if outdir is None:
            outdir = os.path.dirname(os.path.abspath(GT_file_path))

        # output file with the GHC
        rb_name = (os.path.basename(GT_file_path).replace('INTER_', '')).replace('.csv', '')
        with open(outdir + "\\" + rb_name + '.csv', 'w') as f:
            writer = csv.writer(f)
            writer.writerows(self.ghc_mat)

            ghc_str = "%.16f %.16f %.16f %.16f %.16f %.16f %.16f %.16f %.16f %.16f %.16f %.16f \n" \
                      % (self.ghc_mat[0][0], self.ghc_mat[0][1], self.ghc_mat[0][2],
                         self.ghc_mat[0][3],
                         self.ghc_mat[1][0], self.ghc_mat[1][1], self.ghc_mat[1][2],
                         self.ghc_mat[1][3],
                         self.ghc_mat[2][0], self.ghc_mat[2][1], self.ghc_mat[2][2],
                         self.ghc_mat[2][3])

        print ('GHC matrix: ', ghc_str)
        with open(outdir + "\\" + rb_name + '.txt', 'w') as ghcFile:
            ghcFile.write('GHC_mat=' + ghc_str)

        f.close()
        ghcFile.close()