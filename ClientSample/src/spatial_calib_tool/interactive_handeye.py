import os, os.path as osp
import shutil
import pickle
import argparse
from enum import Enum

import pyrealsense2 as rs
import numpy as np
import pandas as pd
import cv2
from datetime import datetime

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from scipy.spatial.transform import Rotation as R
from scipy.optimize import minimize


from common.Utils import Utils, ImageChessboardPose, OptitrackClient


class RecordMode(Enum):
    VIEW = 1
    COLLECT = 2



def csv2pose(path, avg=True):

    df = pd.read_csv(path)

    mat_cols = ['r_0', 'r_1', 'r_2', 'r_3', 'r_4', 'r_5', 'r_6', 'r_7', 'r_8']
    quat_cols = ['qx', 'qy', 'qz', 'qw']
    trans_cols = ['x', 'y', 'z']

    markers_in_scence_col_idx = 22
    markers_in_scence_col = df.columns[markers_in_scence_col_idx]
    rb_n_markers = 8

    df = df[(df['tracked'] == 1)]  ### & (df[error_cols] < 0.001)]
    df = df[(df[markers_in_scence_col] <= rb_n_markers)]

    if len(df) == 0:
        print('no valid opti pose')
        return None


    df_mat = df[mat_cols].astype('float').values.reshape((-1, 3, 3))
    df_quat = df[quat_cols].astype('float').values.reshape((-1, 4))
    df_trans = df[trans_cols].astype('float').values.reshape((-1, 3))

    if np.isnan(df_quat).any():
        print('is nan')
        return None

    rot_mag_std = np.linalg.norm(R.from_matrix(df_mat).as_rotvec(), axis=1).std()

    # R.from_matrix(df_mat).mean().as_euler('xyz', degrees=True)
    # R.from_quat(df_quat).mean().as_euler('xyz', degrees=True)

    rot_mag_std_deg = rot_mag_std * 180 / np.pi
    print('opti rot mag std (deg):', rot_mag_std_deg)



    order = 'xyz'
    # order = 'xzy'
    # order = 'yxz'
    # order = 'yzx'
    # order = 'zxy'
    # order = 'zyx'

    def inv_(m):
        if False:
            return inv(m)
        else:
            return m

    # print('euler (10 first mat):', matrix2euler(inv_(df_mat[:20]), order))
    # print('euler (from quat mean):', matrix2euler(inv_(R.from_quat(df_quat).mean().as_matrix()), order))
    # print('euler (from mat mean):', matrix2euler(inv_(R.from_matrix(df_mat).mean().as_matrix()), order))

    # print('trans (10 first mat):', df_trans[:20])
    # print('trans mean:', df_trans.mean(axis=0))
    #
    # print('quat (10 first quat):', df_quat[:20])
    # print('quat mean', R.from_quat(df_quat).mean().as_quat())

    # if rot_mag_std_deg > 0.02:
    # if rot_mag_std_deg > 0.01:
    if rot_mag_std_deg > 0.015:
        print('please take agin from *slightly* different pose')
        return None
    return Utils.homogeneous(R.from_quat(df_quat).mean().as_matrix(), df[trans_cols].astype('float').values.mean(axis=0))
    return Utils.homogeneous(R.from_matrix(df_mat).mean().as_matrix(), df[trans_cols].astype('float').values.mean(axis=0))


def arr2file(arr, path):
    pd.DataFrame(arr).to_csv(f'{path}.csv', header=False, index=False)
    with open(f'{path}.pkl', 'wb') as f:
        pickle.dump(arr, f)


def save_data(cam_frames, opti_path, opti_share, output_folder, name, pair_index, cam_pose, opti_pose):
    out_dir = osp.join(output_folder, name, f'pair_{pair_index:04d}')
    color_dir = osp.join(out_dir, 'color')
    depth_dir = osp.join(out_dir, 'depth')
    ir_dir = osp.join(out_dir, 'ir')

    for d in [color_dir, depth_dir, ir_dir]:
        os.makedirs(d, exist_ok=True)

    for i, (color, depth, ir) in enumerate(cam_frames):
        curr_im = f'im{i:03d}.png'
        cv2.imwrite(osp.join(color_dir, curr_im), color)
        cv2.imwrite(osp.join(depth_dir, curr_im), depth)
        cv2.imwrite(osp.join(ir_dir, curr_im), ir)

    shutil.copy(osp.join(opti_share, opti_path), osp.join(out_dir, 'opti_pose_list.csv'))
    arr2file(opti_pose, osp.join(out_dir, 'opti_pose'))
    arr2file(cam_pose, osp.join(out_dir, 'cam_pose'))


def save_metadata(output_folder, cam, dist, c2d, board_h, board_w, board_mm_h, board_mm_w):
    os.makedirs(output_folder, exist_ok=True)

    arr2file(cam, osp.join(output_folder, 'cam'))
    arr2file(dist, osp.join(output_folder, 'dist'))
    arr2file(c2d, osp.join(output_folder, 'color2depth'))
    arr2file([board_h, board_w, board_mm_h, board_mm_w], osp.join(output_folder, 'board_params'))


def save_pose_list(out_path, bHg, wHc, dtype):
    with open(osp.join(out_path, f'bHg_{dtype}.pkl'), 'wb') as f:
        pickle.dump(bHg, f)
    with open(osp.join(out_path, f'wHc_{dtype}.pkl'), 'wb') as f:
        pickle.dump(wHc, f)


def estimate_hand_eye(bHg, wHc, initial_ghc=np.eye(4)):
    # if len(bhg) != len(whc) or len(bhg) < 4:
    #     return None

    def coord(ax, i, pose, factor):
        x_axis = [factor, 0, 0]
        y_axis = [0, factor, 0]
        z_axis = [0, 0, factor]

        p = pose[:3, 3]
        ax.text(*p, f'{i}')

        x = pose[:3, :3] @ x_axis + p
        ax.plot3D(*[[a, b] for a, b in zip(p, x)], c='r')
        y = pose[:3, :3] @ y_axis + p
        ax.plot3D(*[[a, b] for a, b in zip(p, y)], c='g')
        z = pose[:3, :3] @ z_axis + p
        ax.plot3D(*[[a, b] for a, b in zip(p, z)], c='b')

    def plot_3d(arr1, arr2, arr0=None):
        fig = plt.figure()
        ax = Axes3D(fig)

        ax.plot3D([-1000, 1000], [0, 0], [0, 0], c='r')
        ax.plot3D([0, 0], [-1000, 1000], [0, 0], c='g')
        ax.plot3D([0, 0], [0, 0], [-1000, 1000], c='b')

        ax.plot3D(arr1[:, 0, 3], arr1[:, 1, 3], arr1[:, 2, 3], marker='o', label='gt_est')
        ax.plot3D(arr2[:, 0, 3], arr2[:, 1, 3], arr2[:, 2, 3], marker='o', label='cam_est')
        if arr0 is not None:
            ax.plot3D(arr0[:, 0, 3], arr0[:, 1, 3], arr0[:, 2, 3], marker='o', label='gt_rel')

        for i in range(len(arr1)):
            factor = 4
            coord(ax, i, arr1[i], factor * 10)
            coord(ax, i, arr2[i], factor * 10)
            if arr0 is not None:
                coord(ax, i, arr0[i], factor * 10)

        plt.legend()
        plt.show(block=True)


    if len(bHg) == len(wHc) and len(bHg) > 3:
        whc = np.asarray(wHc)
        chw = Utils.inv(whc)
        bhg = np.asarray(bHg)
        ghb = Utils.inv(bhg)

        gt_rel = ghb[0] @ bhg
        # cam_rel = chw[0] @ whc
        cam_rel = whc[0] @ chw

        he_R, he_T = cv2.calibrateHandEye(bhg[:, :3, :3], bhg[:, :3, 3], whc[:, :3, :3], whc[:, :3, 3])

        ghc = Utils.homogeneous(he_R, he_T.flatten())
        chg = Utils.inv(ghc)

        cam_est = cam_rel
        gt_est = chg @ gt_rel @ ghc

        print('initial euler', R.from_matrix(initial_ghc[...,:3,:3]).as_euler('xyz', degrees=True))
        print('initial translation', initial_ghc[...,:3,3].flatten())

        print('res euler', R.from_matrix(he_R).as_euler('xyz', degrees=True))
        print('res translation', he_T.flatten())

        diff = cam_est @ Utils.inv(gt_est)
        trans_err = np.linalg.norm(diff[:, :3, 3], axis=1).mean()
        print('trans error:', trans_err)
        # plot_3d(gt_est, cam_est)
        plot_3d(gt_est, cam_est, gt_rel)

        return ghc, to_6dof(ghc)

    return initial_ghc, to_6dof(initial_ghc)


def estimate_hand_eye_nm_opt(bHg, wHc, initial_ghc=np.eye(4)):
    x0 = np.concatenate([Utils.matrix2euler(initial_ghc), initial_ghc[..., :3, 3]])
    res = minimize(ghc_trans_error, x0, (wHc[0] @ Utils.inv(wHc), Utils.inv(bHg[0]) @ bHg), method='Nelder-Mead')
    # with np.printoptions(suppress=True, precision=3):
    #     print('alg0 res', res0.x)
    #     print('alg0 err', ghc_trans_error(res0.x, wHc[0] @ inv(wHc), inv(bHg[0]) @ bHg))

    return Utils.homogeneous(Utils.euler2matrix(res.x[:3]), res.x[3:]), res.x


def to_6dof(x):
    return np.concatenate([Utils.matrix2euler(x).flatten(), x[..., :3, 3].flatten()])


def ghc_trans_error(x, rel_cam, rel_opti):
    ghc = Utils.homogeneous(Utils.euler2matrix(x[:3]), x[3:])
    rel_est = Utils.inv(ghc) @ rel_opti @ ghc

    diff = Utils.inv(rel_cam) @ rel_est
    trans_error = Utils.trans_dist(diff)

    # print(trans_error.mean(), trans_error.max())
    return trans_error.mean()


def validate_pair(cam_pose, cam_list, opti_pose, opti_list):
    if len(cam_list) and len(opti_list):
        cam_rot_diff = Utils.matrix2mag(Utils.inv(np.asarray(cam_list)) @ cam_pose)
        opti_rot_diff = Utils.matrix2mag(Utils.inv(np.asarray(opti_list)) @ opti_pose)

        res = np.linalg.norm(cam_rot_diff-opti_rot_diff)
        print('validate_pair rot diff score:', res)
        if res > 0.1:
            print(f'invalid pair: {res} diff')
            return False

    return True


def update_ghc(ghc_dict, bHg, wHc):
    for n in ghc_dict:
        res_mat, res = ghc_dict[n]['func'](bHg, wHc, ghc_dict[n]['mat'])
        with np.printoptions(suppress=True, precision=3):
            print(f'{n} [rx, ry, rz, tx, ty, tz]:', res)
            print(f'{n} error on fit data:', ghc_trans_error(res, wHc[0] @ Utils.inv(wHc), Utils.inv(bHg[0]) @ bHg))
        ghc_dict[n]['mat'] = res_mat

    return ghc_dict


def publish_calib(color_cam_mat, color_cam_dist, color2depth, ghc_dict, args, test_case):
    for o in [osp.join(args.res_folder, test_case), args.publish_folder]:
        save_metadata(
            o,
            color_cam_mat, color_cam_dist, color2depth,
            args.board_h, args.board_w,
            args.board_mm_h, args.board_mm_w
        )
        for n in ghc_dict:
            with open(osp.join(o, f'gHc_{n}.pkl'), 'wb') as f:
                pickle.dump(ghc_dict[n]['mat'], f)

    print('******** NOTE: calib updated at', args.publish_folder)



####################################### starts here ####

def calibration_main():

    parser = argparse.ArgumentParser()
    parser.add_argument("--output_folder", default='calib-dc', help='')
    parser.add_argument("--board_h", default=7, help='')
    parser.add_argument("--board_w", default=10, help='')
    parser.add_argument("--board_mm_h", default=47, help='')
    parser.add_argument("--board_mm_w", default=47, help='')
    parser.add_argument("--record_count", default=30, help='')
    parser.add_argument("--opti_host", default='optitrack.ger.corp.intel.com', help='')
    parser.add_argument("--opti_server", default='127.0.0.1', help='')
    parser.add_argument("--opti_client", default='127.0.0.1', help='')
    parser.add_argument("--opti_shared_folder", default=r'\\optitrack.ger.corp.intel.com\GTService\output\OptitrackResults\PosesFiles', help='')
    parser.add_argument("--opti_rigid_body", default='AMR_101', help='')
    parser.add_argument("--n_fit_pairs", default=30, help='')
    parser.add_argument("--n_test_pairs", default=15, help='')
    parser.add_argument("--res_folder", default=r'\\optitrack.ger.corp.intel.com\GTService\amr-dc-data\res', help='')
    parser.add_argument("--publish_folder", default=r'\\optitrack.ger.corp.intel.com\GTService\amr-dc-data\publish', help='')

    args = parser.parse_args()
    curr_test_case = f'calib-{datetime.now().strftime("%Y%m%d-%H%M%S")}'

    opti = OptitrackClient(
        server=args.opti_host,
        ot_server=args.opti_server,
        ot_client=args.opti_client,
        rigid_body=args.opti_rigid_body,
        shared_folder=args.opti_shared_folder,
        log=False)
    opti.rb_init()

    record_mode = RecordMode.VIEW
    able_to_collect = False
    frames_collected = []

    ghc_dict = {
        'tsai': {'mat': np.eye(4), 'func': estimate_hand_eye},
        'nelder-mead-from-scratch': {'mat': np.eye(4), 'func': estimate_hand_eye_nm_opt},
        'nelder-mead-from-guess': {'mat': Utils.homogeneous(R.from_euler('xyz', [-63.272, -5.567, 87.657], degrees=True).as_matrix(), [-86.303, 52.676, -51.188]), 'func': estimate_hand_eye_nm_opt}
    }

    main_alg = 'nelder-mead-from-guess'

    bHg = []
    wHc = []

    bHg_test = []
    wHc_test = []


    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # todo: parameterize
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # todo: parameterize
    config.enable_stream(rs.stream.infrared, 640, 480, rs.format.y8, 30)  # todo: parameterize

    align_processor = rs.align(rs.stream.color)

    # Start streaming
    profile = pipeline.start(config)
    depth_profile = profile.get_stream(rs.stream.depth)
    color_profile = profile.get_stream(rs.stream.color)
    color_vs_profile = color_profile.as_video_stream_profile()
    color_cam_intrinsics = color_vs_profile.get_intrinsics()
    # color_cam_mat = np.eye(3)
    # color_cam_mat[0, 0] = color_cam_intrinsics.fx
    # color_cam_mat[1, 1] = color_cam_intrinsics.fy
    # color_cam_mat[0, 2] = color_cam_intrinsics.ppx
    # color_cam_mat[1, 2] = color_cam_intrinsics.ppy

    color_cam_mat = np.asarray([
        [color_cam_intrinsics.fx, 0, color_cam_intrinsics.ppx],
        [0, color_cam_intrinsics.fy, color_cam_intrinsics.ppy],
        [0, 0, 1]
    ])

    color_cam_dist = np.asarray(color_cam_intrinsics.coeffs)

    color2depth = color_profile.get_extrinsics_to(depth_profile)
    color2depth = Utils.homogeneous(np.asarray(color2depth.rotation).reshape((3, 3)), np.asarray(color2depth.translation) * 1000)

    im_pose = ImageChessboardPose()
    im_pose.init_from_data(args.board_h, args.board_w, args.board_mm_h, args.board_mm_w, color_cam_mat, color_cam_dist)


    save_metadata(
        osp.join(args.output_folder, curr_test_case),
        color_cam_mat, color_cam_dist, color2depth,
        args.board_h, args.board_w,
        args.board_mm_h, args.board_mm_w
    )

    if False:
        device = profile.get_device()
        depth_sensor = device.query_sensors()[0]
        emitter = depth_sensor.get_option(rs.option.emitter_enabled)
        print("emitter = ", emitter)
        set_emitter = 0
        depth_sensor.set_option(rs.option.emitter_enabled, set_emitter)
        emitter1 = depth_sensor.get_option(rs.option.emitter_enabled)
        print("new emitter = ", emitter1)

    all_pair_count = 0

    try:
        while True:

            # Wait for a coherent pair of frames: depth and color
            orig_frames = pipeline.wait_for_frames()
            frames = align_processor.process(orig_frames)

            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            infrared_frame = frames.get_infrared_frame()
            if not depth_frame or not color_frame:
                continue

            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            infrared_image = np.asanyarray(infrared_frame.get_data())

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            depth_colormap_dim = depth_colormap.shape
            color_colormap_dim = color_image.shape

            if record_mode is RecordMode.VIEW:
                alpha = 0.6
                beta = 1 - alpha

                color_image = color_image.copy()
                cam_pose, reproj_error, corners = im_pose.get_pose(color_image, draw_corners=False, draw_coord=True)
                bright_rect = color_image[20:100, 5:250]
                fac = 200 if len(bHg) < args.n_fit_pairs else 255
                bright_rect[...] = (bright_rect * alpha + beta * np.asarray([255, fac, fac])).astype('uint8')

                if cam_pose is not None:
                    color_image = cv2.putText(color_image, f're-projection score: {reproj_error :.3f}', (10, 50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0, 0, 255))

                if len(bHg) < args.n_fit_pairs:
                    color_image = cv2.putText(color_image, f'{len(bHg)} good pairs / {args.n_fit_pairs}', (10, 70), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0, 0, 255))
                else:
                    color_image = cv2.putText(color_image, f'{len(bHg_test)} test pairs / {args.n_test_pairs}', (10, 70), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0, 100, 255))

                in_range = opti.in_range()
                if (
                        cam_pose is None
                        # or reproj_error > thresh
                        or not in_range
                ):
                    color_image = (color_image * alpha + beta * np.asarray([0, 0, 255])).astype('uint8')

                    able_to_collect = False
                else:
                    able_to_collect = True

            # If depth and color resolutions are different, resize color image to match depth image for display
            if depth_colormap_dim != color_colormap_dim:
                resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
                images = np.hstack((resized_color_image, depth_colormap))
            else:
                images = np.hstack((color_image, depth_colormap))

            if record_mode is RecordMode.COLLECT:

                remain_collect = args.record_count - len(frames_collected)
                if remain_collect == 0:
                    opti.stop()
                    cam_pose, reproj_error, corners = im_pose.get_pose(
                        np.median([c for c, d, ir in frames_collected], axis=0).astype('uint8'),
                        draw_corners=False, draw_coord=False
                    )

                    opti_pose = csv2pose(osp.join(args.opti_shared_folder, opti.rel_path))

                    save_data(
                        frames_collected, opti.rel_path,
                        args.opti_shared_folder, args.output_folder, curr_test_case,
                        # len(wHc), cam_pose, opti_pose
                        all_pair_count + (0 if len(bHg) < args.n_fit_pairs else 1000), cam_pose, opti_pose
                    )

                    all_pair_count += 1

                    if cam_pose is not None and opti_pose is not None and validate_pair(cam_pose, wHc, opti_pose, bHg):
                        if len(bHg) < args.n_fit_pairs:
                            bHg += [opti_pose]
                            wHc += [cam_pose]
                            if len(bHg) == args.n_fit_pairs:
                                ghc_dict = update_ghc(ghc_dict, bHg, wHc)
                        else:
                            bHg_test += [opti_pose]
                            wHc_test += [cam_pose]

                            for n in ghc_dict:
                                with np.printoptions(suppress=True, precision=3):
                                    res = to_6dof(ghc_dict[n]['mat'])
                                    # print(f'{n} [rx, ry, rz, tx, ty, tz]:', res)
                                    print(f'{n} test error:', ghc_trans_error(res, wHc_test[0] @ Utils.inv(wHc_test), Utils.inv(bHg_test[0]) @ bHg_test))

                            # todo: estimate ghc

                        # initial_ghc = estimate_hand_eye(bHg, wHc, initial_ghc)
                        # estimate_hand_eye(bHg, wHc, initial_ghc)
                        # todo: estimate hand_eye + score
                    else:
                        print('note: Did not add new pair')


                    record_mode = RecordMode.VIEW
                    opti.rb_init()
                    frames_collected = []   # todo: check maybe its happening twice....

                else:

                    images = cv2.cvtColor(images, cv2.COLOR_BGR2GRAY)
                    images = np.stack([images] * 3, axis=-1)

                    images = cv2.putText(images, f'collecting data, please don\'t move ({remain_collect})', (10, 50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)

                    frames_collected += [(color_image.copy(), depth_image.copy(), infrared_image.copy())]


            # Show images
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            if len(wHc) > 0:
                # cam_est = chw[0] @ whc[0] @ chw
                # gt_est = chw[0] @ chg @ ghb[0] @ bhg @ ghc
                cam_est = Utils.inv(wHc[-1])
                gt_est = Utils.inv(wHc[0]) @  Utils.inv(ghc_dict[main_alg]['mat']) @ Utils.inv(bHg[0]) @ bHg[-1] @ ghc_dict[main_alg]['mat']

                # images = im_pose.draw_board(images, inv(cam_est), [0, 255, 0])
                images = im_pose.draw_board(images, Utils.inv(gt_est), [255, 0 , 255])


            cv2.imshow('RealSense', images)
            key = cv2.waitKey(1)
            tfactor = 10    # 1.0
            if key == ord('t'):
                # estimate_hand_eye(bHg, wHc, initial_ghc)
                save_pose_list(osp.join(args.output_folder, curr_test_case), bHg, wHc, 'calc')
                save_pose_list(osp.join(args.output_folder, curr_test_case), bHg_test, wHc_test, 'test')
            if record_mode == RecordMode.VIEW:
                # Press esc or 'q' to close the image window
                if (key & 0xFF == ord('q') or key == 27):
                    ghc_dict = update_ghc(ghc_dict, bHg, wHc)
                    for n in ghc_dict:
                        if len(bHg_test) > 0:
                            res = to_6dof(ghc_dict[n]['mat'])
                            print(f'{n} test error:',
                                  ghc_trans_error(res, wHc_test[0] @ Utils.inv(wHc_test), Utils.inv(bHg_test[0]) @ bHg_test))

                        with open(osp.join(args.output_folder, curr_test_case, f'gHc_{n}.pkl'), 'wb') as f:
                            pickle.dump(ghc_dict[n]['mat'], f)

                    save_pose_list(osp.join(args.output_folder, curr_test_case), bHg, wHc, 'calc')
                    save_pose_list(osp.join(args.output_folder, curr_test_case), bHg_test, wHc_test, 'test')

                    publish_calib(color_cam_mat, color_cam_dist, color2depth, ghc_dict, args, curr_test_case)
                    cv2.destroyAllWindows()
                    break
                elif key == ord('s') and able_to_collect:
                    record_mode = RecordMode.COLLECT
                    frames_collected = []
                    opti.start()
                elif key == ord('g'): # aka good enough :)
                    publish_calib(color_cam_mat, color_cam_dist, color2depth, ghc_dict, args, curr_test_case)
                elif key == ord('c'):
                    ghc_dict = update_ghc(ghc_dict, bHg, wHc)

    finally:
        # Stop streaming
        # pipeline.stop()
        del pipeline

        # todo: save bhg
        # todo: save chw


if __name__ == '__main__':
    calibration_main()
