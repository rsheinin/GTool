import argparse
import os
import time
from collections import deque
import numpy as np
from datetime import datetime
import pickle
from queue import Queue
from enum import Enum, Flag

import pyrealsense2 as rs

import cv2

import threading

import shutil
from ServiceClient.OptiClient import OptiClient

from capture_post_process import post_process, post_process_S, post_process_M
print(post_process_S, post_process_M)

def xyz2dict(data):
    return {'x': data.x, 'y': data.y, 'z': data.z}


#
# class WorkerIMU(threading.Thread):
#     def __init__(self):
#         super().__init__()
#         self.stop_event = threading.Event()
#         self.pipeline = None
#         self.save_stream = False
#         self.history = deque(maxlen=1024)
#         self.alpha = 0.9
#         self.history.append({'ts': 0, 'theta': None})
#
#     def init_device(self):
#         self.pipeline = rs.pipeline()
#         config = rs.config()
#         config.enable_stream(rs.stream.gyro)
#         config.enable_stream(rs.stream.accel)
#         self.pipeline.start(config)
#
#     def get(self, ts=None):
#         ret_val = self.history[-1]
#         if ts is None:
#             pass
#         elif ts > self.history[-1]['ts']:
#             pass
#         elif ts < self.history[0]['ts']:
#             ret_val = self.history[0]
#         else:
#             for i in range(len(self.history) - 1, 0, -1):
#                 ts1 = self.history[i]['ts']
#                 ts0 = self.history[i - 1]['ts']
#                 if ts0 < ts < ts1:
#                     indx = i - 1 if abs(ts - ts0) < abs(ts - ts1) else i
#                     ret_val = self.history[indx]
#                     break
#
#         return ret_val['theta'], ret_val['ts']
#
#     def update(self, g, a, dt):
#         theta_old = self.history[-1]['theta']
#
#         a_pitch = -np.arctan2(a.z, -a.y)
#         a_roll = -np.arctan2(a.x, np.sqrt(a.y ** 2 + a.z ** 2))
#         if theta_old is None:
#             theta_new = np.array([a_roll, a_pitch, 0])
#         else:
#             g_dtheta = np.array([g.z, -g.x, -g.y]) * dt / 1000
#             theta_new = np.array(theta_old)
#             theta_new[0] = (theta_new[0] + g_dtheta[0]) * self.alpha + a_roll * (1 - self.alpha)
#             theta_new[1] = (theta_new[1] + g_dtheta[1]) * self.alpha + a_pitch * (1 - self.alpha)
#             theta_new[2] += g_dtheta[2]
#         return theta_new[0], theta_new[1], theta_new[2]
#
#     def run(self):
#         self.init_device()
#         while not self.stop_event.isSet():
#             frames = self.pipeline.wait_for_frames(1024)
#             ts = frames.get_timestamp()
#             dt = ts - self.history[-1]['ts']
#             if dt == 0:
#                 continue
#
#             gyro_data = frames.first(rs.stream.gyro).as_motion_frame().get_motion_data()
#             accl_data = frames.first(rs.stream.accel).as_motion_frame().get_motion_data()
#             theta_new = self.update(gyro_data, accl_data, dt)
#             self.history.append({'ts': ts, 'theta': theta_new})
#             # print(np.rad2deg(theta_new))
#
#     def close(self):
#         # Request thread to stop.
#         self.stop_event.set()
#         # Wait for thread to exit.
#         self.join()
#

class WorkerZICC(threading.Thread):

    def init_device(self):
        res = (480, 640)

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, res[1], res[0], rs.format.z16, 30)
        config.enable_stream(rs.stream.color, res[1], res[0], rs.format.bgr8, 30)
        # # config.enable_stream(rs.stream.confidence)
        config.enable_stream(rs.stream.infrared, res[1], res[0])

        config.enable_stream(rs.stream.accel)   #  , rs.format.motion_xyz32f, 200)
        config.enable_stream(rs.stream.gyro)    #   , rs.format.motion_xyz32f, 200)

        profile = self.pipeline.start(config)

        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()

        align_to = rs.stream.depth

        intr = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
        self.cam_k = np.array([intr.fx, 0, intr.ppx, 0, intr.fy, intr.ppy, 0, 0, 1]).reshape(3, 3)


    def __init__(self):
        super().__init__()
        self.stop_event = threading.Event()
        self.queue = Queue(8)
        self.last_data = None
        self.pipeline = None
        self.depth_scale = None
        self.align = None
        self.cam_k = None
        self.save_stream = False
        self.ts_prev = 0

    def close(self):
        # Request thread to stop.
        self.stop_event.set()
        # Wait for thread to exit.
        self.join()

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    def peek_head(self):
        return self.last_data

    def get(self):
        if self.queue.empty():
            return None
        data = self.queue.get_nowait()
        return data

    @staticmethod
    def rsmotion2np(data):
        return np.asarray([data.x, data.y, data.z])

    def capture(self):
        frames = self.pipeline.wait_for_frames()
        ts = frames.get_timestamp()
        dt = ts - self.ts_prev
        if dt == 0:
            return None
        self.ts_prev = ts

        # for f in frames[-2:]:
        #     f.as_motion_frame().get_motion_data()

        depth_im = frames.get_depth_frame()
        color_im = frames.get_color_frame()
        ir_im = frames.get_infrared_frame()

        gyro = None
        gyro_ts = None
        accel = None
        accel_ts = None
        for frame in frames:
            motion = frame.as_motion_frame() or None
            if motion is not None and motion.get_profile().stream_type() == rs.stream.gyro:
                gyro = self.rsmotion2np(motion.get_motion_data())
                gyro_ts = motion.get_timestamp()
            elif motion is not None and motion.get_profile().stream_type() == rs.stream.accel:
                    accel = self.rsmotion2np(motion.get_motion_data())
                    accel_ts = motion.get_timestamp()

        # accel, gyro = [self.rsmotion2np(f.as_motion_frame().get_motion_data()) for f in frames[-2:].copy()]


        if not depth_im or not color_im:
            return None
        color_im = np.asanyarray(color_im.get_data())
        depth_im = np.asanyarray(depth_im.get_data())

        # conf_im = np.asanyarray(conf_im.get_data())
        ir_im = np.asanyarray(ir_im.get_data())
        if ir_im.dtype == np.uint16:
            ir_im = (ir_im >> 8).astype(np.uint8)

        data = {'depth': depth_im, 'depth_scale': self.depth_scale, 'ir': ir_im, 'color': color_im,
                'cam_k': self.cam_k, 'ts': ts, 'dt': dt,
                # 'accel': accel,
                'gyro': gyro,
                'gyro_ts': gyro_ts,
                'accel': accel,
                'accel_ts': accel_ts
                }
        return data

    def run(self):
        self.init_device()
        while not self.stop_event.isSet():
            try:
                data = self.capture()
            except:
                print('capture error.....')
                # todo: verify
                self.pipeline.stop()
                del self.pipeline
                self.init_device()
                data = None

            if data is None:
                continue
            if self.save_stream:
                if self.queue.full():
                    self.queue.get()
                self.queue.put_nowait(data)

            self.last_data = data


class OptitrackClient:
    def __init__(self,
                 server='localhost',
                 ot_server='127.0.0.1',
                 ot_client='127.0.0.1',
                 rigid_body='AMR_101',
                 shared_folder=r'F:\GTService\output\OptitrackResults\PosesFiles'):
        self.server = server
        self.ot_server = ot_server
        self.ot_client = ot_client
        self.rigid_body = rigid_body
        self.shared_folder = shared_folder

        self.opti = self.create_instance(server, ot_server, ot_client)
        self.rel_path = None

    @staticmethod
    def create_instance(server, ot_server, ot_client):
        return OptiClient(server, ot_server, ot_client)

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


# # class RecordPhase(Flag):
# class RecordPhase(Enum):
#     PHASE_NONE = 0
#     PHASE_STAT_I = 1
#     PHASE_CALIB_MOVE_I = 2
#     PHASE_STAT_II = 3
#     PHASE_RECORD_MOVE = 4

__VERSION__ = '1.0.6'
if __name__ == '__main__':
    print(f"Capture tool version {__VERSION__}")
    parser = argparse.ArgumentParser()
    default_output_folder = os.path.join(os.path.expanduser("~"), "record")
    default_output_folder = os.path.join(r'F:\AMR', "record")
    parser.add_argument("-o-", "--output_folder", type=str, default=default_output_folder, required=False)
    parser.add_argument("--opti", default=False, action='store_true')
    args = parser.parse_args()

    base_output_folder = args.output_folder
    output_folder = None
    DISP_MAX_DIST = 3


    optiClient = None
    if args.opti:
        optiClient = OptitrackClient()
        optiClient.rb_init()

    imCapture = WorkerZICC()
    imCapture.start()

    # imuCapture = WorkerIMU()
    # imuCapture.start()

    static_frames_i = 0
    static_frames_ii = 0
    calib_frames = 0
    file_index = 0
    # record_phase = RecordPhase.PHASE_NONE

    print(f"Saving data to {base_output_folder}")
    while True:

        disp_data = imCapture.peek_head()

        if disp_data is None:
            # print("No data")
            continue

        thr = np.uint16(DISP_MAX_DIST / disp_data['depth_scale'])
        detph_rgb = np.clip(disp_data['depth'], 0, thr)

        detph_rgb = (detph_rgb.astype(np.uint32) * 255 // thr).astype(np.uint8)
        detph_rgb = cv2.cvtColor(detph_rgb, cv2.COLOR_GRAY2RGB)
        ir_rgb = cv2.applyColorMap(disp_data['ir'], cv2.COLORMAP_BONE)
        # conf_rgb = cv2.applyColorMap(disp_data['conf'], cv2.COLORMAP_MAGMA)
        disp_img = np.r_[
            np.concatenate([detph_rgb, disp_data['color']], axis=1), np.concatenate([ir_rgb, ir_rgb * 0], axis=1)]
        disp_img = disp_img[::2, ::2].copy()

        if optiClient is not None and not optiClient.in_range():
            alpha = 0.6
            beta = 1 - alpha
            disp_img = (disp_img * alpha + beta * np.asarray([0, 0, 255])).astype('uint8')

        # rot_data, _ = imuCapture.get()
        # rot_data = np.rad2deg(rot_data)

        txt = "fps:{:5.3f}".format(1000 / disp_data['dt'])
        # txt += ' {:+3.1f}, {:+3.1f}, {:+3.1f}'.format(*rot_data)

        # with np.printoptions(suppress=True, precision=3):
        #     txt += f' {disp_data["gyro"]}'

        # txt += f' capture phase: {str(record_phase)}'

        cv2.putText(disp_img, txt, (20, disp_img.shape[0] - 20), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1,
                    (0, 0, 255))

        if imCapture.save_stream:

            while True:

                if static_frames_i > 0:
                    for i, txt in enumerate([f'Please Don\'t move', f'({static_frames_i})', f'static phase I']):
                        cv2.putText(disp_img, f'{txt}',
                                    (disp_img.shape[1] // 2, disp_img.shape[0] // 2 + 30 + 30 * i),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1,
                                    (0, 0, 255))
                    static_frames_i -= 1
                elif calib_frames > 0:
                    for i, txt in enumerate([f'Please do calibration', f'movement ({calib_frames})']):
                        cv2.putText(disp_img, f'{txt}',
                                    (disp_img.shape[1] // 2, disp_img.shape[0] // 2 + 30 + 30 * i),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1,
                                    (0, 0, 255))
                    calib_frames -= 1
                elif static_frames_ii > 0:
                    for i, txt in enumerate([f'Please Don\'t move', f'({static_frames_ii})', f'static phase II']):
                        cv2.putText(disp_img, f'{txt}',
                                    (disp_img.shape[1] // 2, disp_img.shape[0] // 2 + 30 + 30 * i),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1,
                                    (0, 0, 255))
                    static_frames_ii -= 1
                else:
                    for i, txt in enumerate([f'Now you can move', f':)']):
                        cv2.putText(disp_img, f'{txt}',
                                    (disp_img.shape[1] // 2, disp_img.shape[0] // 2 + 30 + 30 * i),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1,
                                    (0, 0, 255))

                save_data = imCapture.get()

                if save_data is None:
                    break

                # save_data['rotation'], save_data['imu_dt'] = imuCapture.get(save_data['ts'])

                fn = r"{}/{:05d}.pkl".format(output_folder, file_index)
                file_index += 1
                def save_pkl_func(fn_, save_data_):
                    with open(fn_, 'wb') as fid:
                        pickle.dump(save_data_, fid)

                x = threading.Thread(target=save_pkl_func, args=(fn, save_data))
                # logging.info("Main    : before running thread")
                x.start()


            disp_img[:10] = [0, 0, 255]
            disp_img[-10:] = [0, 0, 255]
            disp_img[:, -10:] = [0, 0, 255]
            disp_img[:, :10] = [0, 0, 255]

        else:
            for i, txt in enumerate(['Please place the',
                                     'device in a stable',
                                     'position and then',
                                     'press space to',
                                     'start capturing']):
                cv2.putText(disp_img, f'{txt}',
                            (disp_img.shape[1] // 2, disp_img.shape[0] // 2 + 30 + 30*i), cv2.FONT_HERSHEY_SIMPLEX, 1,
                            (0, 0, 255))

        cv2.imshow("RealSense recording tool", disp_img)

        k = cv2.waitKey(1)
        if k == 32:  # space
            if not imCapture.save_stream and ((optiClient is None) or (optiClient is not None and optiClient.in_range())):
                if optiClient is not None:
                    optiClient.start()
                    time.sleep(1)
                imCapture.save_stream = True
                output_folder = '{}/{}'.format(base_output_folder, datetime.now().strftime("%Y%m%d-%H%M%S"))
                os.makedirs(output_folder, exist_ok=True)

                static_frames_i = post_process_S
                calib_frames = post_process_M
                static_frames_ii = post_process_S
            else:
                imCapture.save_stream = False
                if output_folder is not None and optiClient is not None:
                    time.sleep(1)
                    optiClient.stop()

                    # todo: copy opti file and post process
                    optiClient.copy_csv(output_folder)

                    optiClient.rb_init()

                    with np.printoptions(suppress=True, precision=3):
                        post_process(output_folder,
                                     r'F:\AMR\calib-dc\calib-20211114-134246\gHc_nelder-mead-from-scratch.pkl',
                                     # r'D:\AMR\gtool\ClientSample\src\gt_raw\ghc_opt2021-11-03-03-10-42.pkl',
                                     True,
                                     r'D:\AMR\gtool\ClientSample\src\gt_raw')

        elif k == 27:
            break

    imCapture.close()
    # imuCapture.close()
    cv2.destroyAllWindows()
    if optiClient is not None:
        optiClient.stop()
