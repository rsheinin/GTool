import sys
import math
import os
import glob
from enum import Enum
import numpy as np
sys.path.append(os.path.join(os.path.dirname(__file__), '..\\Common\\python\\PosesReader'))
from PosesReader import Format_File, Poses_Container, Poses_Utils
sys.path.append(os.path.join(os.path.dirname(__file__), '..\\3rdparty\\python\\transformations'))
import transformations as tm


class Type(Enum):
    GT = 1
    SRC = 2


class DataHolder:

    def __init__(self, file_path=None, config_name=None, config_file_path=None, type=None):

        if type is not None:
            self.type = type

        if config_name is not None and config_file_path is None:
            config_file_path = glob.glob('..\\Common\\python\\PosesReader\\KnownFormats\\*' + config_name + '*.ini')[0]
            if not os.path.isfile(config_file_path):
                print("Error: config_file_path is wrong.", config_file_path)
                exit(-1)

        # load poses file
        self.container = Poses_Container(file_path, format=Format_File(configFilePath=config_file_path))

        if self.container.file_name is not None:
            self.container.read_pose_file()

        self.mask = [True] * len(self.container.poses)
        self.initial_phase_ms = 0

    def add_sixdof_poses(self):
        '''
        compute 6dof pose from frame data
        '''
        raise NotImplementedError("Data Object must implement add_sixdof_poses func")

    def valid_pose(self, pose):
        '''
        determines whether a given pose is a valid pose or not
        :param pose: a pose to check
        :return: True if pose is valid, False if not
        '''
        for key in pose:
            if isinstance(pose[key], float) and math.isnan(pose[key]) or isinstance(pose[key], str) and pose[key] == 'nan':
                return False
        return True

    @staticmethod
    def apply_list_on_holders(list_of_data_holders, list_of_func):
        for func in list_of_func:
            DataHolder.apply_func_on_holders(func, list_of_data_holders)

    @staticmethod
    def apply_list_on_holder(data_holder, list_of_func):
        for func in list_of_func:
            func(data_holder)

    @staticmethod
    def apply_func_on_holders(func, list_of_data_holders):
        for data_holder in list_of_data_holders:
            func(data_holder)

    @staticmethod
    def remove_duplicates(data_holder):
        org_len = len(data_holder.container.poses)
        last_pose = data_holder.container.poses[len(data_holder.container.poses) - 1]
        data_holder.container.poses = [pose for (i, pose) in enumerate(data_holder.container.poses[:-1]) if
                                       float(data_holder.container.poses[i + 1]['timestamp']) - float(
                                           pose['timestamp']) > sys.float_info.epsilon]
        data_holder.container.poses.append(last_pose)
        data_holder.mask = [True] * len(data_holder.container.poses)
        print ("{} duplicate poses were removed from {} holder".format(org_len - len(data_holder.container.poses),
                                                                       data_holder.name))

    @staticmethod
    def remove_unique_poses_by_key(data_holders_list, key='timestamp', precision='%0.4f'):
        '''
        remove all poses which their value according to a given key does not exist in all containers
        :param data_holders_list: a list of poses containers to intersect
        :param key: key of the value need to be shared
        :param precision: precision for comparing values
        :return: new containers after removing unique poses
        '''

        org_lengths = {data_holder.name: len(data_holder.container.poses) for data_holder in data_holders_list}

        # create a set of shared poses according to a given key
        unique_poses = [set(precision % pose[key] for pose in data_holder.container.poses) for data_holder in
                        data_holders_list]
        intersected_poses = unique_poses[0]
        for s in unique_poses[1:]:
            intersected_poses.intersection_update(s)

        for data_holder in data_holders_list:
            for i, pose in enumerate(data_holder.container.poses):
                if precision % pose[key] not in intersected_poses:
                    print('found unique pose! holder:', data_holder.name, 'index:', i, 'key:', pose[key])
            data_holder.container.poses = [pose for pose in data_holder.container.poses if
                                           (precision % pose[key] in intersected_poses)]
            print(
            'holder:', data_holder.name, 'before remove unique poses:', org_lengths[data_holder.name], 'poses. after:',
            len(data_holder.container.poses))
            data_holder.mask = [True] * len(data_holder.container.poses)

    @staticmethod
    def time_to_ms(data_holder):
        Poses_Utils.apply_list_on_poses(data_holder.container, [Poses_Utils.update_fw_timestamp_to_ms])
        print("timestamps of", data_holder.name, "poses have been converted to milliseconds")

    @staticmethod
    def translation_to_mm(data_holder):
        Poses_Utils.apply_list_on_poses(data_holder.container, [Poses_Utils.update_translation_to_mm])
        print("translation values of", data_holder.name, "poses have been converted to millimeters")

    @staticmethod
    def add_magnitude(data_holder):
        Poses_Utils.apply_list_on_poses(data_holder.container, [Poses_Utils.add_magnitude_from_quaternion])
        print("added magnitude based quaternion to", data_holder.name, "poses")

    @staticmethod
    def add_rotation_matrix(data_holder):
        Poses_Utils.apply_list_on_poses(data_holder.container, [Poses_Utils.add_rotmat])
        print("added rotation matrix to", data_holder.name, "poses")

    @staticmethod
    def add_euler_angles(data_holder):
        Poses_Utils.apply_list_on_poses(data_holder.container, [Poses_Utils.add_euler_angles])
        print("added Euler angles to", data_holder.name, "poses")

    @staticmethod
    def ms_to_frames_number(data_holder, value_in_ms):
        poses = data_holder.container.poses
        timestamp_list = []
        for pose in poses:
            timestamp_list.append(float(pose['timestamp']))
        return int(value_in_ms / (np.mean(np.diff(timestamp_list))))

    @staticmethod
    def compare_sizes(holder1, holder2):
        length = len(holder1.container.poses)
        if any(len(lst) != length for lst in [holder1.container.poses, holder2.container.poses]):
            print("Unique {} poses number ({}) is not equal to unique {} poses number ({})".format(
                holder1.name, len(holder1.container.poses), holder2.name, len(holder2.container.poses)))
            return False
        print("{} poses number and {} poses number are equal!".format(holder1.name, holder2.name))
        return True


class Gyro(DataHolder):
    def __init__(self, file_path=None, config_name=None, config_file_path=None, type=None):
        self.type = Type.SRC
        super(Gyro,self).__init__(file_path, config_name=config_name, config_file_path=config_file_path,type=type)
        self.initial_phase_ms = 0
        self.name = "gyro"

    def add_sixdof_poses(self):
        '''
        compute quaternion from velocity data
        '''

        self.container.poses[0]['qw'] = 1
        self.container.poses[0]['qx'] = 0
        self.container.poses[0]['qy'] = 0
        self.container.poses[0]['qz'] = 0

        for i in range(1, len(self.container.poses)):
            '''
            TODO: move scale and bias factors calculations to readPoses function
            '''
            velocity_x = float(self.container.poses[i]['x']) * float(self.container.format.general_definition['scale_x']) + float(
                self.container.format.general_definition['bias_x'])
            velocity_y = float(self.container.poses[i]['y']) * float(self.container.format.general_definition['scale_y']) + float(
                self.container.format.general_definition['bias_y'])
            velocity_z = float(self.container.poses[i]['z']) * float(self.container.format.general_definition['scale_z']) + float(
                self.container.format.general_definition['bias_z'])

            time_diff_s = 0.001 * ((float(self.container.poses[i]['timestamp']) - float(self.container.poses[i - 1]['timestamp'])) * float(
                self.container.format.general_definition['time_factor_to_ms']))

            angle_delta_euler = [V * time_diff_s for V in [velocity_x, velocity_y, velocity_z]]
            angle_delta_quaternion = tm.quaternion_from_euler(angle_delta_euler[0], angle_delta_euler[1], angle_delta_euler[2])
            ref_pose = tm.quaternion_multiply([self.container.poses[i - 1]['qw'], self.container.poses[i - 1]['qx'], self.container.poses[i - 1]['qy'], self.container.poses[i - 1]['qz']], angle_delta_quaternion)

            self.container.poses[i]['qw'] = ref_pose[0]
            self.container.poses[i]['qx'] = ref_pose[1]
            self.container.poses[i]['qy'] = ref_pose[2]
            self.container.poses[i]['qz'] = ref_pose[3]

            self.container.poses[i]['tx'] = 0
            self.container.poses[i]['ty'] = 0
            self.container.poses[i]['tz'] = 0


class Sixdof(DataHolder):
    def __init__(self, file_path=None, config_name=None, config_file_path=None, type=None):
        self.type = Type.SRC
        super(Sixdof, self).__init__(file_path, config_name=config_name, config_file_path=config_file_path,type=type)
        self.initial_phase_ms = 1000
        self.name = "6dof"
        self.min_confidence = 3

    def valid_pose(self, pose):
        '''
        determines whether a given pose is a valid pose or not
        :param pose: a pose to check
        :return: True if pose is valid, False if not
        '''
        if 'confidence' in pose and isinstance(pose['confidence'], float) and pose['confidence'] < self.min_confidence:
            return False
        return super(Sixdof, self).valid_pose(pose)

    def add_sixdof_poses(self):
        pass


class OT(DataHolder):
    def __init__(self, file_path=None, config_name=None, config_file_path=None, type=None):
        self.type = Type.GT
        super(OT, self).__init__(file_path, config_name=config_name, config_file_path=config_file_path,type=type)
        self.initial_phase_ms = 0
        self.name = "optitrack"
        self.max_error = 0.0008#0.0003

    def valid_pose(self, pose):
        '''
        determines whether a given pose is a valid pose or not
        :param pose: a pose to check
        :return: True if pose is valid, False if not
        '''
        if 'error' in pose and isinstance(pose['error'], float) and pose['error'] > self.max_error:
            return False
        if 'tracked' in pose and isinstance(pose['tracked'], float) and pose['tracked'] != 1:
            return False
        return super(OT, self).valid_pose(pose)

    def add_sixdof_poses(self):
        pass



