import math
import operator
import copy
import logging
import numpy as np
import collections
import os
import sys
import configparser
import platform
tranformation_path= '../../../3rdparty/python'
if platform.system() == 'Windows':
    sys.path.append(os.path.join(os.path.dirname(__file__), tranformation_path))
elif platform.system() == 'Linux':
    sys.path.insert(0, tranformation_path)
import transformations as tm


class Format_File():

    def __init__(self, config=None, configFilePath=None):
        '''
        :param config: configParser of file in foramt as example_format.ini
        '''
        if config is None:
            config = configparser.ConfigParser()
            if configFilePath is None:
                return

        if configFilePath is not None:
            config.read(configFilePath)

        self.general_definition = {}
        if config.has_section('General'):
            for (each_key, each_val) in config.items('General'):
                if each_val != '':
                    self.general_definition[each_key] = each_val

        #detemines the type of every value in file(string, int etc. default is float)
        self.format_types = {}
        if config.has_section('Format Types'):
            for (each_key, each_val) in config.items('Format Types'):
                if each_val != '':
                    self.format_types[each_key] = each_val

                            # take all the format location in the config file.
        # assuming necessary items are in the config file (like timestamp,x,y,z,qi,qj,qk,qw)- we are not validating the config file!!!
        self.format_locations = {}
        if config.has_section('Format Locations'):
            for (each_key, each_val) in config.items('Format Locations'):
                if each_val != '':
                    try:
                        self.format_locations[each_key] = int(each_val)
                    except:
                        logging.error(each_val + ' is not int (using key ' + each_key + ')')

        #a list of conditions. a line that meets the condition will not be read
        self.exclude_conditions = {}
        if config.has_section('Exclude Condition'):
            for (each_key, each_val) in config.items('Exclude Condition'):
                if each_val != '':
                    self.exclude_conditions[each_key] = each_val.split(';')


        self.basic_format_verification()

    def basic_format_verification(self):
        '''
        test if there is something missing for use basic utils
        :return: None - log error with the problem/missing key
        '''


        # check for duplication values:
        value_occurrences = collections.Counter(self.format_locations.values())

        filtered_dict = {key: value for key, value in self.format_locations.items()
                         if value_occurrences[value] > 1}

        if len(filtered_dict)>0:
            logging.error("basic_format_verification: duplicates values in format_locations:"+str(filtered_dict))


        #check for dulplication keys:
        value_occurrences = collections.Counter(self.format_locations.values())

        filtered_dict = {key: value for key, value in self.format_locations.items()
                         if value_occurrences[value] > 1}

        if len(filtered_dict) > 0:
            logging.error("basic_format_verification: duplicates keys in format_locations:" + str(filtered_dict))

        if 'delimiter' not in self.general_definition:
            logging.warning('basic_format_verification: no delimeter has given.')

        if 'time_factor_to_ms' not in self.general_definition:
            logging.warning('basic_format_verification: no time_factor_to_ms has given.')

        if 'translation_factor_to_mm' not in self.general_definition:
            logging.warning('basic_format_verification: no translation_factor_to_mm has given.')

        if 'timestamp' not in self.format_locations:
            logging.warning('basic_format_verification: no timestamp in the given keys.')


        #set defaults
        if 'suffix' not in self.general_definition:
            self.general_definition['suffix'] = 'txt'


class Poses_Container():

    def __init__(self, file_name=None, format=None):
        '''

        :param file_name: the filename path of the poses
        :param format: the format object represent all data about the poses file.
        '''

        self.format = format
        self.file_name = file_name
        self.poses = []


    def get_poses (self):
        '''
        :return: the poses of the current container
        '''
        return self.poses

    def set_poses(self, poses, new_format = None):
        '''
        update the container poses to be the given poses and the given format.
        :param poses: list of poses - match to the format of the container or to the new given format
        :return: None
        '''
        if new_format is not None:
            self.format = new_format
        self.poses = poses



    def read_pose_file(self, list_of_utils_func = None):
        '''
        read the poses from the file_name
        :return: None
        '''
        fp = open(self.file_name)
        skip_lines_num = 0 if 'skip_lines' not in self.format.general_definition else int(self.format.general_definition['skip_lines'])
        skip_lines_num += 1 if 'contain_title' in self.format.general_definition and self.format.general_definition['contain_title']=='True' else 0

        delimiter = ' ' if 'delimiter' not in self.format.general_definition else self.format.general_definition['delimiter']

        for l in fp.readlines():
            data = l.split(delimiter)
            if len(data) < len(self.format.format_locations):
                continue
            if skip_lines_num > 0 :
                skip_lines_num -= 1
                continue
            pose = {}
            for key in self.format.format_locations.keys():
                if self.format.format_locations[key] is not None:
                    try:
                        pose[key] = eval(self.format.format_types[key])(data[self.format.format_locations[key]]) if key in self.format.format_types else float(data[self.format.format_locations[key]])
                    except:
                        logging.error('problem parsing the key:'+ str(key)+" "+ str(self.format.format_locations[key]))

            if list_of_utils_func is not None:
                Poses_Utils.apply_list_on_pose(pose, self.format, list_of_utils_func, warn=False)

            if Poses_Utils.pose_is_acceptable(pose, self.format):
                self.poses.append(pose)

        fp.close()

        #print appropriate warnings to functions applied on poses
        Poses_Utils.print_warnings_of_list(list_of_utils_func, self)

    def write_pose_file(self, output_file_name, format=None):
        '''
        save the poses to the given output path, using the given format
        :param output_file_name: full path to save the poses
        :param format_file: the format of saving
        :return:
        '''
        saving_format = self.format if format is None else format
        delimiter = ' ' if 'delimiter' not in saving_format.general_definition else saving_format.general_definition['delimiter']

        file = open(output_file_name, 'w')

        #titles line
        if 'contain_title' in saving_format.general_definition and saving_format.general_definition['contain_title']=='True':
            new_line = [''] * (max(saving_format.format_locations.values()) + 1)
            for k, v in saving_format.format_locations.items():
                new_line[v] = k
            file.write(delimiter.join(new_line) + '\n')

        #poses
        for pose in copy.deepcopy(self.poses):

            if 'time_factor_to_ms' in self.format.general_definition and 'time_factor_to_ms' in saving_format.general_definition:
                Poses_Utils.update_fw_timestamp_to_ms(pose, self.format, warn=False)
                Poses_Utils.update_fw_timestamp_from_ms(pose, saving_format, warn=False)
            if 'translation_factor_to_mm' in self.format.general_definition and 'translation_factor_to_mm' in saving_format.general_definition:
                Poses_Utils.update_translation_to_mm(pose, self.format, warn=False)
                Poses_Utils.update_translation_from_mm(pose, saving_format, warn=False)

            if (Poses_Utils.pose_is_acceptable(pose, self.format)):
                new_line = [''] * (max(saving_format.format_locations.values()) + 1)
                for key in pose.keys():
                    if pose[key] is not None and (key in saving_format.format_locations.keys()):
                        try:
                            new_line[saving_format.format_locations[key]] = '%0.16f' % (float(pose[key]))
                        except:
                            new_line[saving_format.format_locations[key]] = str(pose[key])
                file.write(delimiter.join(new_line) + '\n')

        file.close()


    ################################## Helpers ######################################


class Poses_Utils():

    '''
        appliers:
    '''

    @staticmethod
    def apply_list_on_poses(poses_container, list_of_func):
        for pose in poses_container.poses:
            Poses_Utils.apply_list_on_pose(pose,poses_container.format, list_of_func, warn=False)
        Poses_Utils.print_warnings_of_list(list_of_func, poses_container)

    @staticmethod
    def apply_list_on_pose(pose, format, list_of_func, warn=True):
        for func in list_of_func:
            if 'warn' in func.__code__.co_varnames:
                func(pose, format, warn)
            else:
                func(pose, format)

    '''
        for single pose:
    '''

    @staticmethod
    def pose_is_acceptable(pose, format):
        for key in pose.keys():
            if key in format.exclude_conditions:
                if pose[key] in [eval(self.format.format_types[key])(val) if key in format.format_types else float(val)
                                 for val in format.exclude_conditions[key]]:
                    return False
        return True

    @staticmethod
    def update_fw_timestamp_to_ms(pose, format, warn=True):
        if 'time_factor_to_ms' in format.general_definition and 'timestamp' in pose:
            pose['timestamp'] = float(pose['timestamp']) * float(format.general_definition['time_factor_to_ms'])
            if warn:
                logging.warning(Poses_Utils.post_function_warnings[sys._getframe().f_code.co_name])

    @staticmethod
    def update_fw_timestamp_from_ms(pose, format, warn=True):
        if 'time_factor_to_ms' in format.general_definition and 'timestamp' in pose:
            pose['timestamp'] = float(pose['timestamp']) / float(format.general_definition['time_factor_to_ms'])
            if warn:
                logging.warning(Poses_Utils.post_function_warnings[sys._getframe().f_code.co_name])

    @staticmethod
    def update_translation_to_mm(pose, format, warn=True):
        if 'translation_factor_to_mm' in format.general_definition:
            if 'x' in pose:
                pose['x'] = float(pose['x']) * float(format.general_definition['translation_factor_to_mm'])
            if 'y' in pose:
                pose['y'] = float(pose['y']) * float(format.general_definition['translation_factor_to_mm'])
            if 'z' in pose:
                pose['z'] = float(pose['z']) * float(format.general_definition['translation_factor_to_mm'])
            if warn:
                logging.warning(Poses_Utils.post_function_warnings[sys._getframe().f_code.co_name])

    @staticmethod
    def update_translation_from_mm(pose, format, warn=True):
        if 'translation_factor_to_mm' in format.general_definition:
            if 'x' in pose:
                pose['x'] = float(pose['x']) / float(format.general_definition['translation_factor_to_mm'])
            if 'y' in pose:
                pose['y'] =float( pose['y']) / float(format.general_definition['translation_factor_to_mm'])
            if 'z' in pose:
                pose['z'] = float(pose['z']) / float(format.general_definition['translation_factor_to_mm'])
            if warn:
                logging.warning(Poses_Utils.post_function_warnings[sys._getframe().f_code.co_name])

    @staticmethod
    def add_rotmat(pose, format = None):
        if 'rotmat' in pose:
            return
        rot_pose_list = ['r_{0}'.format(inx) for inx in range(0, 9)]
        if all([rpos in pose for rpos in rot_pose_list]):
            M = np.identity(4)
            M[:3, :3] = np.reshape(np.array([pose[x] for x in rot_pose_list]), (3, 3))
        elif 'qw' in pose and 'qx' in pose and 'qy' in pose and 'qz' in pose:
            M = tm.quaternion_matrix([float(i) for i in [pose['qw'], pose['qx'], pose['qy'], pose['qz']]])
        elif 'angle_x' in pose and 'angle_y' in pose and 'angle_z' in pose:
            M = tm.euler_matrix(pose['angle_x'], pose['angle_x'], pose['angle_x'])
        if M is not None and 'x' in pose and 'y' in pose and 'z' in pose:
            M[0:3, 3] = [float(i) for i in [pose['x'], pose['y'], pose['z']]]
            pose['rotmat'] = M

    @staticmethod
    def add_euler_angles(pose, format = None):
        Poses_Utils.add_rotmat(pose)
        if 'rotmat' in pose:
            ax, ay, az = tm.euler_from_matrix(pose['rotmat'])
            pose['angle_x'] = math.degrees(ax)
            pose['angle_y'] = math.degrees(ay)
            pose['angle_z'] = math.degrees(az)

    @staticmethod
    def add_quaternion(pose, format = None):
        if 'qw' in pose and 'qx' in pose and 'qy' in pose and 'qz' in pose:
            return
        rot_pose_list = ['r_{0}'.format(inx) for inx in range(0, 9)]
        if all([rpos in pose for rpos in rot_pose_list]):
            Poses_Utils.add_rotmat(pose)
        if 'rotmat' in pose:
            q = tm.quaternion_from_matrix(pose['rotmat'])
        elif 'angle_x' in pose and 'angle_y' in pose and 'angle_z' in pose:
            q = tm.quaternion_from_euler(pose['angle_x'], pose['angle_x'], pose['angle_x'])
        if q is not None:
            pose['qw'] = q[0]
            pose['qx'] = q[1]
            pose['qy'] = q[2]
            pose['qz'] = q[3]

    @staticmethod
    def add_magnitude(pose, format = None):
        Poses_Utils.add_rotmat(pose)
        if 'rotmat' in pose:
            if np.trace((pose['rotmat'])) > 4:
                pose['magnitude'] = 0
            else:
                pose['magnitude'] = float(math.acos((np.trace(pose['rotmat']) - 2) / 2))

    @staticmethod
    def add_rot_trace(pose, format = None):
        if 'rotmat' not in pose:
            Poses_Utils.add_rotmat(pose)
        if  'rotmat' in pose:
            pose['trace'] = pose['rotmat'][0][0]+pose['rotmat'][1][1]+ pose['rotmat'][2][2]





    @staticmethod
    def add_magnitude_from_quaternion(pose, format=None):
        if 'qw' in pose:
            pose['magnitude'] = 2 * math.acos(max(min(float(pose['qw']), 1), -1))


    '''
    for format changes:
    '''

    post_function_warnings = {
        'update_fw_timestamp_to_ms': "timestamp unit has been changed to milliseconds. make sure to fit other poses units and container format to new units!",
        'update_fw_timestamp_from_ms': "timestamp unit has been changed from milliseconds to original unit. make sure to fit other poses units and container format to new units!",
        'update_translation_to_mm': "translation values units have been changed to millimeters. make sure to fit other poses units and container format to new units!",
        'update_translation_from_mm': "timestamp unit has been changed from millimeters to original units. make sure to fit other poses units and container format to new units!"}

    @staticmethod
    def print_warnings_of_list(list_of_utils_func, poses_container):
        if list_of_utils_func is not None:
            for func in list_of_utils_func:
                if func.__code__.co_name in Poses_Utils.post_function_warnings:
                    logging.warning(Poses_Utils.post_function_warnings[
                                        func.__code__.co_name] + ' in container: ' + poses_container.file_name)

    @staticmethod
    def update_format(section_in_format, key, new_val):
        section_in_format[key] = new_val

    @staticmethod
    def update_formats(*list_of_update_requests):
        for update_request in list_of_update_requests:
            try:
                Poses_Utils.update_format(update_request['section'], update_request['key'], update_request['val'])
            except:
                logging.error('Failed tp apply update request')

    @staticmethod
    def apply_updates_on_formats(list_of_formats, list_of_key_val):
        list_of_update_requests = []
        for format_section in list_of_formats:
            for key in list_of_key_val:
                list_of_update_requests.append({'section': format_section, 'key': key, 'val': list_of_key_val[key]})
        Poses_Utils.update_formats(*list_of_update_requests)



