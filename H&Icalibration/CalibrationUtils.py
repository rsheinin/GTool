import math
import operator
import logging
import numpy as np
import os
import sys
import matplotlib.pyplot as plt
from matplotlib.widgets import CheckButtons
from DataHolder import DataHolder
from CalibrationConstants import CalibrationConstants
sys.path.append(os.path.join(os.path.dirname(__file__), '..\\..\\..\\3rdparty\\python'))
import transformations as tm


def is_sorted(_list, key=lambda x: x):
    '''
    check if given list is sorted according to specified key
    :param _list: given list
    :param key: key of which the list is sorted or not by
    :return: True if given list is sorted, False otherwise
    '''
    for i in range(len(_list) - 1):
        if key(_list[i]) > key(_list[i + 1]):
            return False
    return True
    #return all([key(_list[i]) <= key(_list[i + 1]) for i in range(len(_list) - 1)])


def make_dir(dir_name):
    if not os.path.isfile(dir_name) and not os.path.exists(dir_name):
        os.makedirs(dir_name)


class FilterFormat:

    def __init__(self, holders, func, params=None):

        self.holders = holders
        self.func = func
        self.params = params


class FilterDefinition:

    def __init__(self, holder, value, threshold, operand):

        self.holder = holder
        self.value = value
        self.threshold = threshold
        self.operand = operand


class Filter:
    '''
    static class for filters involved in calibration process
    '''

    @staticmethod
    def apply_filters(list_of_filters):
        for filter in list_of_filters:
            #print (filter.func)
            try:
                Filter.filter_holder(filter.func(filter.holders)) if filter.params is None else Filter.filter_holder(filter.func(filter.holders, filter.params))
            except:
                print('Failed to filter data holder', filter.holders, 'with filter', filter.func)

    @staticmethod
    def filter_holder(list_of_filter_definitions):
        for filter_definition in list_of_filter_definitions:
            holder = filter_definition.holder
            val = filter_definition.value
            thresh = filter_definition.threshold
            operand = filter_definition.operand
            print(holder.name, "(", holder.type.name, ")", "before", val, "filter:", Filter.count_positive_poses(holder.mask), "positive poses")
            for i, pose in enumerate(holder.container.poses):
                try:
                    holder.mask[i] &= Filter.filter_pose(pose[val], thresh, operand)
                except:
                    holder.mask[i] = False
                    print('Failed to filter data holder', holder.name, 'with value', val, 'in pose number', i, '. set mask to False')
            print(holder.name, "(", holder.type.name, ")", "after", val, "filter:", Filter.count_positive_poses(holder.mask), "positive poses")

    @staticmethod
    def filter_pose(val, thresh, operand):
        return operand(val, thresh)

    @staticmethod
    def filter_by_known_distance(gt_poses, src_poses):
        #TODO: implement
        pass

    @staticmethod
    def filter_by_initial_phase(list_of_holders):
        filter_definitions = []
        for data_holder in list_of_holders:
            first_timestamp = float(data_holder.container.poses[0]['timestamp'])
            for i in range(len(data_holder.container.poses)):
                data_holder.container.poses[i]['rel_timestamp'] = \
                    float(data_holder.container.format.general_definition['time_factor_to_ms']) * \
                    (float(data_holder.container.poses[i]['timestamp']) - first_timestamp)
            filter_definitions.append(FilterDefinition(holder=data_holder, value='rel_timestamp', operand=operator.gt, threshold=data_holder.initial_phase_ms))
        return filter_definitions

    @staticmethod
    def filter_by_magnitude(list_of_data_holders):
        filter_definitions = []
        for i in range(len(list_of_data_holders)):
            filter_definitions.append(FilterDefinition(holder=list_of_data_holders[i], value='magnitude', operand=operator.gt,
                                         threshold=CalibrationConstants.MIN_ANGLE_MAGNITUDE))
        if len(list_of_data_holders) == 2:
            for i, pose in enumerate(list_of_data_holders[0].container.poses):
                pose['magnitude_diff'] = math.fabs(pose['magnitude'] - list_of_data_holders[1].container.poses[i]['magnitude'])
            filter_definitions.append(FilterDefinition(holder=list_of_data_holders[0], value='magnitude_diff', operand=operator.lt, threshold=CalibrationConstants.MAX_ANGLE_MAGNITUDE_DIFF))
        return filter_definitions

    @staticmethod
    def filter_by_irregular_interval(list_of_holders, params):
        filter_definitions = []
        for data_holder in list_of_holders:
            for pose in data_holder.container.poses:
                pose['interval_ms'] = float(pose["timestamp1"]) - float(pose["timestamp0"])
            filter_definitions.append(FilterDefinition(holder=data_holder, value='interval_ms', operand=operator.lt, threshold=params["max_interval"]))
        return filter_definitions

    @staticmethod
    def filter_by_confidence(list_of_holders):
        filter_definitions = []
        for data_holder in list_of_holders:
            for i in range(len(data_holder.container.poses)):
                data_holder.container.poses[i]['confidence'] = float(data_holder.container.poses[i]['confidence'])
            filter_definitions.append(FilterDefinition(holder=data_holder, value='confidence', operand=operator.ge, threshold=data_holder.min_confidence))
        return filter_definitions

    @staticmethod
    def filter_by_error(list_of_holders):
        filter_definitions = []
        for data_holder in list_of_holders:
            filter_definitions.append(FilterDefinition(holder=data_holder, value='error', operand=operator.le, threshold=data_holder.max_error))
        return filter_definitions

    @staticmethod
    def filter_by_status(list_of_holders):
        filter_definitions = []
        for data_holder in list_of_holders:
            if 'tracked' in data_holder.container.format.format_locations:
                filter_definitions.append(FilterDefinition(holder=data_holder, value='tracked', operand=operator.eq,
                                             threshold=1))
        return filter_definitions

    @staticmethod
    def filter_by_validity(list_of_holders):
        filter_definitions = []
        for data_holder in list_of_holders:
            for pose in data_holder.container.poses:
                pose['valid'] = data_holder.valid_pose(pose)
            filter_definitions.append(FilterDefinition(holder=data_holder, value='valid', operand=operator.eq,
                                                           threshold=True))
        return filter_definitions

    @staticmethod
    def filter_by_RPE(list_of_holders, params):
        RPE_avg = params["RPE_avg"]
        sorted_distances_from_RPE_avg = sorted([math.fabs(pose['dist'] - RPE_avg) for pose in list_of_holders[0].container.poses])
        threshold = sorted_distances_from_RPE_avg[int(len(sorted_distances_from_RPE_avg) * CalibrationConstants.max_dist_from_RPE_avg)]
        for pose in list_of_holders[0].container.poses:
            pose['distance_from_RPE_avg'] = math.fabs(pose['dist'] - RPE_avg)
        return [FilterDefinition(holder=list_of_holders[0], value='distance_from_RPE_avg', operand=operator.le, threshold=threshold)]

    @staticmethod
    def intersect_masks(list_of_masks):
        '''
            intersect all masks in list
            :param list_of_masks: list of masks to intersect
            :return: intersected mask
        '''
        #TODO:verify equal sizes of all masks
        length = len(list_of_masks[0])
        if any(len(lst) != length for lst in list_of_masks):
            logging.error("Masks in different sizes")
        intersected_mask = [True] * length
        for i in range(len(list_of_masks[0])):
            for mask in list_of_masks:
                intersected_mask[i] &= mask[i]
        return intersected_mask

    @staticmethod
    def count_positive_poses(mask):
        return mask.count(True)


class RelativePoses:

    def __init__(self, gt_data, src_data, interval_ms):

        self.interval_in_ms = interval_ms

        intersected_mask = Filter.intersect_masks([gt_data.mask, src_data.mask])

        self.rel_gt = RelativePoses.relative_poses(gt_data, DataHolder.ms_to_frames_number(gt_data, self.interval_in_ms), intersected_mask)
        self.rel_src = RelativePoses.relative_poses(src_data, DataHolder.ms_to_frames_number(src_data, self.interval_in_ms), intersected_mask)

        if not DataHolder.compare_sizes(self.rel_gt, self.rel_src):
            raise Exception("Failed to create relative matrices due to different number in source and GT")

        Filter.apply_filters([FilterFormat(holders=[self.rel_gt, self.rel_src], func=Filter.filter_by_irregular_interval, params={"max_interval": CalibrationConstants.max_interval * self.interval_in_ms})])
        DataHolder.apply_list_on_holders([self.rel_gt, self.rel_src], [DataHolder.add_magnitude])
        Filter.apply_filters([FilterFormat(holders=[self.rel_gt, self.rel_src], func=Filter.filter_by_magnitude)])

    @staticmethod
    def relative_poses(data_holder, interval, mask=None):
        '''
            create relative poses from given poses in a fixed window sixe
            :param data_holder: holder to apply relative poses on its poses
            :param interval: window size in frames number
            :param mask: mask to specify which poses to take for creating the relative poses
            :return: data holder with relative poses
        '''

        relative_holder = data_holder.__class__()
        relative_holder.name = "rel_" + data_holder.name
        relative_holder.type = data_holder.type

        poses = data_holder.container.poses

        if mask is not None and len(mask) != len(poses):
            mask = None

        for i in range(len(data_holder.container.poses) - interval):

            if mask is None or (mask[i] and mask[i + interval]):
                rel_quaternion = tm.quaternion_multiply(tm.quaternion_inverse([float(j) for j in
                                                                               [poses[i]['qw'], poses[i]['qx'],
                                                                                poses[i]['qy'], poses[i]['qz']]]),
                                                        [float(j) for j in
                                                         [poses[i + interval]['qw'], poses[i + interval]['qx'],
                                                          poses[i + interval]['qy'], poses[i + interval]['qz']]])

                rel_mat = np.dot(np.linalg.inv(poses[i]['rotmat']), poses[i + interval]['rotmat'])

                pose = {}
                pose["timestamp0"] = poses[i]['timestamp']
                pose["timestamp1"] = poses[i + interval]['timestamp']
                pose['qw'] = rel_quaternion[0]
                pose['qx'] = rel_quaternion[1]
                pose['qy'] = rel_quaternion[2]
                pose['qz'] = rel_quaternion[3]
                pose['x'] = rel_mat[0, 3]
                pose['y'] = rel_mat[1, 3]
                pose['z'] = rel_mat[2, 3]
                pose['rotmat'] = rel_mat

                relative_holder.container.poses.append(pose)
        relative_holder.container.file_name = data_holder.container.file_name
        relative_holder.mask = [True] * len(relative_holder.container.poses)

        return relative_holder


class Plot:

    def __init__(self, gt_holder, src_holder):
        self.gt_holder = gt_holder
        self.src_holder = src_holder

    def plot_calibration(self):
        '''
            :return: Plot with drawn relative poses of both GT and source data
        '''

        fig, subplots = plt.subplots(2)
        trans = Plot.plot_poses_translation(subplots[0], [self.gt_holder, self.src_holder])
        rot = Plot.plot_poses_rotation(subplots[1], [self.gt_holder, self.src_holder])

        trans['check_button'].on_clicked(lambda x: Plot.control_line(x, trans['labels'], trans['lines']))
        rot['check_button'].on_clicked(lambda x: Plot.control_line(x, rot['labels'], rot['lines']))

        plt.show()

    @staticmethod
    def control_line(label, labels, lines):
        index = labels.index(label)
        lines[index].set_visible(not lines[index].get_visible())
        plt.draw()

    @staticmethod
    def plot_poses_translation(subplot, list_of_holders):
        '''
            :param subplot: subplot of translation part
            :param list_of_holders: all holders to add their translation data to plot
            :return: needed specification for adding translation data to plot
        '''

        subplot.set_title('Translation')

        lines = []

        for holder in list_of_holders:
            x = [float(pose['x']) for pose in holder.container.poses]
            y = [float(pose['y']) for pose in holder.container.poses]
            z = [float(pose['z']) for pose in holder.container.poses]

            l_x, = subplot.plot(x, visible=True, lw=1, label='x_' + holder.name)
            l_y, = subplot.plot(y, visible=True, lw=1, label='y_' + holder.name)
            l_z, = subplot.plot(z, visible=True, lw=1, label='z_' + holder.name)

            lines.append(l_x)
            lines.append(l_y)
            lines.append(l_z)

        # Make checkbuttons with all plotted lines with correct visibility
        rax = plt.axes([0.05, 0.62, 0.15, 0.18])
        labels = [str(line.get_label()) for line in lines]
        visibility = [line.get_visible() for line in lines]
        check_button = CheckButtons(rax, labels, visibility)


        return {'check_button': check_button, 'labels': labels, 'lines': lines}


    @staticmethod
    def plot_poses_rotation(subplot, list_of_holders):
        '''
            :param subplot: subplot of rotation part
            :param list_of_holders: all holders to add their rotation data to plot
            :return: needed specification for adding rotation data to plot
        '''

        subplot.set_title('Euler')

        lines = []

        for holder in list_of_holders:

            DataHolder.apply_list_on_holders([holder], [DataHolder.add_euler_angles])

            yaw = [float(pose['angle_x']) for pose in holder.container.poses]
            pitch = [float(pose['angle_y']) for pose in holder.container.poses]
            roll = [float(pose['angle_z']) for pose in holder.container.poses]

            l_yaw, = subplot.plot(yaw, visible=True, lw=1, label='yaw_' + holder.name)
            l_pitch, = subplot.plot(pitch, visible=True, lw=1, label='pitch_' + holder.name)
            l_roll, = subplot.plot(roll, visible=True, lw=1, label='roll_' + holder.name)

            lines.append(l_yaw)
            lines.append(l_pitch)
            lines.append(l_roll)

        plt.subplots_adjust(left=0.29, right=0.95)

        # Make checkbuttons with all plotted lines with correct visibility
        rax = plt.axes([0.05, 0.22, 0.15, 0.18])
        labels = [str(line.get_label()) for line in lines]
        visibility = [line.get_visible() for line in lines]
        check_button = CheckButtons(rax, labels, visibility)

        return {'check_button': check_button, 'labels': labels, 'lines': lines}


