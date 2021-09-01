import argparse
import os
import sys
from bisect import bisect_left
sys.path.append(os.path.join(os.path.dirname(__file__), '..\\..\\Common\\python\\PosesReader'))
from PosesReader import Format_File, Poses_Container, Poses_Utils
sys.path.append(os.path.join(os.path.dirname(__file__), '..\\3rdparty\\python'))
import transformations as tm

#TODO: move to utils file
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


def interpolation(poses_container, timestamps, method, max_interval, sort=False):
	'''
	interpolate poses of poses_container according to given timestamps list
	:param poses_container: container of poses to interpolate accordingly
	:param timestamps: list of wanted timestamps
	:param method: the method of interpolation (linear, nearest etc.)
	:param sort: boolean flag to indicate whether to create the interpolated poses in a sorted order or keep the original timestamps list order
	:return: new container with interpolated poses
	'''

	interpolation_method = eval(method + '_interpolation')
	interpolated_container = Poses_Container()

	#sort the poses according to their timestamps if not sorted
	sorted_poses = sorted(poses_container.poses, key=lambda k: k['timestamp']) if not is_sorted(poses_container.poses, key=lambda x: x['timestamp']) else poses_container.poses
	poses_ts = [pose['timestamp'] for pose in sorted_poses]

	if sort and not is_sorted(timestamps):
		timestamps = sorted(timestamps)

	index = 0
	for ts in timestamps:
		index = get_previous_pose_index(poses_ts, ts, initial_index=index if sort else 0)
		if 0 <= index < len(sorted_poses):
			if ts - sorted_poses[index]['timestamp'] > max_interval or sorted_poses[index + 1]['timestamp'] - ts > max_interval:
				interpolated_pose = {'timestamp': ts, 'x': 'nan',  'y': 'nan',  'z': 'nan',  'qx': 'nan',  'qy': 'nan',  'qz': 'nan',  'qw': 'nan', 'tracked': 0}
			else:
				try:
					interpolated_pose = interpolation_method(sorted_poses[index], sorted_poses[index + 1], ts)
				except:
					raise Exception("Failed to run", interpolation_method, "function")
			interpolated_container.poses.append(interpolated_pose)

	Poses_Utils.apply_list_on_poses(interpolated_container, [Poses_Utils.add_rotmat])
	return interpolated_container


def get_previous_pose_index(poses_ts, ts, initial_index=0):
	'''
	find the index of the pose with the closest (before) timestamp to the given timestamp
	:param poses_ts: list of poses timestamps
	:param ts: wanted timestamp
	:param initial_index: initial index for searching in poses timestamps
	:return: index of the pose with the closest (before) timestamp to the given timestamp
	'''

	pos = bisect_left(poses_ts, ts, lo=max(initial_index, 0))
	if pos == len(poses_ts) and (poses_ts[len(poses_ts) - 1] - ts) > sys.float_info.epsilon: #last pose ts is smaller that wanted ts
		return -1
	if pos == 0 and abs(poses_ts[0] - ts) < sys.float_info.epsilon: #first pose timestamp is equal to wanted timestamp
		return 0
	return pos - 1


def linear_interpolation(pose_before, pose_after, ts):
	'''
	interpolate 2 poses in linear method: slerp algorithm for rotation part and linear interpolation for translation part
	:param pose_before: the closest pose before the wanted timestamp
	:param pose_after: the closest pose after the wanted timestamp
	:param ts: the wanted timestamp to interpolate to
	:return: new pose with wanted timestamp and interpolation data
	'''

	fraction = (ts - pose_before['timestamp']) / (pose_after['timestamp'] - pose_before['timestamp'])

	pose = {'timestamp': ts, 'tracked': 1}

	#interpolate rotation only if quaternion data exist
	if all(key in pose_before and key in pose_after for key in ['qw', 'qx', 'qy', 'qz']):
		q0 = [pose_before['qw'], pose_before['qx'], pose_before['qy'], pose_before['qz']]
		q1 = [pose_after['qw'], pose_after['qx'], pose_after['qy'], pose_after['qz']]
		if fraction == 1 or fraction == 0: #wanted timestamp is equal to pose_after timestamp or to pose_before timestamp
			interpolated_quaternion = [naive_interpolation(val1, val2, fraction) for val1, val2 in zip(q0, q1)]
		else:
			interpolated_quaternion = tm.quaternion_slerp(q0, q1, fraction)
		pose.update({'qw': interpolated_quaternion[0], 'qx': interpolated_quaternion[1], 'qy': interpolated_quaternion[2], 'qz': interpolated_quaternion[3]})

	#interpolate translation only if translation data exist
	if all(key in pose_before and key in pose_after for key in ['x', 'y', 'z']):
		interpolated_translation = [naive_interpolation(adjacent_vals[0], adjacent_vals[1], fraction) for adjacent_vals in [[pose_before['x'], pose_after['x']], [pose_before['y'], pose_after['y']], [pose_before['z'], pose_after['z']]]]
		pose.update({'x': interpolated_translation[0], 'y': interpolated_translation[1], 'z': interpolated_translation[2]})

	#basic linear interpolation for other values
	for key in set(key for key in pose_before.keys() if key in pose_after.keys()):
		if key not in pose:
			pose[key] = naive_interpolation(pose_before[key], pose_after[key], fraction)

	return pose

def naive_interpolation(val1, val2, fraction):
	return val1 + (val2 - val1) * fraction


def parse_arguments():
	'''
	parse arguments
	:return: arguments after parsing
	'''
	parser = argparse.ArgumentParser(description="interpolation")
	parser.add_argument("--poses_file", type=str, default=None, help="a path to file with input poses")
	parser.add_argument("--poses_config", type=str, default=None, help="config file of the given poses file")
	parser.add_argument("--ts_file", type=str, default=None, help="a path to file with list of timestamps to interpolate accordingly")
	parser.add_argument("--ts_config", type=str, default=None, help="config file of the ts file")
	parser.add_argument("--out_file", type=str, help="output file path")
	parser.add_argument("--out_config", type=str, default=None, help="config file for output poses")
	parser.add_argument("--method", type=str, default='linear', help="method of interpolation. current options: 'linear'")
	parser.add_argument("--interval", type=int, default=15, help="maximum interval (in milliseconds) between pose time and the desired time enabled for interpolation")
	parser.add_argument("--sort", action='store_true', default=False,
						help="boolean flag to indicate whether to create the interpolated poses in a sorted order of timestamps (True) or keep the original timestamps list order (False)")

	args = parser.parse_args()

	if not os.path.isfile(args.poses_file):
		print("Error: given poses_file path is wrong.", args.poses_file)
		exit(-1)
	if not os.path.isfile(args.poses_config):
		print("Error: given poses_config path is wrong.", args.poses_config)
		exit(-1)
	if not os.path.isfile(args.ts_file):
		print("Error: given ts_file path is wrong.", args.ts_file)
		exit(-1)
	if not os.path.isfile(args.ts_config):
		print("Error: given ts_config path is wrong.", args.ts_config)
		exit(-1)
	if args.out_file and not os.path.isdir(os.path.dirname(args.out_file)):
		print("Error: given out_file path is wrong.", args.out_file)
		exit(-1)
	if args.out_config and not os.path.isfile(args.out_config):
		print("Error: given out_config path is wrong.", args.out_config)
		exit(-1)


	return args


if __name__ == "__main__":

    args = parse_arguments()

    output_config = args.out_config if args.out_config else args.poses_config

    poses_container = Poses_Container(file_name=args.poses_file, format=Format_File(configFilePath=args.poses_config))
    poses_container.read_pose_file([Poses_Utils.update_fw_timestamp_to_ms, Poses_Utils.update_translation_to_mm])

    ts_list = Poses_Container(file_name=args.ts_file, format=Format_File(configFilePath=args.ts_config))
    ts_list.read_pose_file([Poses_Utils.update_fw_timestamp_to_ms])

    # update containers formats accordingly to changes
    Poses_Utils.apply_updates_on_formats([poses_container.format.general_definition, ts_list.format.general_definition], {'time_factor_to_ms': 1})
    Poses_Utils.apply_updates_on_formats([poses_container.format.general_definition], {'translation_factor_to_mm': 1})

    interpolation_method = args.method if args.method else 'linear'
    interpolated_container = interpolation(poses_container, [pose['timestamp'] for pose in ts_list.poses], interpolation_method, args.interval, args.sort)
    interpolated_container.format = poses_container.format

    output_file = args.out_file if args.out_file else os.path.dirname(
    	args.poses_file) + '\\interpolated_' + os.path.basename(args.ts_file) + '.' + interpolated_container.format.general_definition['suffix']

    interpolated_container.write_pose_file(output_file_name=output_file, format=Format_File(configFilePath=output_config))





