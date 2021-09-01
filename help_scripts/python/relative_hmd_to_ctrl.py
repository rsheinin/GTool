import argparse
import os
import sys
import numpy as np
sys.path.append(os.path.join(os.path.dirname(__file__), '..\\..\\Common\\python\\PosesReader'))
from PosesReader import Format_File, Poses_Container, Poses_Utils
sys.path.append(os.path.join(os.path.dirname(__file__), '..\\3rdparty\\python'))
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


def relative_matrices(hmd_poses, ctrl_poses):

	relative_poses = []
	for hmd_pose, ctrl1_pose in zip(hmd_poses, ctrl_poses):
		if abs(hmd_pose['timestamp'] - ctrl1_pose['timestamp']) > sys.float_info.epsilon:
			raise Exception("hmd pose timestamp:", hmd_pose['timestamp'], "is not equal to controller 1 pose timestamp:",ctrl1_pose['timestamp'])

		qw, qx, qy, qz = tm.quaternion_multiply(tm.quaternion_inverse([hmd_pose['qw'], hmd_pose['qx'], hmd_pose['qy'], hmd_pose['qz']]), [ctrl1_pose['qw'], ctrl1_pose['qx'], ctrl1_pose['qy'], ctrl1_pose['qz']])

		mat = np.dot(np.linalg.inv(hmd_pose['rotmat']), ctrl1_pose['rotmat'])

		relative_poses.append({'timestamp': ctrl1_pose['timestamp'], 'qw': qw, 'qx': qx, 'qy': qy, 'qz': qz, 'confidence': ctrl1_pose['confidence'], 'sensor_id': ctrl1_pose['sensor_id'],
										 'x': mat[0, 3], 'y': mat[1, 3], 'z': mat[2, 3]})

	return relative_poses


def parse_arguments():
	'''
	parse arguments
	:return: arguments after parsing
	'''
	parser = argparse.ArgumentParser(description="interpolation")
	parser.add_argument("--hmd", type=str, default=None, help="a path to file with hmd poses, intrepolated to controller timestamps")
	parser.add_argument("--hmd_config", type=str, default=None, help="config file of the given hmd file")
	parser.add_argument("--ctrl", type=str, default=None, help="a path to file with controller poses")
	parser.add_argument("--ctrl_config", type=str, default=None, help="config file of the given controller file")
	parser.add_argument("--out_file", type=str, help="output file path")
	parser.add_argument("--out_config", type=str, default=None, help="config file for output poses")

	args = parser.parse_args()

	if not os.path.isfile(args.hmd):
		print("Error: given hmd file path is wrong.", args.hmd)
		exit(-1)
	if not os.path.isfile(args.hmd_config):
		print("Error: given hmd config file path is wrong.", args.hmd_config)
		exit(-1)
	if not os.path.isfile(args.ctrl):
		print("Error: given controller file path is wrong.", args.ctrl)
		exit(-1)
	if not os.path.isfile(args.ctrl_config):
		print("Error: given controller config file path is wrong.", args.ctrl_config)
		exit(-1)
	if args.out_file and not os.path.isdir(os.path.dirname(args.out_file)):
		print("Error: given out file path is wrong.", args.out_file)
		exit(-1)
	if args.out_config and not os.path.isfile(args.out_config):
		print("Error: given out config file path is wrong.", args.out_config)
		exit(-1)

	return args


if __name__ == "__main__":

	args = parse_arguments()

	output_config = args.out_config if args.out_config else args.ctrl_config

	hmd_container = Poses_Container(file_name=args.hmd, format=Format_File(configFilePath=args.hmd_config))
	hmd_container.read_pose_file([Poses_Utils.update_fw_timestamp_to_ms, Poses_Utils.update_translation_to_mm, Poses_Utils.add_rotmat])
	if not is_sorted(hmd_container.poses, key=lambda x: x['timestamp']):
		hmd_container.poses = sorted(hmd_container.poses, key=lambda k: k['timestamp'])

	ctrl_container = Poses_Container(file_name=args.ctrl, format=Format_File(configFilePath=args.ctrl_config))
	ctrl_container.read_pose_file([Poses_Utils.update_fw_timestamp_to_ms, Poses_Utils.update_translation_to_mm, Poses_Utils.add_rotmat])
	if not is_sorted(ctrl_container.poses, key=lambda x: x['timestamp']):
		ctrl_container.poses = sorted(ctrl_container.poses, key=lambda k: k['timestamp'])

	#update containers formats accordingly to changes
	Poses_Utils.apply_updates_on_formats([hmd_container.format.general_definition, ctrl_container.format.general_definition], {'time_factor_to_ms': 1, 'translation_factor_to_mm': 1})

	if len(hmd_container.poses) != len(ctrl_container.poses):
		raise Exception("hmd poses number:", len(hmd_container.poses), "is not equal to controller poses number:", len(ctrl_container.poses))

	relative_container = Poses_Container()
	relative_container.format = ctrl_container.format
	relative_container.poses = relative_matrices(hmd_container.poses, ctrl_container.poses)

	output_file = args.out_file if args.out_file else os.path.dirname(
		args.ctrl) + '\\relative_hmd_to_ctrl.' + relative_container.format.general_definition['suffix']

	relative_container.write_pose_file(output_file_name=output_file, format=Format_File(configFilePath=output_config))





