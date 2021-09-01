import argparse
import os
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), '..\\..\\Common\\python\\PosesReader'))
from PosesReader import Format_File, Poses_Container, Poses_Utils


def parse_arguments():
	'''
	parse arguments
	:return: arguments after parsing
	'''
	parser = argparse.ArgumentParser(description="interpolation")
	parser.add_argument("--in_file", type=str, help="a path to file to convert")
	parser.add_argument("--src_config", type=str, help="config file of the given file")
	parser.add_argument("--dst_config", type=str, help="config file of of the wanted format")
	parser.add_argument("--out_file", type=str, default=None, help="a path to the new converted file")

	args = parser.parse_args()

	if not os.path.isfile(args.in_file):
		print("Error: given file path is wrong.", args.in_file)
		exit(-1)
	if not os.path.isfile(args.src_config):
		print("Error: given source config file path is wrong.", args.src_config)
		exit(-1)
	if not os.path.isfile(args.dst_config):
		print("Error: given destination config file path is wrong.", args.dst_config)
		exit(-1)

	return args


if __name__ == "__main__":

	args = parse_arguments()

	container = Poses_Container(file_name=args.in_file, format=Format_File(configFilePath=args.src_config))
	container.read_pose_file([Poses_Utils.add_quaternion, Poses_Utils.add_rotmat, Poses_Utils.add_euler_angles])

	output_file = args.out_file if args.out_file else os.path.dirname(
		args.in_file) + '\\converted_' + os.path.basename(args.in_file)

	container.write_pose_file(output_file_name=output_file, format=Format_File(configFilePath=args.dst_config))
