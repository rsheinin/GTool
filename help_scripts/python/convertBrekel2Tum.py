#!/usr/bin/env python
import numpy as np
import os
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), '..\\..\\3rdparty\\python'))
import transformations as tm
import time

src_poses = []
src_hmd_poses = []
src_controller_poses = []
id_index = 0
hmd_id = "0"
controller_id = "1"
comp1_id = "1"
delimiter = '\t'
ts = 1
x = 2
y = 3
z = 4
pitch = 5
yaw = 6
roll = 7
out_delimiter = ' '


def read_src_poses(filename):
    with open(filename, "r") as srcfile:
		for line in srcfile.readlines():
			#print len(line)
			pose = line.split(delimiter)
			pose = [v.strip() for v in pose]
			pose = filter(None, pose)
			if pose:
				if pose[id_index] == hmd_id:
					src_hmd_poses.append(pose)
				if pose[id_index] == controller_id:
					src_controller_poses.append(pose)
				try:
					id = int(pose[id_index])
					src_poses[id].append(pose) 
				except: 
  					pass

def write_hmd_out_file(filename):
	counter = 0
	with open(filename + "_RigidBody.csv", "w") as outfile:
		outfile.write("object_id,timestamp,frame_number,tracked,r_0,r_1,r_2,x,r_3,r_4,r_5,y,r_6,r_7,r_8,z,error,qx,qy,qz,qw\n")
		for pose in src_hmd_poses:
			rotx = np.radians(float(pose[pitch]))
			roty = np.radians(float(pose[yaw]))
			rotz = np.radians(float(pose[roll]))
			q = tm.quaternion_from_euler(rotx, roty, rotz, 'sxyz')
			M = tm.euler_matrix(rotx, roty, rotz, 'sxyz')
			#outfile.write("{} {} {} {} {} {} {} {}\n".format(pose[ts], pose[x], pose[y], pose[z], q[1], q[2], q[3], q[0]))
			outfile.write("{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},0,{},{},{},{}\n".format(1, float(pose[ts]) * 1000, counter, 1,
			M[0,0], M[0,1],M[0,2],float(pose[x])*1000,
			M[1,0], M[1,1],M[1,2],float(pose[y])*1000,
			M[2,0], M[2,1],M[2,2],float(pose[z])*1000,
			q[1], q[2], q[3], q[0]))
			counter = counter+1
	outfile.close()	
	
def write_ctrl_out_file(filename):
	counter = 0
	with open(filename + "_ctrl_RigidBody.csv", "w") as outfile:
		outfile.write("object_id,timestamp,frame_number,tracked,r_0,r_1,r_2,x,r_3,r_4,r_5,y,r_6,r_7,r_8,z,error,qx,qy,qz,qw\n")
		for pose in src_controller_poses:
			q = tm.quaternion_from_euler(np.radians(float(pose[pitch])),np.radians(float(pose[yaw])),np.radians(float(pose[roll])), 'sxyz')
			M = tm.euler_matrix(np.radians(float(pose[pitch])),np.radians(float(pose[yaw])),np.radians(float(pose[roll])),'sxyz')
			#outfile.write("{} {} {} {} {} {} {} {}\n".format(pose[ts], pose[x], pose[y], pose[z], q[1], q[2], q[3], q[0]))
			outfile.write("{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},0,{},{},{},{}\n".format(1, pose[ts], counter, 1,
			M[0,0], M[0,1],M[0,2],pose[x],
			M[1,0], M[1,1],M[1,2],pose[y],
			M[2,0], M[2,1],M[2,2],pose[z],
			q[1], q[2], q[3], q[0]))
			counter = counter+1
	outfile.close()		

def write_out_files(fileName):
	for idx, val in enumerate(src_poses):
			# print only list with data
			if len(val) > 1:
				print ("{}, {}".format(idx,len(val)))
				counter = 0
				with open(fileName + str(idx) + "_RigidBody.csv", "w") as outfile:
					outfile.write("object_id,timestamp,frame_number,tracked,r_0,r_1,r_2,x,r_3,r_4,r_5,y,r_6,r_7,r_8,z,error,qx,qy,qz,qw\n")
					for pose in val:
						rotx = np.radians(float(pose[pitch]))
						roty = np.radians(float(pose[yaw]))
						rotz = np.radians(float(pose[roll]))
						q = tm.quaternion_from_euler(rotx, roty, rotz, 'sxyz')
						M = tm.euler_matrix(rotx, roty, rotz, 'sxyz')
						#outfile.write("{} {} {} {} {} {} {} {}\n".format(pose[ts], pose[x], pose[y], pose[z], q[1], q[2], q[3], q[0]))
						outfile.write("{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},0,{},{},{},{}\n".format(1, float(pose[ts]) * 1000, counter, 1,
						M[0,0], M[0,1],M[0,2],float(pose[x])*1000,
						M[1,0], M[1,1],M[1,2],float(pose[y])*1000,
						M[2,0], M[2,1],M[2,2],float(pose[z])*1000,
						q[1], q[2], q[3], q[0]))
						counter = counter+1
				outfile.close()
	
def main():

	start = time.time()

	for i in range (0, 5):
		src_poses.append([])
		

	if len(sys.argv) != 3:
		print "Usage: python", sys.argv[0], "<Brekel_file.txt> <output file name>"
		sys.exit(1)
	
	read_src_poses(sys.argv[1])
	write_out_files(sys.argv[2])
	#write_hmd_out_file(sys.argv[2])
	#write_ctrl_out_file(sys.argv[2])
	
	
	end = time.time()
	print(end - start)
	
	#print "Euler angles:{},{},{} ".format(float(src_hmd_poses[0][pitch]),float(src_hmd_poses[0][yaw]),float(src_hmd_poses[0][roll]))
	#q = tm.quaternion_from_euler(np.radians(float(src_hmd_poses[0][pitch])),np.radians(float(src_hmd_poses[0][yaw])),np.radians(float(src_hmd_poses[0][roll])), 'sxyz')
	#print q 
	#e = tm.euler_from_quaternion(q,'sxyz')
	#print e
	#ed = np.degrees(e)
	#print ed
if __name__ == '__main__':
    main()