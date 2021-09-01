import subprocess

import shutil
import signal
import re
import os
import csv
import numpy as np
import sys
import configparser
import copy
import platform
from itertools import groupby

PosesReader_path='../Common/python/PosesReader'
if platform.system() == 'Windows':
    sys.path.append(os.path.join(os.path.dirname(__file__), PosesReader_path))
elif platform.system() == 'Linux':
    sys.path.insert(0, PosesReader_path)

from PosesReader import Format_File, Poses_Container, Poses_Utils

signal.signal(signal.SIGINT, signal.SIG_IGN)


class Recorder:

    def recorder_path(self):pass
    def out_dir (self): pass
    def outputName(self):pass

    def __init__(self,outputName, outdir,recorderpath):
        self.outputName=outputName
        self.out_dir=outdir
        self.recorder_path=recorderpath
        self.createOutFolder()

    def createOutFolder(self):
        if not os.path.exists(self.out_dir+"/"+self.outputName):
            os.makedirs(self.out_dir+"/"+self.outputName)
        self.out_dir = self.out_dir+"/"+self.outputName
        if platform.system() == 'Linux':
            command = "sudo chmod 777 " + self.out_dir
            os.system(command)

    def run(self):
        raise NotImplementedError("GT controller must implement run func")

    def arrangeData(self,mounted_dir,exporting_script_path):
        raise NotImplementedError("GT controller must implement arrangeData func")

    def testOutput(self,testerPath):
        raise NotImplementedError("GT controller must implement testOutput func")


class TM2_Recorder(Recorder):
    recordMode = ''

    def __init__(self,recordMode,outputName, outdir,recorderpath=None):
        if recorderpath is None:
            if sys.platform is 'Linux':
                recorderpath = "/home/ntuser/Desktop/TM2_client"
        super(TM2_Recorder, self).__init__(outputName, outdir,recorderpath)
        self.recordMode=recordMode



    def run(self,controller=None):
        if self.recordMode == '6c':
            p = subprocess.Popen(
                ['sudo', '-S', self.recorder_path+'/tm2_client', '-6', self.out_dir+"/"+self.outputName + '.txt', '-c',self.out_dir+"/"+ self.outputName + '.rc'], )
        elif self.recordMode=='c':
            p = subprocess.Popen(
                ['sudo', '-S', self.recorder_path+'/tm2_client', '-c', self.out_dir+"/"+self.outputName + '.rc', '-m', "276e02cd31e4"], )
        elif self.recordMode=='6':
            p = subprocess.Popen(
                ['sudo', '-S', self.recorder_path  + '/tm2_client', '-6',self.out_dir+"/"+ self.outputName + '.txt', '-m', "276e02cd31e4"], )
        else:
            p = subprocess.Popen(['sudo', '-S', os.getcwd() + '/tm2_client', '-'], )

        p.communicate('q')

        print("Process ID of subprocess {}".format(p.pid))

        returncode = p.wait()
    
        print("Return code of subprocess: {}".format(returncode))

    def arrangeData(self, mounted_dir,exporting_script_path):
        extracted_dir = "extracted"
        if self.recordMode == '6c' or self.recordMode == 'c':
            print("Extracting {} to {}".format(self.outputName + ".rc ", mounted_dir))
            command = "sudo python " + exporting_script_path + "RC_exporting_with_ctrl_no_images.py" + " " + self.out_dir+"/"+self.outputName + ".rc " + mounted_dir + "//" + extracted_dir
            os.system(command)
            command = "sudo chmod 777 " + mounted_dir + "/" + extracted_dir
            os.system(command)

        if self.recordMode == '6c' or self.recordMode == '6':
            #shutil.move(self.outputName + '.txt', self.out_dir)
            command = "sudo chmod 777 " + self.out_dir + "/" + self.outputName + '.txt'
            os.system(command)
            command = "sudo cp " + self.out_dir + "/" + self.outputName + '.txt' + ' ' + mounted_dir + '/' +  self.outputName + '.txt'
            os.system(command)

    def testOutput(self,testerPath):
        if self.recordMode == '6c' or self.recordMode == 'c':
            print("------")
            print("Running capture check over {} ...............".format(self.outputName + ".rc "))
            command = "sudo python " + testerPath + "capturecheck.py" + " " + self.out_dir + "/" + self.outputName + ".rc "
            os.system(command)
        if self.recordMode == '6c' or self.recordMode == '6':
            if 'nan' in open(self.out_dir + "/" + self.outputName + ".txt").read():
                print("-------------------------------------------------------")
                print("ERROR: nan: in {} 6Dof file".format(self.out_dir + "/" + self.outputName + ".txt"))


class Host_Recorder(Recorder):

    time = 30
    def __init__(self,time, outputName, outdir, recorder_path=None):
        if platform.system() == 'Linux':
            if recorder_path is None:
                recorder_path = "/usr/bin/"
            self.extention = ""
        if platform.system() == 'Windows':
            if recorder_path is None:
                recorder_path = "C:\\Users\\srosentx\\Downloads\\drop-win64-804266\\bin\\"
            self.extention = ".exe"
        super(Host_Recorder, self).__init__(outputName, outdir,recorder_path)
        self.time = time

    def move_file(self,mounted_dir):

        for file in os.listdir(self.out_dir):
            if file.endswith(".txt"):
                shutil.move(self.out_dir+'/' +file, mounted_dir + '/' + file)
        if platform.system() == 'Linux':
            command = "sudo chmod -R 777 " + Recorder.mounted_dir
            os.system(command)

    def run(self,ctrl1=None,ctrl2=None):
        command = self.recorder_path+"/realsense_tm_capture_tool"+self.extention+" -record " + self.out_dir + "/" +  self.outputName  + ".bag -time "+self.time +"-fw_log info "+self.out_dir + "/" +  self.outputName+"_debug"

        if ctrl1 is not None:
            command += " -controller " + ctrl1.mac_address
        if ctrl2 is not None:
            command += " -controller " + ctrl2.mac_address
        os.system(command)

    def arrangeData(self,mounted_dir,exporting_script_path):
        command = self.recorder_path + "/realsense_tm_capture_tool"+self.extention+" -convert " + self.out_dir + "/" + self.outputName + ".bag " + self.out_dir + "/" + self.outputName + " algo_data -convert " + self.out_dir + "/" + self.outputName + ".bag " + self.out_dir + "/" + self.outputName + " sensors_data "
        os.system(command)

        poses_container = Poses_Container(file_name=self.out_dir + '/' + self.outputName + '_pose.csv', format=Format_File(configFilePath='../Common/python/PosesReader/KnownFormats/host_pose_sdk.ini'))
        poses_container.read_pose_file()
        poses_container.write_pose_file(output_file_name=self.out_dir+'/'+self.outputName+'.txt', format=Format_File(configFilePath='../Common/python/PosesReader/KnownFormats/TUM.ini'))

        motion_container = Poses_Container(file_name=self.out_dir+'/'+self.outputName+'_motion.csv', format=Format_File(configFilePath='../Common/python/PosesReader/KnownFormats/host_motion_sdk.ini'))

        print('poses writen into '+ self.out_dir+'/gyro.txt')
        gyro_container = copy.copy(motion_container)
        gyro_container.poses = [item for item in gyro_container.poses if item['type'].startswith('gyro')]
        gyro_container.write_pose_file(output_file_name=self.out_dir+'/gyro.txt', format=Format_File(configFilePath='../Common/python/PosesReader/KnownFormats/motion.ini'))

        accel_container = copy.copy(motion_container)
        print('poses writen into '+ self.out_dir+'/accel.txt')
        accel_container.poses = [item for item in accel_container.poses if item['type'].startswith('accel')]
        accel_container.write_pose_file(output_file_name=self.out_dir+'/accel.txt', format=Format_File(configFilePath='../Common/python/PosesReader/KnownFormats/motion.ini'))


        self.move_file(mounted_dir)



    def testOutput(self,testerPath):
        print("Running capture check over {} ...............".format(self.outputName + ".bag "))
        command = self.recorder_path+"/realsense_tm_capture_check"+self.extention+" -f "+ self.out_dir + "/" + self.outputName + ".bag -s -e"
        os.system(command)

