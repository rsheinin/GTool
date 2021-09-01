import os
import sys
import numpy
import platform
import matplotlib.pyplot as plt
PosesReader_path='../Common/python/PosesReader'
if platform.system() == 'Windows':
    sys.path.append(os.path.join(os.path.dirname(__file__), PosesReader_path))
elif platform.system() == 'Linux':
    sys.path.insert(0, PosesReader_path)

from PosesReader import Format_File, Poses_Container, Poses_Utils

class GT_TEST_DEFULTS:
    trackedPrecent = 0.8
    errorAvgMax = 0.0007
    maxLostTracking = 1  # in second
    thresholdValidGT = 1


class GTCheck():

    def __init__(self, GT_file_path):

        self.GT_container = Poses_Container(file_name=GT_file_path, format=self.get_gt_format())
        self.GT_container.read_pose_file()

        self.tests_on_GT = [{'test': self.errorAVG, 'errorMsg': "GT ERROR: error average is too high! (higher than "+str(GT_TEST_DEFULTS.errorAvgMax)+" )."},
                   {'test': self.amountOfTrack, 'errorMsg': "GT ERROR: Not enough tracked poses. (minimum precent of tracked poses is "+str(GT_TEST_DEFULTS.trackedPrecent) +" )."},
                   {'test': self.massiveLostTracking, 'errorMsg': "GT ERROR: more than "+str(GT_TEST_DEFULTS.maxLostTracking)+" second lost tracking."},
                    {'test': self.spikes, 'errorMsg': "GT ERROR: too much spikes."}]


    def get_gt_format(self):
        return Format_File(configFilePath = os.path.join(os.path.dirname(__file__), '..\\Common\\python\\PosesReader\\KnownFormats\\GT.ini'))


    def runTests(self):
        GTisGood = True
        CRED = '\033[91m'
        CEND = '\033[0m'
        CGREEN='\033[32m'

        for test in self.tests_on_GT:
            # run test:
            test_res = test['test']()
            if not test_res[0]:
                print(CRED + test['errorMsg'] + '(test results is {0})'.format(test_res[1])+ CEND)
                GTisGood = False
            else:
                print(CGREEN+ 'Test {0} passed, test results is : {1}'.format(test['test'].__name__, test_res[1]) + CEND)
        return GTisGood


    ### TESTS ###

    def massiveLostTracking(self):
        if 'time_factor_to_ms' in self.GT_container.format.general_definition:
            tracked_poses = [pose['timestamp']*float(self.GT_container.format.general_definition['time_factor_to_ms'])/1000 for pose in self.GT_container.poses  if 'tracked' in pose and pose['tracked']==1]
            for i in range(1,len(tracked_poses)):
                if tracked_poses[i] - tracked_poses[i-1] >= GT_TEST_DEFULTS.maxLostTracking:
                    return (False, None)
        else:
            print("no time_factor_to_ms in poses definitions.")
        return (True, None)

    def amountOfTrack(self):
        tracked_amount = len([True for pose in self.GT_container.poses if 'tracked' in pose and pose['tracked'] == 1])/len(self.GT_container.poses)
        return (tracked_amount >= GT_TEST_DEFULTS.trackedPrecent , tracked_amount)

    def errorAVG(self):
        meanError = numpy.mean([pose['error'] for pose in self.GT_container.poses if 'tracked' in pose  and pose['tracked'] == 1 and 'error' in pose])
        return (meanError < GT_TEST_DEFULTS.errorAvgMax, meanError)

    def spikes(self, plot = False):

        Poses_Utils.apply_list_on_poses(self.GT_container, [Poses_Utils.add_rot_trace])

        count = 0
        lastInvalid = None
        lastValid = None
        i = 0
        while i< len(self.GT_container.poses):
            while 'tracked' in self.GT_container.poses[i] and self.GT_container.poses[i]['tracked'] == 0:
                i += 1
            if i >= len(self.GT_container.poses):
                break

            if lastInvalid is not None and abs(lastInvalid['trace'] - self.GT_container.poses[i]['trace']) < GT_TEST_DEFULTS.thresholdValidGT:
                self.GT_container.poses[i]['tracked'] = 0
                count += 1
                lastInvalid = self.GT_container.poses[i]

            if lastInvalid is None or abs(lastInvalid['trace'] - self.GT_container.poses[i]['trace']) > GT_TEST_DEFULTS.thresholdValidGT:
                lastValid = self.GT_container.poses[i]

            while 'tracked' in self.GT_container.poses[i] and i < len(self.GT_container.poses) and self.GT_container.poses[i]['tracked'] == 1:
                if 'trace' in self.GT_container.poses[i] and abs(self.GT_container.poses[i]['trace'] - lastValid['trace']) < GT_TEST_DEFULTS.thresholdValidGT:
                    lastValid = self.GT_container.poses[i]
                else:
                    self.GT_container.poses[i]['tracked'] = 0
                    count += 1

                    lastInvalid = self.GT_container.poses[i]
                i += 1
            i += 1
        if plot :
            start_trace = [(pose['timestamp'], pose['trace']) for pose in self.GT_container.poses]
            end_trace = [(pose['timestamp'], pose['trace']) for pose in self.GT_container.poses if pose['tracked'] == 1]
            self.plot_lists(start_trace, end_trace)

        print (str(count) + " frames were removed from GT repo by cleaning up spikes")
        return self.amountOfTrack()


    def plot_lists(self, l1, l2, title = None):

        x1 = [i[0] for i in l1]
        y = [i[1] for i in l1]

        x2 = [i[0] for i in l2]
        z = [i[1] for i in l2]

        plt.xlabel("X-axis")
        plt.ylabel("Y-axis")
        title = title or "values"
        plt.title(title)

        plt.plot(x1, y, 'r', marker='o')  # plotting t, a separately
        plt.plot(x2, z, 'b', marker='o')  # plotting t, b separately
        plt.show()

