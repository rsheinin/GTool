import argparse
import os
import GHC_Creator
import configparser
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), '..\\..\\..\\3rdparty\\python'))
import numpy as np
import math
from CalibrationConstants import CalibrationConstants, bcolors


class Clip:

    def __init__(self, ghc=None):
        if ghc is None:
            self.ghc = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0]


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="verifying hand eye calibration results according to defined KPI's")

    parser.add_argument('--config',type=str,required=True, help="config file path for involved clips in calibration results investigation")
    parser.add_argument('--outdir', type=str, required=True, help="a folder path for output file")
    args = parser.parse_args()

    if not os.path.isfile(args.config):
        print("Error: given config file path is wrong.", args.config)
        exit(-1)

    if not os.path.isdir(args.outdir):
        print("Error: given output folder path is wrong.", args.outdir)
        exit(-1)

    return args


def calc_KPI(clip_list,outdir):

    file = open(outdir+'\KPI.csv', 'w')
    file.write('first clip,second clip,rotation_diff_deg,translation_diff_mm\n')
    for i in range(len(clip_list)):
        for j in range(i+1,len(clip_list)):
            inv = np.dot(clip_list[j].calibrator.ghc_mat,np.linalg.inv(clip_list[i].calibrator.ghc_mat))
            rotation_diff_deg = math.degrees(float(math.acos((np.trace(inv) - 2) / 2)))
            if rotation_diff_deg > CalibrationConstants.max_rotation_degrees:
                print(bcolors.WARNING + 'Calibration Failed!!! process results exceed the KPIs, rotation diff deg is : '  +bcolors.FAIL+ str(rotation_diff_deg)+ bcolors.ENDC)
            x,y,z = inv[0, 3], inv[1, 3], inv[2, 3]
            translation_diff_mm = math.sqrt(x*x+y*y+z*z)
            if translation_diff_mm > CalibrationConstants.max_translation_mm:
                print ( bcolors.WARNING + 'Calibration Failed!!! process results exceed the KPIs, translation diff mm is : ' +bcolors.FAIL+ str(translation_diff_mm)+ bcolors.ENDC)
            file.write(os.path.basename(clip_list[i].calibrator.src_file_name)+','+os.path.basename(clip_list[j].calibrator.src_file_name)+','+str(rotation_diff_deg)+','+str(translation_diff_mm) + '\n')
    file.close()


def main():
    args = parse_arguments()
    config = configparser.ConfigParser()
    config.read(args.config)

    clip_list = []
    for each_section in config.sections():
        clip = Clip()
        for (each_key, each_val) in config.items(each_section):
            setattr(clip, each_key, each_val)
        GHC_Creator.arguments_verification(clip)
        clip.calibrator = GHC_Creator.run(clip)
        clip_list.append(clip)
        
    calc_KPI(clip_list, args.outdir)


if __name__ == '__main__':
    main()
