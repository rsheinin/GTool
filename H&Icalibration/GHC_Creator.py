import argparse
import math
import os
import CalibrationUtils
from CalibrationUtils import Plot
from CalibrationConstants import CalibrationConstants
from HandEyeCalibration import Calibrator
from DataHolder import Gyro, Sixdof, OT


def arguments_verification(args):

    if hasattr(args,'gt') and not os.path.isfile(args.gt):
        print("Error: given GT file path is wrong.", args.gt)
        exit(-1)
    if hasattr(args,'gyro') and args.gyro is not None and not os.path.isfile(args.gyro):
        print("Error: given gyro file path is wrong.", args.gyro)
        exit(-1)
    if hasattr(args,'sixdof') and args.sixdof is not None and not os.path.isfile(args.sixdof):
        print("Error: given sixdof file path is wrong.", args.sixdof)
        exit(-1)
    if hasattr(args,'outdir') and args.outdir is not None:
        CalibrationUtils.make_dir(args.outdir)
        if not os.path.isdir(args.outdir):
            print("Error: given outdir file path is wrong.", args.outdir)
            exit(-1)
    if hasattr(args,'gt_config') and args.gt_config is not None and not os.path.isfile(args.gt_config):
        print("Error: given gt config file path is wrong.", args.gt_config)
        exit(-1)
    if hasattr(args,'src_config') and args.src_config is not None and not os.path.isfile(args.src_config):
        print("Error: given src config file path is wrong.", args.src_config)
        exit(-1)

    if not hasattr(args, 't_xyz'):
        args.t_xyz = None
    if not hasattr(args, 'outdir'):
        args.outdir = None
    if not hasattr(args, 'gt_config'):
        args.gt_config = None
    if not hasattr(args, 'src_config'):
        args.src_config = None
    if not hasattr(args, 'show'):
        args.show = False


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Hand Eye Calibration")

    parser.add_argument("--gyro", type=str, default=None, help="input gyro file path")
    parser.add_argument("--sixdof", type=str, default=None, help="input 6dof file path")
    parser.add_argument("--gt", type=str, required=True, help="input gt file path")
    parser.add_argument("--outdir", type=str, help="output directory")
    parser.add_argument('--t_xyz', nargs='+', type=float, default=None,
                        help="insert known translation distance between systems")
    parser.add_argument('--ghc', nargs='+', type=float, help='initial ghc metric',
                        default=[1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0])
    parser.add_argument('--gt_config', type=str, help="input config file for gt")
    parser.add_argument('--src_config', type=str, help="input config file for src")
    parser.add_argument('--show', action='store_true', default=False,help="show a plot with relative matrices")

    args = parser.parse_args()
    arguments_verification(args)
    return args


def run(args):
    if hasattr(args,'gyro') and args.gyro:
        src_data = Gyro(file_path=args.gyro, config_name='IMU', config_file_path=args.src_config)
    elif hasattr(args,'sixdof') and args.sixdof:
        src_data = Sixdof(file_path=args.sixdof, config_name='TUM', config_file_path=args.src_config)

    if hasattr(args,'gt') and args.gt:
        gt_data = OT(file_path=args.gt, config_name='GT', config_file_path=args.gt_config)

    t_xyz = args.t_xyz

    calibrator = Calibrator(gt_data, src_data, args.ghc)

    while True:
        print("--------------------------------Calibration Iteration----------------------------------")
        prev_score = calibrator.score
        prev_ghc = calibrator.ghc_mat
        calibrator.Tsai_algorithm(t_xyz=t_xyz)

        calibrator.calculate_score()
        print("score:", calibrator.score)
        print("GHC:", calibrator.ghc_mat.flatten().tolist())
        if calibrator.score >= prev_score or math.isnan(calibrator.score):  # score does not change anymore or even getting worse
            break

    calibrator.ghc_mat = prev_ghc #go back to previous best GHC matrix
    calibrator.calculate_score(final=True)
    print ("\nRPE average with interval of:", CalibrationConstants.interval_in_ms_s, "ms is:", calibrator.score, "mm",
           "\nRRE average with interval of:", CalibrationConstants.interval_in_ms_s, "ms is:",
           calibrator.calculate_RRE(), "degrees")
    calibrator.print_results(GT_file_path=args.gt, outdir=args.outdir)

    projected_rel_gt = calibrator.project_rel_gt()

    if args.show:
        p = Plot(projected_rel_gt, calibrator.rel_score.rel_src)
        p.plot_calibration()

    return calibrator


def main():
    args = parse_arguments()
    run(args)


if __name__ == '__main__':
    main()

