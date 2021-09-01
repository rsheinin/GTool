import math

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

class CalibrationConstants:

    #filter
    MAX_ANGLE_MAGNITUDE_DIFF = 0.1 / 180.0 * math.pi  # 0.1 degree
    MIN_ANGLE_MAGNITUDE = 5.0 / 180.0 * math.pi  # 5 degrees
    max_dist_from_RPE_avg = 0.9

    #Calibrator
    interval_in_ms_r = 300
    interval_in_ms_t = 500
    interval_in_ms_s = 1000
    min_relative_matrices_num = 20

    #RelativePoses
    max_interval = 1.1

    #KPI
    max_rotation_degrees = 1
    max_translation_mm = 1