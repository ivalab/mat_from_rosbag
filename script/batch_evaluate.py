import glob
import json
import numpy as np
import os
from pathlib import Path
import subprocess
import zipfile

CLOSEDLOOP_DIR = '/mnt/DATA/Yanwei/closedloop/2023/20.04/'
BAGFILES = Path(CLOSEDLOOP_DIR).rglob('*.bag')
# IMUS = ['mpu6000']
IMUS = ['mpu6000', 'ADIS16448']
METHODS = ['DSOL']

enable_evaluation = True

# -------------------------------------------------------------------------------


def evo_evaluate(prefix, name):
    basename = name.split('.')[0]
    file_gt = os.path.join(prefix, basename + '_arr_act.txt')
    file_et = os.path.join(prefix, basename + '_arr_est.txt')
    file_result = os.path.join(prefix, basename + '_result.zip')
    if os.path.exists(file_result):
        cmd_rm = 'rm ' + file_result
        subprocess.call(cmd_rm, shell=True)
    # run evaluation
    cmd_eval = \
        'evo_ape tum' + \
        ' ' + file_gt + \
        ' ' + file_et + \
        ' ' + '-v --save_results ' + file_result
    print(cmd_eval)
    subprocess.call(cmd_eval, shell=True)


# -------------------------------------------------------------------------------
# process bagfiles
for f in BAGFILES:
    name = str(f.as_posix())
    if not any(imu in name for imu in IMUS):
        continue
    if not any(method in name for method in METHODS):
        continue

    est_file = name.replace(".bag", "_arr_act.txt")
    if os.path.exists(est_file):
        continue

    # parsing
    print('Processing ', f)
    cmd_parsing = \
        'rosrun mat_from_rosbag mat_from_rosbag' + \
        ' ' + f.as_posix() + \
        ' ' + '/visual/odom /odom_sparse /desired_path'
    print(cmd_parsing)
    subprocess.call(cmd_parsing, shell=True)

    # call evaluation
    if enable_evaluation:
        print('Evaluating APE ...')
        evo_evaluate(f.parent, f.name)

print('Done')
