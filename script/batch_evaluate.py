import glob
import json
import numpy as np
import os
from pathlib import Path
import subprocess
import zipfile


## Please configure path
CLOSEDLOOP_DIR = "/mnt/DATA/Yanwei/closedloop/2023/20.04/"
IMUS = ["mpu6000", "ADIS16448"]
METHODS = ["ORB3"]

enable_evaluation = True

## Please keep the following values unchanged.
BAGFILES = Path(CLOSEDLOOP_DIR).rglob("*.bag")
enable_planning_time_compenstation = False
desired_path_topic = "/desired_path_compensated"  # "/desired_path"

# ------------------------------------------------------------------------------
def compensate_planning_time(plan_txyz, act_txyz):
    """
    In current implementation, /desired_path from planning module is stamped
    with the time(t0) when triggered, however controller module assigned the
    /desired_path with the latest odometry timestamp(t1). Because a very large
    planning time causes delay and leads the robot to track a very far-away
    waypoint. This function approximates the offset (t0 - t1) by measuring the
    change in act and plan files.
    """

    assert plan_txyz.shape[0] > 10
    assert plan_txyz.shape[1] == 4
    assert act_txyz.shape[0] > 10
    assert act_txyz.shape[1] == 4

    act_idx = 0
    act_timestamps = []
    plan_timestamps = []
    plan_idx = 0
    for dist in np.arange(0.1, 1.1, 0.1):
        while np.linalg.norm(act_txyz[act_idx, 1:]) < dist:
            act_idx += 1
        act_timestamps.append(act_txyz[act_idx, 0])
        while np.linalg.norm(plan_txyz[plan_idx, 1:]) < dist:
            plan_idx += 1
        plan_timestamps.append(plan_txyz[plan_idx, 0])

    timeoffset = np.mean(np.array(plan_timestamps) - np.array(act_timestamps))

    return timeoffset if np.abs(timeoffset) < 10.0 else np.nan


def evo_evaluate(prefix, name, enable_planning_time_compenstation=False):
    basename = name.split(".")[0]
    file_plan = os.path.join(prefix, basename + "_arr_plan.txt")
    file_act = os.path.join(prefix, basename + "_arr_act.txt")
    file_est = os.path.join(prefix, basename + "_arr_est.txt")
    file_result_nav = os.path.join(prefix, basename + "_result_nav.zip")
    file_result_est = os.path.join(prefix, basename + "_result_est.zip")

    # remove existing results
    if os.path.exists(file_result_nav):
        cmd_rm = "rm " + file_result_nav
        subprocess.call(cmd_rm, shell=True)
    if os.path.exists(file_result_est):
        cmd_rm = "rm " + file_result_est
        subprocess.call(cmd_rm, shell=True)

    if enable_planning_time_compenstation:
        act = np.loadtxt(file_act)
        plan = np.loadtxt(file_plan)
        timeoffset = compensate_planning_time(plan[:, :4], act[:, :4])
        if np.isnan(timeoffset):
            raise Exception("Wrong in planning time compenstation")
        plan[:, 0] = plan[:, 0] - timeoffset
        file_old_plan = file_plan
        file_plan = file_plan.replace("_arr_plan", "_arr_plan_comp")
        np.savetxt(file_plan, plan)

    # run evaluation
    print("Evaluating navigation error ...")
    cmd_eval = (
        "evo_ape tum" + " " + file_plan + " " + file_act + " " + "-va --align_origin --save_results " + file_result_nav
    )
    print(cmd_eval)
    subprocess.call(cmd_eval, shell=True)

    print("Evaluating estimation error ...")
    cmd_eval = (
        "evo_ape tum" + " " + file_act + " " + file_est + " " + "-va --align_origin --save_results " + file_result_est
    )
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
    if not os.path.exists(est_file):
        # parsing
        print("Processing ", f)
        cmd_parsing = (
            "rosrun mat_from_rosbag mat_from_rosbag"
            + " "
            + f.as_posix()
            + " "
            + "/visual/odom /odom_sparse "
            + desired_path_topic
        )
        print(cmd_parsing)
        subprocess.call(cmd_parsing, shell=True)

    # call evaluation
    if enable_evaluation:
        print("Evaluating APE ...")
        evo_evaluate(f.parent, f.name, enable_planning_time_compenstation)

print("Done")
