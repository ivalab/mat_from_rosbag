import glob
import json
import numpy as np
import os
from pathlib import Path
import subprocess
import zipfile

SeqNameList = ["loop", "long", "square", "zigzag", "two_circle", "infinite"]
SeqNameToTableIndex = ["s1", "s2", "m1", "m2", "l1", "l2"]
SeqLenList = [40, 50, 105, 125, 200, 245]
RESULT_ROOT = "/mnt/DATA/Yanwei/closedloop/2023/20.04/"
ET_FREQ = 100.0

## adjust the variables
ImuPool = ["ADIS16448", "mpu6000"]
VelPool = [0.5, 1.0, 1.5]
NumRepeating = 3
Number_GF_List = [20]  # [40, 60, 80, 120, 160];
MethodPool = ["DSOL"]  # ['GF_GG_skf'] # [ORB3]

## evaluation threshold
RMSE_THRESH = 10.0  # m
TRACKING_THRESH = 0.6

for method in MethodPool:

    for feature in Number_GF_List:

        rmse_table = np.full((len(ImuPool), len(VelPool), len(SeqNameList), NumRepeating), -1.0)

        for imu_index, imu_type in enumerate(ImuPool):

            for vel_index, vel in enumerate(VelPool):

                for seq_index, seq in enumerate(SeqNameList):

                    Experiment_dir = os.path.join(
                        RESULT_ROOT, seq, imu_type, method, "ObsNumber_" + str(int(feature)) + "_Vel" + str(vel)
                    )

                    for round_index in range(NumRepeating):

                        prefix = os.path.join(Experiment_dir, "round" + str(round_index + 1))

                        # print(prefix)
                        # print(os.path.exists(prefix + '.bag'))

                        file_result = prefix + "_result.zip"
                        if not os.path.exists(file_result):
                            print(f"{file_result} not exists")
                            continue

                        with zipfile.ZipFile(file_result, "r") as z:
                            with z.open("stats.json") as f:
                                data = f.read()
                                error = json.loads(data)["rmse"]
                                if error < RMSE_THRESH:
                                    rmse_table[imu_index, vel_index, seq_index, round_index] = error
                                else:
                                    print(f"{file_result} large rmse: {error}")
                                if error > 1.0:
                                    print(f"{file_result} large rmse: {error}")

                        # check tracking ratio
                        file_plan = prefix + "_arr_plan.txt"
                        array_plan = np.loadtxt(file_plan)
                        exp_duration = array_plan[-1, 0] - array_plan[0, 0]
                        file_et = prefix + "_arr_est.txt"
                        array_et = np.loadtxt(file_et)
                        acc_duration = array_et[-1, 0] - array_et[0, 0]
                        if acc_duration < exp_duration * TRACKING_THRESH:
                            print(f"tracking failed: {file_et}, tracking ratio: {acc_duration}")
                            rmse_table[imu_index, vel_index, seq_index, round_index] = -1.0

        cur_table = np.full((len(ImuPool) * len(VelPool), len(SeqNameList) + 1), -1.0)
        for i in range(rmse_table.shape[0]):
            for j in range(rmse_table.shape[1]):
                ridx = i * len(VelPool) + j
                for k in range(rmse_table.shape[2]):
                    indices = rmse_table[i, j, k, :] > 0.0
                    if np.sum(indices) == NumRepeating:
                        cur_table[ridx, k] = np.mean(rmse_table[i, j, k, :], axis=-1)
                indices = cur_table[ridx, :] > 0.0
                if np.sum(indices) == 0:
                    continue
                cur_table[ridx, -1] = np.mean(cur_table[ridx, indices])
        print(cur_table)
        np.savetxt(
            os.path.join(RESULT_ROOT, method + "_" + str(feature) + "_closedloop_vis.txt"),
            cur_table,
            fmt="%.6f",
            delimiter=",",
            header=" ".join(SeqNameList) + " mean ",
        )
        np.savetxt(
            os.path.join(RESULT_ROOT, method + "_" + str(feature) + "_closedloop.txt"),
            rmse_table.reshape(len(ImuPool) * len(VelPool) * len(SeqNameList), -1),
            fmt="%.6f",
            delimiter=",",
            header=" ".join(["Round" + str(i) for i in range(1, NumRepeating + 1)]),
        )
