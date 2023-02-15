import glob
import json
import numpy as np
import os
from pathlib import Path
import subprocess
import zipfile

## adjust the variables
RESULT_ROOT = "/mnt/DATA/Yanwei/closedloop/2023/20.04/"
ImuPool = ["ADIS16448", "mpu6000"]
VelPool = [0.5, 1.0, 1.5]
NumRepeating = 3
Number_GF_List = [400, 800]  # [40, 60, 80, 120, 160];
MethodPool = ["ORB3"]  # ['GF_GG_skf'] # [ORB3]

# dataset variables
SeqNameList = ["loop", "long", "square", "zigzag", "two_circle", "infinite"]
SeqNameToTableIndex = ["s1", "s2", "m1", "m2", "l1", "l2"]
SeqLenList = [40, 50, 105, 125, 200, 245]

## evaluation parameters
ResultFilePool = ["nav", "est"]
RMSE_THRESH = 10.0  # m
TRACKING_THRESH = 0.6
ET_FREQ = 100.0

for method in MethodPool:

    for feature in Number_GF_List:

        rmse_table = np.full((len(ResultFilePool), len(ImuPool), len(VelPool), len(SeqNameList), NumRepeating), -1.0)

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

                        for file_index, file_type in enumerate(ResultFilePool):
                            file_result = prefix + "_result_" + file_type + ".zip"
                            if not os.path.exists(file_result):
                                print(f"{file_result} not exists")
                                continue

                            with zipfile.ZipFile(file_result, "r") as z:
                                with z.open("stats.json") as f:
                                    data = f.read()
                                    error = json.loads(data)["rmse"]
                                    if error < RMSE_THRESH:
                                        rmse_table[file_index, imu_index, vel_index, seq_index, round_index] = error
                                    else:
                                        print(f"{file_result} large rmse: {error}")
                                    # if error > 1.0:
                                    # print(f"{file_result} large rmse: {error}")

                        # check tracking ratio
                        file_plan = prefix + "_arr_plan.txt"
                        array_plan = np.loadtxt(file_plan)
                        exp_duration = array_plan[-1, 0] - array_plan[0, 0]
                        file_et = prefix + "_arr_est.txt"
                        array_et = np.loadtxt(file_et)
                        acc_duration = array_et[-1, 0] - array_et[0, 0]
                        if acc_duration < exp_duration * TRACKING_THRESH:
                            print(f"tracking failed: {file_et}, tracking ratio: {acc_duration}")
                            for file_index in range(ResultFilePool):
                                rmse_table[file_index, imu_index, vel_index, seq_index, round_index] = -1.0

        for file_index, file_type in enumerate(ResultFilePool):
            cur_table = np.full((len(ImuPool) * len(VelPool), len(SeqNameList) + 1), -1.0)
            for i in range(rmse_table.shape[1]):
                for j in range(rmse_table.shape[2]):
                    ridx = i * len(VelPool) + j
                    for k in range(rmse_table.shape[3]):
                        indices = rmse_table[file_index, i, j, k, :] > 0.0
                        if np.sum(indices) == NumRepeating:
                            cur_table[ridx, k] = np.mean(rmse_table[file_index, i, j, k, :], axis=-1)
                    indices = cur_table[ridx, :] > 0.0
                    if np.sum(indices) == 0:
                        continue
                    cur_table[ridx, -1] = np.mean(cur_table[ridx, indices])
            print(cur_table)
            file_name = "_closedloop_" + file_type
            np.savetxt(
                os.path.join(RESULT_ROOT, method + "_" + str(feature) + file_name + "_vis.txt"),
                cur_table,
                fmt="%.6f",
                delimiter=",",
                header=" ".join(SeqNameList) + " mean ",
            )
            np.savetxt(
                os.path.join(RESULT_ROOT, method + "_" + str(feature) + file_name + ".txt"),
                rmse_table[file_index, :, :, :, :].reshape(len(ImuPool) * len(VelPool) * len(SeqNameList), -1),
                fmt="%.6f",
                delimiter=",",
                header=" ".join(["Round" + str(i) for i in range(1, NumRepeating + 1)]),
            )
