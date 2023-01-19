#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file batch_clean.py
@author Yanwei Du (duyanwei0702@gmail.com)
@date 02-06-2023
@version 1.0
@license Copyright (c) 2023
@desc None
"""


"""
!!! USE WITH CAUTION !!!
"""

import glob
import json
import numpy as np
import os
from pathlib import Path
import subprocess
import zipfile

CLOSEDLOOP_DIR = "/mnt/DATA/Yanwei/closedloop/2023/20.04/"
# IMUS = ['mpu6000']
IMUS = ["mpu6000", "ADIS16448"]
METHODS = ["GFGG"]

REMOVE_ONLY_BAG = True
REMOVE_WHOLE_METHOD = False

# -------------------------------------------------------------------------------
# remove methods
if REMOVE_WHOLE_METHOD:
    print(f"Removing whole method ...")
    for method in METHODS:
        method_dir = Path(CLOSEDLOOP_DIR).rglob(method)
        for f in method_dir:
            # parsing
            cmd_rm = "rm -r " + str(f.as_posix())
            print(cmd_rm)
            subprocess.call(cmd_rm, shell=True)

# remove bagfiles
elif REMOVE_ONLY_BAG:
    print("Removing bags ...")
    BAGFILES = Path(CLOSEDLOOP_DIR).rglob("*.bag")
    for f in BAGFILES:
        name = str(f.as_posix())
        if not any(imu in name for imu in IMUS):
            continue
        if not any(method in name for method in METHODS):
            continue

        # parsing
        cmd_rm = "rm " + name
        print(cmd_rm)
        # subprocess.call(cmd_rm, shell=True)

print("Done")

# -------------------------------------------------------------------------------
