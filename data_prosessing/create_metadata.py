import json
import numpy as np


# A helper script that generates metadata for the processing the obtained logs.
# The metadata is used by data_handler.m to know where to look for what data.

# This script is mostly hard-coded and not very readable. Only experiment 2 and 3 are used. The maneuver number to those in the folder plots_from_exp_2_exp_3.


def create_maneuver_dict(maneuver_type="not_set", maneuver_start=-1, maneuver_end=-1):
    d = {
        "type": maneuver_type,
        "start_s": maneuver_start,
        "end_s": maneuver_end,
    }
    return d


def set_maneuver_times(exp_dict, maneuver_indices, maneuver_times, maneuver_type):
    for maneuver_i in maneuver_indices:
        start_time = -1
        end_time = -1
        if maneuver_i in maneuver_times:
            times = maneuver_times[maneuver_i]
            start_time = times[0]
            end_time = times[1]

        exp_dict["Maneuvers"][str(maneuver_i)] = create_maneuver_dict(
            maneuver_type, start_time, end_time
        )
    return


##############
# Experiment 2
##############

# Experiment from no-wind day with Pero as pilot
# This experiment contains:
#   Pitch 2-1-1
#   Freehand coupled flight

exp2 = {}
exp2["LogName"] = "07_12_32"
exp2["Number"] = 2
exp2["Maneuvers"] = {}

pitch_211_maneuver_indices = np.arange(1, 18)
set_maneuver_times(exp2, pitch_211_maneuver_indices, {}, "pitch_211")

# Correct pitch maneuver times
pitch_211_maneuver_times = {
    1: [535, 540.5],  # split maneuvers
    2: [-1, -1],  # skip, disturbance
    3: [-1, -1],  # skip, disturbance
    7: [-1, -1],  # skip, dropout
    8: [-1, -1],  # skip, lateral movement
    9: [-1, -1],  # skip, disturbance
    11: [-1, -1],  # skip, dropout
    14: [-1, -1],  # skip, disturbance
    17: [-1, -1],  # skip, dropout
}

set_maneuver_times(
    exp2, pitch_211_maneuver_indices, pitch_211_maneuver_times, "pitch_211"
)

# Pick some freehand maneuvers for validation
exp2_freehand_times = {
    "18": [135, 150], # transition
    "19": [193, 206], # some roll stuff
    "20": [212, 229], # some mixed stuff
    "21": [338, 349],
}
for key, value in exp2_freehand_times.items():
    exp2["Maneuvers"][str(key)] = create_maneuver_dict("freehand", value[0], value[1])


##############
# Experiment 3
##############

# Experiment from no-wind day with Pero as pilot
# This experiment contains:
# Contains:
# 2-1-1 on pitch
# 2-1-1 on pitch without thrust
# 2-1-1 on roll
# 2-1-1 on roll without thrust
# 2-1-1 on yaw
# 2-1-1 on yaw without thrust
# sweeps on pitch
# freehand

exp3 = {}
exp3["LogName"] = "07_24_19"
exp3["Number"] = 3
exp3["Maneuvers"] = {}

# Indices for the different 2-1-1 maneuvers
pitch_211_maneuver_indices = np.arange(1, 22)
pitch_211_nt_maneuver_indices = np.arange(22, 37)
sweep_maneuver_indices = np.arange(177, 192)


# index 3 is maneuver 20. Maneuver start at maneuver no 19
# Correct pitch maneuver times
pitch_211_maneuver_times = {
    1: [-1, -1],  # skip, dropout
    3: [906, -1],  # clean maneuver
    4: [913, -1],  # clean maneuver
    5: [920.3, -1],  # clean maneuver
    6: [938.3, -1],  # clean maneuver
    7: [-1, -1],  # dropout
    8: [-1, -1],  # dropout
    9: [-1, 981],  # clean maneuver
    10: [980, 985.5],  # clean maneuver
    11: [-1, 990],  # clean maneuver
    12: [990, 995],  # clean maneuver
    13: [1006, 1011],  # clean maneuver
    14: [1010.5, 1015],  # clean maneuver
    15: [-1, -1],  # disturbance
    16: [1019, 1025],  # disturbance
    17: [1024, 1029.5],  # disturbance
    18: [-1, -1],  # dropout
    19: [-1, 1051],  # dropout
    20: [-1, 1055],  # dropout
}

pitch_211_nt_maneuver_times = {
    22: [-1, 1083],  # take both 22 and 23
    27: [-1, 1150],  # take both 27 and 28
}
set_maneuver_times(
    exp3,
    pitch_211_maneuver_indices,
    pitch_211_maneuver_times,
    "pitch_211",
)

set_maneuver_times(
    exp3,
    pitch_211_nt_maneuver_indices,
    {},
    "pitch_211_no_throttle",
)


# ROLL MANEUVERS

roll_211_maneuver_indices = np.arange(37, 57)
roll_211_nt_maneuver_indices = np.arange(57, 100)

# Correct roll maneuver times
roll_211_maneuver_times = {
    37: [1347, 1351], # 1
    38: [1352, 1355.5],
    39: [1356, 1360],
    40: [1359, 1362.8], # 4
    41: [1375, 1379.2], # 5
    # Skip 6
    43: [1385.5, 1390.5], # 7
    44: [1389.5, 1394], # 8
    45: [1393, 1397], # 9
    46: [1425, 1429.5],  # Clip away dropout on 10
    # Skip 11
    # Skip 12
    49: [1455.5, 1460.5], # 13
    #50: [1460.5, 1465], # Skip 14
    51: [1465.5, 1470.5], # 15
    52: [1470.5, 1476.5],  # 16
    53: [1476, 1480.5],  # 17
    54: [1491, 1497.5], # 18
    55: [1496.5, 1502.5], # 19
}

set_maneuver_times(exp3, roll_211_maneuver_indices, roll_211_maneuver_times, "roll_211")

# No throttle maneuvers are not used
roll_211_nt_maneuver_times = {
    57: [-1, 1551],  # 57 and 58
    61: [-1, 1595],  # 61 and 62
    76: [-1, 1764],  # 61 and 62
    92: [-1, 1990],  # 92 and 93
}

set_maneuver_times(
    exp3,
    roll_211_nt_maneuver_indices,
    {},
    "roll_211_no_throttle",
)


yaw_211_maneuver_indices = np.hstack((np.arange(100, 113), np.arange(149, 177)))
yaw_211_nt_maneuver_indices = np.arange(113, 149)

yaw_211_maneuver_times = {
    # Skip 1,2,3,4,5
    #105: [2261, 2272],  # skip 6,7
    # Skip 8,9 dropout
    # Skip 10 disturbance
    # Skip 11 disturbance
    111: [-1, 2335.5],  # 12
    112: [-1, -1],  # 13
    149: [-1, 3071.3],  # 14 starts at index 149!
    150: [3073,3081], # 15
    151: [-1,-1], # 16 Small disturbance but I think this is okay
    152: [-1,3140], # 17
    #153: [-1,3147], # 18 Skip 18
    154: [3157,-1], # 19
    155: [-1,-1], # 20
    156: [3195,-1], # 21
    #157: [3195,-1], # Skip 22
    #158: [-1,-1], # Skip 23
    #159: [-1,-1], # Skip 24
    #160: [-1,-1], # Skip 25, dropout
    #161: [-1, 3256],  # 26 Skip 26
    #162: [-1, -1],  # Skip 27
    163: [3273, -1],  # 28
    #164: [3282, -1],  # Skip 29
    #165: [-1, -1],  # Skip 30
    166: [3325, 3332],  # 31
    167: [3343, 3349.5],  # 32
    168: [3350, 3357],  # 33
    169: [3356, 3364],  # 34
    #170: [-1, -1],  # 35 Dropout
    #171: [-1, -1],  # 36 Disturbance
    #172: [3391.5, -1],  # 37
    #173: [01, 3404],  # 38
    #174: [-1, -1],  # 39
    175: [3409.5, 3415.5],  # 40
    #176: [3414.5, -1],  # 41
}

set_maneuver_times(exp3, yaw_211_maneuver_indices, yaw_211_maneuver_times, "yaw_211")

yaw_211_nt_maneuver_times = {}

set_maneuver_times(
    exp3,
    yaw_211_nt_maneuver_indices,
    {},
    "yaw_211_no_throttle",
)

sweep_maneuver_times = {}
set_maneuver_times(exp3, sweep_maneuver_indices, {}, "sweep")


# Add freehand times
exp3_freehand_times = {
    192: [1283, 1302],
#    192: [4067, 4080],
#    193: [4095, 4110],
#    194: [4110, 4125],
#    195: [4180, 4195],
#    196: [4195, 4210],
#    197: [4210, 4225],
#    198: [4225, 4240],
#    199: [4240, 4255],
#    200: [4255, 4270],
#    201: [4270, 4285],
#    202: [4285, 4300],
#    203: [4300, 4315],
#    204: [4315, 4325],
}
for key, value in exp3_freehand_times.items():
    exp3["Maneuvers"][str(key)] = create_maneuver_dict("freehand", value[0], value[1])


##############

# Create metadata object
metadata = {"Experiments": [exp2, exp3]}

with open("../data/flight_data/metadata.json", "w") as outfile:
    json.dump(metadata, outfile, indent=4, sort_keys=True)
