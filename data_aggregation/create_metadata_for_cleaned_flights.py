import json
import numpy as np


# A helper script that generates metadata for the processing the obtained logs.
# The metadata is used by data_handler.m to know where to look for what data.


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

# Pretend freehand times are the last maneuvers, just for simplicity
# Does not make a difference for anything
exp2_freehand_times = {
    "18": [180, 195],
    "19": [195, 210],
    "20": [210, 225],
    "21": [225, 240],
    "22": [240, 255],
    "23": [255, 270],
    "24": [270, 285],
    "25": [285, 300],
    "26": [300, 315],
    "27": [315, 330],
    "28": [330, 345],
    "29": [345, 360],
    "30": [360, 375],
    "31": [375, 390],
    "32": [390, 405],
    "33": [405, 420],
    "34": [420, 435],
    "35": [435, 450],
    "36": [450, 465],
    "37": [465, 480],
    "38": [480, 495],
    "39": [495, 510],
    "40": [510, 525],
    "41": [525, 530],
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

## Correct pitch maneuver times
pitch_211_maneuver_times = {
    9: [-1, 981],  # split maneuvers
    10: [-1, 985.5],  # split maneuvers
    11: [-1, 990.5],  # split maneuvers
    13: [-1, 1011],  # split maneuvers
    14: [-1, 1016],  # split maneuvers
    15: [-1, 1020.5],  # split maneuvers
    16: [-1, 1025],  # split maneuvers
    17: [-1, 1029.5],  # split maneuvers
    19: [-1, 1051],  # split maneuvers
    20: [-1, 1055],  # use two maneuvers in one
}

pitch_211_nt_maneuver_times = {
    22: [-1, 1083],  # take both 22 and 23
    27: [-1, 1150],  # take both 27 and 28
}
set_maneuver_times(
    exp3,
    pitch_211_maneuver_indices,
    {},
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
    37: [1347, 1363],  # Take 1,2,3,4 in one go
    41: [-1, 1379.2],  # Take 5 by itself
    43: [1385.5, 1397],  # Take 7, 8, 9 in one go
    46: [-1, 1429.5],  # Clip away dropout on 10
    49: [-1, 1480.5],  # 13, 14, 15, 16, 17 in one go
    54: [-1, 1502.5],  # 18, 19 in one go
}

set_maneuver_times(exp3, roll_211_maneuver_indices, roll_211_maneuver_times, "roll_211")

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
    105: [2260, 2272],  # 6,7
    111: [-1, 2342],  # 12, 13, next maneuver (14) start at index 149
    161: [-1, 3256],  # 26
    166: [-1, 3333],  # 31
    167: [3343, 3364],  # 32, 33, 34
    172: [3392, 3404],  # 37, 38
    # 40 41 can be included if I need more data
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
    192: [4067, 4080],
    193: [4095, 4110],
    194: [4110, 4125],
    195: [4180, 4195],
    196: [4195, 4210],
    197: [4210, 4225],
    198: [4225, 4240],
    199: [4240, 4255],
    200: [4255, 4270],
    201: [4270, 4285],
    202: [4285, 4300],
    203: [4300, 4315],
    204: [4315, 4325],
}
for key, value in exp3_freehand_times.items():
    exp3["Maneuvers"][str(key)] = create_maneuver_dict("freehand", value[0], value[1])


##############

# Create metadata object
breakpoint()
metadata = {"Experiments": [exp2, exp3]}

with open("../data/flight_data/metadata.json", "w") as outfile:
    json.dump(metadata, outfile, indent=4, sort_keys=True)