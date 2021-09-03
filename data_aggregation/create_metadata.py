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
# Experiment 1
##############

# Experiment from no-wind day with Pero as pilot
# This experiment contains:
#   Sweeps
#   Some cruise flight
# NB: Not used in the end

exp1 = {}
exp1["LogName"] = "06_31_21"
exp1["Number"] = 1
exp1["Maneuvers"] = {}

sweep_maneuver_indices = np.arange(1, 32)
maneuvers_w_dropout = [1, 8, 17, 21]
skip = [2]  # too close to other maneuvers
set_maneuver_times(exp1, sweep_maneuver_indices, {}, "sweep")

# Pick some random cruise flight times
exp1_cruise_times = {
    31: [640, 650],
    32: [650, 670],
    33: [720, 740],
    34: [770, 790],
    35: [900, 920],
    36: [1160, 1180],
    37: [1970, 1990],
    38: [1990, 1997],
    39: [2015, 2035],
    40: [2060, 2080],
    41: [2080, 2100],
    42: [2115, 2130],
    43: [2140, 2155],
    44: [2175, 2190],
    45: [2230, 2245],
    46: [2250, 2270],
}
for key, value in exp1_cruise_times.items():
    exp1["Maneuvers"][str(key)] = create_maneuver_dict("cruise", value[0], value[1])


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
roll_211_maneuver_indices = np.arange(37, 57)
roll_211_nt_maneuver_indices = np.arange(57, 100)
yaw_211_nt_maneuver_indices = np.arange(113, 149)
sweep_maneuver_indices = np.arange(177, 192)

# Correct pitch maneuver times
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
    pitch_211_maneuver_times,
    "pitch_211",
)

set_maneuver_times(
    exp3,
    pitch_211_nt_maneuver_indices,
    pitch_211_nt_maneuver_times,
    "pitch_211_no_throttle",
)


# Correct roll maneuver times
roll_211_maneuver_times = {
    37: [1347, 1351],
    38: [-1, 1356],
    39: [1355.5, 1360],
    40: [1359, 1363],
    41: [-1, 1379],  # Remove 42
    43: [1385.5, 1390],  # remove other exp and dropout
    44: [1389, 1394],
    45: [1393, 1397.4],
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
    roll_211_nt_maneuver_times,
    "roll_211_no_throttle",
)


# Correct yaw maneuver times
yaw_211_maneuver_times = {
    102: [-1, 2225], # maneuver 3
    104: [-1, 2249.5], # maneuver 5
    104: [-1, 2249.5],
    123: [-1, 3071],
}
yaw_211_maneuver_indices = np.hstack((np.arange(100, 113), np.arange(149, 177)))

set_maneuver_times(exp3, yaw_211_maneuver_indices, yaw_211_maneuver_times, "yaw_211")

yaw_211_nt_maneuver_times = {}

set_maneuver_times(
    exp3,
    yaw_211_nt_maneuver_indices,
    yaw_211_nt_maneuver_times,
    "yaw_211_no_throttle",
)

sweep_maneuver_times = {}
set_maneuver_times(exp3, sweep_maneuver_indices, sweep_maneuver_times, "sweep")


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
# Experiment 4
##############

exp4 = {}
exp4["LogName"] = "2021_04_18_flight_1_static_curves"
exp4["Number"] = 4
exp4["Maneuvers"] = {}
sweep_maneuver_times = {}

# Correct sweep times
# Do not use these, as they are with throttle
# sweep_maneuver_indices = np.arange(1, 32)
# set_maneuver_times(exp4, sweep_maneuver_indices, sweep_maneuver_times, [], "sweep")


##############
# Experiment 5
##############

exp5 = {}
exp5["LogName"] = "2021_04_18_flight_2_static_curves_no_thrust_211_roll"
exp5["Number"] = 5
exp5["Maneuvers"] = {}

sweep_maneuver_indices = np.arange(1, 32)
set_maneuver_times(exp5, sweep_maneuver_indices, {}, "sweep")

roll_211_maneuver_indices = np.arange(32, 76)
roll_maneuver_times = {
    41: [-1, 927],  # maneuver 29
    43: [940.5, -1],
    51: [-1, 1011],
    52: [-1, 1028.5],
    53: [1028, -1],
    55: [-1, 1068.5],
    58: [-1, 1098.5],
    59: [-1, 1102],
    60: [1101, -1],
}
set_maneuver_times(exp5, roll_211_maneuver_indices, roll_maneuver_times, "roll_211")

##############
# Experiment 6
##############

exp6 = {}
exp6["LogName"] = "2021_3_25_freehand_2_1_1_maneuvers"
exp6["Number"] = 6
exp6["Maneuvers"] = {}
roll_maneuver_indices = np.arange(1, 25)
pitch_maneuver_indices = np.arange(25, 53)
yaw_maneuver_indices = np.arange(53, 66)
set_maneuver_times(exp6, roll_maneuver_indices, {}, "roll_211")
set_maneuver_times(exp6, pitch_maneuver_indices, {}, "pitch_211")
set_maneuver_times(exp6, yaw_maneuver_indices, {}, "yaw_211")


##################

# Create metadata object
breakpoint()
metadata = {"Experiments": [exp2, exp3]}

with open("../data/flight_data/metadata.json", "w") as outfile:
    json.dump(metadata, outfile, indent=4, sort_keys=True)
