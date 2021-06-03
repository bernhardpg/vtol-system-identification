import json
import numpy as np


# A helper script that generates metadata for the processing the obtained logs.
# The metadata is used by data_handler.m to know where to look for what data.


def create_maneuver_dict(
    maneuver_type="not_set", maneuver_start=-1, maneuver_end=-1, skip_maneuver=False
):
    d = {
        "type": maneuver_type,
        "start_s": maneuver_start,
        "end_s": maneuver_end,
        "skip": skip_maneuver,
    }
    return d


def set_maneuver_times(
    exp_dict, maneuver_indices, maneuver_times, maneuvers_to_skip, maneuver_type
):
    for maneuver_i in maneuver_indices:
        start_time = -1
        end_time = -1
        skip = False
        if maneuver_i in maneuver_times:
            times = maneuver_times[maneuver_i]
            start_time = times[0]
            end_time = times[1]
        if maneuver_i in maneuvers_to_skip:
            skip = True

        exp_dict["Maneuvers"][str(maneuver_i)] = create_maneuver_dict(
            maneuver_type, start_time, end_time, skip
        )
    return


##############
# Experiment 1
##############

exp1 = {}
exp1["LogName"] = "06_31_21"
exp1["Number"] = 1
exp1["Maneuvers"] = {}

sweep_maneuver_indices = np.arange(1, 32)
maneuvers_w_dropout = [1, 8, 17, 21]
skip = [2]  # too close to other maneuvers
set_maneuver_times(
    exp1, sweep_maneuver_indices, {}, maneuvers_w_dropout + skip, "sweep"
)

# Pick some random cruise flight times
exp1_cruise_times = {
    31: [640, 650],
    32: [650, 670],
    33: [720, 720],
    34: [770, 790],
    35: [900, 920],
    36: [1160, 1180],
    37: [1970, 1990],
}
for key, value in exp1_cruise_times.items():
    exp1["Maneuvers"][str(key)] = create_maneuver_dict("not_set", value[0], value[1])


##############
# Experiment 2
##############

exp2 = {}
exp2["LogName"] = "07_12_32"
exp2["Number"] = 2
exp2["Maneuvers"] = {}

pitch_211_maneuver_indices = np.arange(1, 18)
dropouts = [1, 2, 7, 11, 17]
set_maneuver_times(exp2, pitch_211_maneuver_indices, {}, dropouts, "pitch_211")

# Pretend freehand times are the last maneuvers, just for simplicity
# Does not make a difference for anything
exp2_freehand_times = {"18": [180, 520]}
for key, value in exp2_freehand_times.items():
    exp2["Maneuvers"][str(key)] = create_maneuver_dict("freehand", value[0], value[1])


##############
# Experiment 3
##############

# Contains:
# 2-1-1 on pitch
# 2-1-1 on pitch without thrust
# 2-1-1 on roll
# 2-1-1 on roll without thrust
# 2-1-1 on yaw
# 2-1-1 on yaw without thrust
# sweeps on pitch

exp3 = {}
exp3["LogName"] = "07_24_19"
exp3["Number"] = 3
exp3["Maneuvers"] = {}

# Indices for the different 2-1-1 maneuvers
pitch_211_maneuver_indices = np.arange(1, 22)
pitch_211_nt_maneuver_indices = np.arange(22, 37)
roll_211_maneuver_indices = np.arange(37, 57)
roll_211_nt_maneuver_indices = np.arange(57, 100)
yaw_211_maneuver_indices = np.hstack((np.arange(100, 113), np.arange(149, 177)))
yaw_211_nt_maneuver_indices = np.arange(113, 149)
sweep_maneuver_indices = np.arange(177, 192)

# Correct pitch maneuver times
pitch_211_maneuver_times = {
    9: [-1, 981],  # split maneuvers
    10: [-1, 986],  # split maneuvers
    11: [-1, 991],  # split maneuvers
    13: [-1, 1011],  # split maneuvers
    14: [-1, 1016],  # split maneuvers
    15: [-1, 1020.5],  # split maneuvers
    16: [-1, 1025],  # split maneuvers
    17: [-1, 1030],  # split maneuvers
    19: [-1, 1051],  # split maneuvers
    20: [-1, 1061],  # use two maneuvers in one
}
pitch_211_skip = [
    4,  # dropout
    8,  # dropout
    18,  # dropout
    21,  # used in 20
]

pitch_211_nt_maneuver_times = {
    22: [-1, 1083],  # take both 22 and 23
    27: [-1, 1150],  # take both 27 and 28
}
pitch_211_nt_skip = [
    23,  # already used
    32,  # dropout
]
set_maneuver_times(
    exp3,
    pitch_211_maneuver_indices,
    pitch_211_maneuver_times,
    pitch_211_skip,
    "pitch_211",
)

set_maneuver_times(
    exp3,
    pitch_211_nt_maneuver_indices,
    pitch_211_nt_maneuver_times,
    pitch_211_nt_skip,
    "pitch_211_no_throttle",
)


# Correct roll maneuver times
roll_211_maneuver_times = {
    37: [-1, 1365],  # Take both 37,38,39,40 in one as they are so close
    41: [-1, 1379],  # Remove 42
    43: [1385.5, 1390],  # remove other exp and dropout
    44: [1389, 1400],  # Take 44 and 45
    46: [-1, 1429.5],  # dropout
    47: [1432.5, 1443],  # 47 and 48
    49: [-1, 1466],  # 47 and 48
    51: [-1, 1477],  # 51 and 52
}

roll_211_skip = [
    42,  # dropout
    45,  # already used
    48,  # already used
    50,  # already used
    56,  # dropout
]

set_maneuver_times(
    exp3, roll_211_maneuver_indices, roll_211_maneuver_times, roll_211_skip, "roll_211"
)

roll_211_nt_maneuver_times = {
    57: [-1, 1551],  # 57 and 58
    61: [-1, 1595],  # 61 and 62
    76: [-1, 1764],  # 61 and 62
    92: [-1, 1990], # 92 and 93
}

roll_211_nt_skip = [
    58,  # already used
    61,  # already used
    68,  # dropout
    77, # already used
    78, # dropout
    82,  # dropout
    91, # dropout
]

set_maneuver_times(
    exp3,
    roll_211_nt_maneuver_indices,
    roll_211_nt_maneuver_times,
    roll_211_nt_skip,
    "roll_211_no_throttle",
)


# Correct yaw maneuver times
yaw_211_maneuver_times = {
    105: [-1, 2320],  # 105 - 110
}

yaw_211_skip = [
    101,  # dropout
    107,  # dropout
    108,  # dropout
]
set_maneuver_times(
    exp3, yaw_211_maneuver_indices, yaw_211_maneuver_times, yaw_211_skip, "yaw_211"
)

yaw_211_nt_maneuver_times = {}
yaw_211_nt_skip = [
    118,  # dropout
]

set_maneuver_times(
    exp3,
    yaw_211_nt_maneuver_indices,
    yaw_211_nt_maneuver_times,
    yaw_211_nt_skip,
    "yaw_211_no_throttle",
)

sweep_maneuver_times = {}
set_maneuver_times(exp3, sweep_maneuver_indices, sweep_maneuver_times, [], "sweep")


# Add freehand times
exp3_freehand_times = {
    192: [4070, 4120],
    193: [4180, 4325],
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

sweep_maneuver_times = {}

# Correct sweep times
sweep_maneuver_indices = np.arange(1, 32)
skip = [
    1,  # dropout
    10,  # dropout
    12,  # something strange with drag
    13,  # something strange with drag
]
set_maneuver_times(exp5, sweep_maneuver_indices, sweep_maneuver_times, skip, "sweep")


##################

# Create metadata object
breakpoint()
metadata = {"Experiments": [exp1, exp2, exp3, exp4, exp5], "dt": 0.01}

with open("../data/metadata.json", "w") as outfile:
    json.dump(metadata, outfile, indent=4, sort_keys=True)
