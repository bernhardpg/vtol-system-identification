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
        if str(maneuver_i) in maneuver_times:
            times = maneuver_times[str(maneuver_i)]
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
set_maneuver_times(exp1, sweep_maneuver_indices, {}, maneuvers_w_dropout, "sweep")


##############
# Experiment 2
##############

exp2 = {}
exp2["LogName"] = "07_12_32"
exp2["Number"] = 2
exp2["Maneuvers"] = {}

pitch_211_maneuver_indices = np.arange(1, 18)
pitch_211_skip = [7, 8, 11, 17]
set_maneuver_times(exp2, pitch_211_maneuver_indices, {}, pitch_211_skip, "pitch_211")

# Pretend freehand times are the last maneuvers, just for simplicity
# Does not make a difference for anything
exp2_freehand_times = {"18": [180, 520]}
for key, value in exp2_freehand_times.items():
    exp2["Maneuvers"][key] = create_maneuver_dict("freehand", value[0], value[1])


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
    "2": [891, -1],  # disturbance
}
pitch_211_skip = [
    1,  # disturbance
    4,  # dropout
    8,  # dropout
    13,  # disturbance
    16,  # disturbance
    18,  # dropout
]

pitch_211_nt_maneuver_times = {
    "22": [-1, 1077],  # not still input
    "23": [-1, 1080.7],  # throttle
    "24": [1107, 1109.8],  # throttle
    "25": [-1, 1117],  # throttle
    "26": [-1, 1137],  # throttle
    "27": [-1, 1142.7],  # throttle
    "28": [1145, 1148],  # throttle
    "29": [1163, 1167.5],  # throttle
    "30": [-1, 1188],  # throttle
    "33": [-1, 1231],  # throttle
    "34": [1234.5, 1238],  # throttle
    "36": [-1, 1265.5],  # throttle
}
pitch_211_nt_skip = [
    31,  # throttle
    32,  # aborted
    35,  # throttle
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
    "38": [1352.3, 1356.5],  # nudge in input
    "39": [-1, 1360],  # nudge in input
    "40": [-1, 1362.7],  # nudge in input
    "41": [-1, 1379.2],  # nudge in input
    "43": [-1, 1390.5],  # nudge in input
    "44": [-1, 1394],  # nudge in input
    "45": [-1, 1397],  # nudge in input
    "50": [1460.4, 1466],  # not still
    "52": [-1, 1475],  # nudge in input
}

roll_211_skip = [
    37,  # nudge in input can use this
    47,  # not still
    48,  # not still
    56,  # dropout
]

set_maneuver_times(
    exp3, roll_211_maneuver_indices, roll_211_maneuver_times, roll_211_skip, "roll_211"
)

roll_211_nt_maneuver_times = {
    "57": [-1, 1546],  # nudge in input
    "58": [-1, 1548.7],  # throttle
    "59": [1550.8, 1554.5],  # throttle
    "60": [1577, 1580.3],  # throttle
    "61": [-1, 1586.3],  # throttle
    "62": [-1, 1591.7],  # throttle
    "63": [-1, 1608.5],  # throttle
    "64": [1610.5, 1613.5],  # throttle
    "65": [1616.5, 1619.2],  # throttle
    "66": [1636.6, 1640.5],  # throttle
    "67": [1645.4, 1649],  # throttle
    "69": [-1, 1675.3],  # throttle
    "70": [1679, -1],  # throttle
    "71": [1699.8, 1702.8],  # throttle
    "72": [1707.4, 1711],  # throttle
    "73": [1722, 1725.3],  # throttle
    "74": [1730, 1733.4],  # throttle
    "75": [1737, 1740],  # throttle
    "76": [-1, 1758.7],  # throttle
    "81": [1817.5, 1821],  # throttle
    "83": [1844.5, 1848.2],  # throttle
    "84": [1854.5, 1858],  # throttle
    "87": [-1, 1903.7],  # throttle
    "88": [-1, 1918],  # throttle
    "89": [-1, 1941],  # throttle
    "90": [-1, 1951.5],  # throttle
    "92": [-1, 1977.8],  # throttle
    "93": [-1, 1987],  # throttle
    "95": [-1, 2014],  # throttle
    "96": [-1, 2023],  # throttle
    "97": [-1, 2035],  # throttle
    "98": [2046.7, 2050.4],  # throttle
    "99": [-1, 2064.9],  # throttle
}

roll_211_nt_skip = [
    68,  # dropout
    77,  # to early throttle
    78,  # aborted,
    79,  # throttle,
    80,  # throttle
    82,  # dropout
    85,  # disturbance
    86,  # throttle
    91,  # aborted
    94,  # throttle
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
    "109": [2302, -1],  # disturbance
}

yaw_211_skip = [
    100,  # disturbance
    101,  # dropout
    103,  # disturbance
    105,  # disturbance
    106,  # disturbance
    107,  # dropout
    108,  # dropout
    151,  # disturbance
    162,  # disturbance
    167,  # disturbance
    171,  # disturbance
]
set_maneuver_times(
    exp3, yaw_211_maneuver_indices, yaw_211_maneuver_times, yaw_211_skip, "yaw_211"
)

yaw_211_nt_maneuver_times = {
    "113": [-1, 2368.5],  # throttle
    "115": [2391, 2396],  # throttle
    "116": [-1, 2406.9],  # throttle
    "120": [2472.3, -1],  # throttle
    "121": [2485, -1],  # throttle
    "122": [-1, 2508.7],  # throttle
    "123": [-1, 2518.5],  # throttle
    "124": [-1, 2536.5],  # throttle
    "127": [-1, 2591],  # throttle
    "140": [-1, 2868],  # throttle
    "143": [-1, -1],  # throttle
}
yaw_211_nt_skip = [
    114,  # throttle
    117,  # throttle
    118,  # dropout
    119,  # throttle
    123,  # throttle
    126,  # throttle
    128,  # aborted
    129,  # throttle
    132,  # aborted
    136,  # aborted
    138,  # throttle
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
    "192": [4070, 4120],
    "193": [4180, 4325],
}
for key, value in exp3_freehand_times.items():
    exp3["Maneuvers"][key] = create_maneuver_dict("freehand", value[0], value[1])


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
    12, # something strange with drag
    13, # something strange with drag
]
set_maneuver_times(exp5, sweep_maneuver_indices, sweep_maneuver_times, skip, "sweep")


##################

# Create metadata object
metadata = {"Experiments": [exp1, exp2, exp3, exp4, exp5], "dt": 0.01}

with open("../data/metadata.json", "w") as outfile:
    json.dump(metadata, outfile, indent=4, sort_keys=True)
