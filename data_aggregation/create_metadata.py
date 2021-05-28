import json
import numpy as np


def create_maneuver_dict(maneuver_type="not_set", maneuver_start=-1, maneuver_end=-1):
    d = {
        "type": maneuver_type,
        "start_s": maneuver_start,
        "end_s": maneuver_end,
    }
    return d


def set_maneuver_times(exp_dict, maneuver_indices, maneuver_times, maneuver_type):
    for maneuver_i in maneuver_indices:
        if str(maneuver_i) in maneuver_times:
            times = maneuver_times[str(maneuver_i)]
            exp_dict["Maneuvers"][str(maneuver_i)] = create_maneuver_dict(
                maneuver_type, times[0], times[1]
            )
        else:
            exp_dict["Maneuvers"][str(maneuver_i)] = create_maneuver_dict(maneuver_type)


# A helper script that generates metadata for the processing the obtained logs.
# The metadata is used by data_handler.m to know where to look for what data.


##############
# Experiment 1
##############

exp1 = {}
exp1["LogName"] = "06_31_21"
exp1["Number"] = 1
exp1["Maneuvers"] = {}

sweep_maneuver_indices = np.arange(1, 32)
sweep_maneuver_times = {
    "1": [0, 0],
    "2": [687, 689],
    "3": [747, 749],
    "4": [809, 812],
    "5": [869, 871],  # a bit unclear signal
    "6": [924.5, 928],
    "7": [994, 997],  # unclear signal
    "8": [1065, 1067],  # logging dropout after 1068
    "9": [1129, 1132],
    "10": [0, 0],  # high disturbances
    "11": [0, 0],  # high disturbance
    "12": [0, 0],  # high disturbance
    "13": [0, 0],  # unrealistically high c_L and high disturbance
    "14": [0, 0],  # high disturbance
    "15": [0, 0],  # high disturbance
    "16": [0, 0],  # high disturbance
    "17": [0, 0],  # dropout
    "18": [0, 0],  # high disturbance
    "19": [0, 0],  # high disturbance
    "20": [0, 0],  # high disturbance
    "21": [0, 0],  # dropout
    "22": [0, 0],  # high disturbance
    "23": [0, 0],  # high spread
    "24": [0, 0],  # high spread
    "25": [0, 0],  # high disturbance
    "26": [0, 0],  # high disturbance
    "27": [0, 0],  # high disturbance
    "28": [0, 0],  # high disturbance
    "29": [0, 0],  # high disturbance
    "30": [0, 0],  # high disturbance
    "31": [0, 0],  # high disturbance
}
set_maneuver_times(exp1, sweep_maneuver_indices, sweep_maneuver_times, "sweep")


##############
# Experiment 2
##############

exp2 = {}
exp2["LogName"] = "07_12_32"
exp2["Number"] = 2
exp2["Maneuvers"] = {}

pitch_211_maneuver_indices = np.arange(1, 18)
pitch_211_maneuver_times = {
    "7": [0, 0],  # dropout
    "8": [0, 0],  # not still input
}
set_maneuver_times(
    exp2, pitch_211_maneuver_indices, pitch_211_maneuver_times, "pitch_211"
)

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
    "1": [0, 0],  # disturbance
    "2": [891, -1],  # disturbance
    "8": [0, 0],  # dropout
    "13": [0, 0],  # disturbance
    "16": [0, 0],  # disturbance
    "18": [0, 0],  # dropout
}
pitch_211_nt_maneuver_times = {
    "23": [-1, 1080.7],  # throttle
    "24": [1107, 1109.8],  # throttle
    "25": [-1, 1117],  # throttle
    "26": [-1, 1137],  # throttle
    "27": [-1, 1142.7],  # throttle
    "28": [1145, 1148],  # throttle
    "29": [1163, -1],  # throttle
    "30": [-1, 1188],  # throttle
    "31": [0, 0],  # throttle
    "32": [0, 0],  # aborted
    "33": [-1, 1231],  # throttle
    "34": [1234.5, 1238],  # throttle
    "35": [0, 0],  # throttle
    "36": [-1, 1265.5],  # throttle
}
set_maneuver_times(
    exp3, pitch_211_maneuver_indices, pitch_211_maneuver_times, "pitch_211"
)
set_maneuver_times(
    exp3,
    pitch_211_nt_maneuver_indices,
    pitch_211_nt_maneuver_times,
    "pitch_211_no_throttle",
)


# Correct roll maneuver times
roll_211_maneuver_times = {
    "38": [1352.3, 1356.5],  # nudge in input
    "40": [-1, 1362.7],  # nudge in input
    "47": [0, 0],  # not still
    "48": [0, 0],  # not still
    "50": [1460.4, -1],  # not still
    "50": [-1, 1469],  # not still
    "56": [0, 0],  # dropout
}
roll_211_nt_maneuver_times = {
    "58": [-1, 1548.7],  # throttle
    "59": [1550.8, -1],  # throttle
    "60": [-1, 1580.3],  # throttle
    "61": [-1, 1586.3],  # throttle
    "62": [-1, 1591.7],  # throttle
    "63": [-1, 1608.5],  # throttle
    "64": [1610.5, 1613.5],  # throttle
    "65": [1610.5, 1619.2],  # throttle
    "66": [1636.6, -1],  # throttle
    "67": [1645.4, -1],  # throttle
    "68": [0, 0],  # dropout
    "69": [-1, 1875.3],  # throttle
    "70": [1679, -1],  # throttle
    "71": [1699.5, 1702.8],  # throttle
    "72": [1707.4, -1],  # throttle
    "73": [1722, 1725.3],  # throttle
    "74": [1730, 1733.4],  # throttle
    "75": [1737, 1740],  # throttle
    "77": [0, 0],  # too early throttle
    "78": [0, 0],  # aborted
    "79": [1790.5, 1793],  # throttle
    "80": [0, 0],  # throttle
    "81": [1817.5, -1],  # throttle
    "82": [0, 0],  # dropout
    "83": [1844.5, -1],  # throttle
    "84": [1854.5, -1],  # throttle
    "85": [0, 0],  # disturbance
    "86": [0, 0],  # throttle
    "90": [-1, 1951.5],  # throttle
    "91": [0, 0],  # aborted
    "93": [-1, 1987],  # throttle
    "94": [-1, 1995.5],  # throttle
    "95": [-1, 2014],  # throttle
    "98": [2046.7, -1],  # throttle
}
set_maneuver_times(exp3, roll_211_maneuver_indices, roll_211_maneuver_times, "roll_211")
set_maneuver_times(
    exp3,
    roll_211_nt_maneuver_indices,
    roll_211_nt_maneuver_times,
    "roll_211_no_throttle",
)


# Correct yaw maneuver times
yaw_211_maneuver_times = {
    "100": [0, 0],  # disturbance
    "101": [0, 0],  # dropout
    "103": [0, 0],  # disturbance
    "105": [0, 0],  # disturbance
    "106": [0, 0],  # disturbance
    "107": [2272.2, -1],  # disturbance
    "108": [2294, -1],  # disturbance
    "109": [2302, -1],  # disturbance
    "151": [0, 0],  # disturbance
    "162": [0, 0],  # disturbance
    "167": [0, 0],  # disturbance
    "171": [0, 0],  # disturbance
}
yaw_211_nt_maneuver_times = {
    "113": [-1, 2358.5],  # throttle
    "114": [0, 0],  # throttle
    "115": [2391, 2396],  # throttle
    "116": [-1, 2406.9],  # throttle
    "117": [0, 0],  # throttle
    "118": [0, 0],  # dropout
    "119": [0, 0],  # throttle
    "120": [2472.3, -1],  # throttle
    "121": [2485, -1],  # throttle
    "122": [-1, 2508.7],  # throttle
    "123": [-1, 2518.5],  # throttle
    "123": [0, 0],  # throttle
    "124": [-1, 2536.5],  # throttle
    "126": [0, 0],  # throttle
    "127": [-1, 2591],  # throttle
    "128": [0, 0],  # aborted
    "129": [0, 0],  # throttle
    "132": [0, 0],  # aborted
    "136": [0, 0],  # aborted
    "138": [0, 0],  # throttle
    "140": [-1, 2868],  # throttle
    "143": [-1, -1],  # throttle
}
set_maneuver_times(exp3, yaw_211_maneuver_indices, yaw_211_maneuver_times, "yaw_211")
set_maneuver_times(
    exp3,
    yaw_211_nt_maneuver_indices,
    yaw_211_nt_maneuver_times,
    "yaw_211_no_throttle",
)

# Correct sweep times
sweep_maneuver_times = {
    "177": [3543, 3546],
    "178": [0, 0],  # high spread in drag
    "179": [3653, 3655],
    "180": [0, 0],  # unrealistically high c_L value
    "181": [3728, 3731],
    "182": [3760, 3762],
    "183": [3801.5, 3804],
    "184": [0, 0],  # high spread
    "185": [0, 0],  # high spread
    "186": [0, 0],  # high spread
    "187": [3931, 3933],  # a bit noisy
    "188": [0, 0],  # high spread
    "189": [3992, 3993.2],
    "190": [0, 0],  # something very strange going on
    "191": [0, 0],  # same
}
set_maneuver_times(exp3, sweep_maneuver_indices, sweep_maneuver_times, "sweep")

# Add freehand times
exp3_freehand_times = {
    "192": [4070, 4120],
    "193": [4180, 4325],
}
for key, value in exp3_freehand_times.items():
    exp3["Maneuvers"][key] = create_maneuver_dict("freehand", value[0], value[1])


# Create metadata object
metadata = {"Experiments": [exp1, exp2, exp3]}

with open("../data/metadata.json", "w") as outfile:
    json.dump(metadata, outfile, indent=4, sort_keys=True)
