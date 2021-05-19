import json
import numpy as np


def create_maneuver_dict(maneuver_type="not_set", maneuver_start=-1, maneuver_end=-1):
    d = {
        "type": maneuver_type,
        "start_s": maneuver_start,
        "end_s": maneuver_end,
    }
    return d


# Static curves of exp 1

exp1 = {}
exp1["LogName"] = "06_31_21"
exp1["Number"] = 1
exp1["Maneuvers"] = {str(i): create_maneuver_dict("sweep") for i in range(1, 32)}

exp1_static_times = {
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
    "23": [0, 0], # high spread
    "24": [0, 0], # high spread
    "25": [0, 0],  # high disturbance
    "26": [0, 0],  # high disturbance
    "27": [0, 0],  # high disturbance
    "28": [0, 0],  # high disturbance
    "29": [0, 0], # high disturbance
    "30": [0, 0], # high disturbance
    "31": [0, 0], # high disturbance
}

for key, value in exp1_static_times.items():
    exp1["Maneuvers"][key] = create_maneuver_dict("sweep", value[0], value[1])

exp2 = {}
exp2["LogName"] = "07_24_19"
exp2["Number"] = 2
exp2["Maneuvers"] = {str(i): create_maneuver_dict() for i in range(1, 192)}

exp2_static_times = {
    "177": [3543, 3546],
    "178": [0, 0], # high spread in drag
    "179": [3653, 3655],
    "180": [0, 0], # unrealistically high c_L value
    "181": [3728, 3731],
    "182": [3760, 3762],
    "183": [3801.5, 3804],
    "184": [0, 0], # high spread
    "185": [0, 0], # high spread
    "186": [0, 0], # high spread
    "187": [3931, 3933], # a bit noisy
    "188": [0, 0], # high spread
    "189": [3992, 3993.2],
    "190": [0, 0], # something very strange going on
    "191": [0, 0], # same
}

for key, value in exp2_static_times.items():
    exp2["Maneuvers"][key] = create_maneuver_dict("sweep", value[0], value[1])

# Longitudinal dynamic experiments in exp 2
pitch_211 = np.arange(1, 22)
pitch_211_no_throttle_indices = np.arange(22, 37)
roll_211 = np.arange(37, 57)
roll_211_no_throttle_indices = np.arange(57, 100)
yaw_211 = np.hstack((np.arange(100, 113), np.arange(149, 177)))
yaw_211_no_throttle_indices = np.arange(113, 149)

pitch_211_nt_times = {
    "11": [0, 0],
}


for i in pitch_211:
    exp2["Maneuvers"][str(i)] = create_maneuver_dict("pitch_211")
for i in range(len(pitch_211_no_throttle_indices)):
    if str(i) in pitch_211_nt_times:
        times = pitch_211_nt_times[str(i)]
        exp2["Maneuvers"][str(pitch_211_no_throttle_indices[i])] = create_maneuver_dict(
            "pitch_211_no_throttle", times[0], times[1]
        )
    else:
        exp2["Maneuvers"][str(pitch_211_no_throttle_indices[i])] = create_maneuver_dict(
            "pitch_211_no_throttle"
        )


for i in roll_211:
    exp2["Maneuvers"][str(i)] = create_maneuver_dict("roll_211")
for i in roll_211_no_throttle_indices:
    exp2["Maneuvers"][str(i)] = create_maneuver_dict("roll_211_no_throttle")

for i in yaw_211:
    exp2["Maneuvers"][str(i)] = create_maneuver_dict("yaw_211")
for i in yaw_211_no_throttle_indices:
    exp2["Maneuvers"][str(i)] = create_maneuver_dict("yaw_211_no_throttle")

metadata = {"Experiments": [exp1, exp2]}

with open("../data/metadata.json", "w") as outfile:
    json.dump(metadata, outfile, indent=4, sort_keys=True)
