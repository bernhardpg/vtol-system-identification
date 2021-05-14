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
exp1["Maneuvers"] = {str(i): create_maneuver_dict() for i in range(1, 32)}

exp1_static_times = {
    "1": [0, 0],
    "2": [687.3, 688],
    "3": [747, 747.7],
    "4": [809.5, 810.3],
    "5": [0, 0],
    "6": [925, 926],
    "7": [0, 0],  # too high c_L
    "8": [0, 0],
    "9": [1129.5, 1131],
    "10": [0, 0],
    "11": [0, 0],
    "12": [0, 0],  # too high c_L
    "13": [0, 0],
    "14": [0, 0],
    "15": [0, 0],
    "16": [0, 0],
    "17": [0, 0],
    "18": [0, 0],
    "19": [0, 0],
    "20": [0, 0],
    "21": [0, 0],
    "22": [0, 0],
    "23": [1838.7, 1839.6],
    "24": [1894.2, 1896],
    "25": [1950.3, 1950.6],
    "26": [0, 0],
    "27": [0, 0],
    "28": [0, 0],
    "29": [2158.5, 2159.4],
    "30": [2216.5, 2217.6],
    "31": [2277, 2278],
}


for key, value in exp1_static_times.items():
    exp1["Maneuvers"][key] = create_maneuver_dict("sweep", value[0], value[1])


exp2 = {}
exp2["LogName"] = "07_24_19"
exp2["Number"] = 2
exp2["Maneuvers"] = {str(i): create_maneuver_dict() for i in range(1, 192)}

# Static curves of exp 2
exp2_static_times = {
    "177": [3543.5, 3545],
    "178": [0, 0],
    "179": [3653, 3654.5],
    "180": [0, 0],
    "181": [3728, 3730],
    "182": [0, 0],
    "183": [3801.3, 3802.6],
    "184": [0, 0],
    "185": [3872, 3873.5],
    "186": [0, 0],
    "187": [0, 0],
    "188": [0, 0],
    "189": [3992, 3993.2],
    "190": [0, 0],
    "191": [0, 0],
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

for i in pitch_211:
    exp2["Maneuvers"][str(i)] = create_maneuver_dict("pitch_211")
for i in pitch_211_no_throttle_indices:
    exp2["Maneuvers"][str(i)] = create_maneuver_dict("pitch_211_no_throttle")

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
