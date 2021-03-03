#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
import argparse
import pdb
import pandas as pd
import os

from vehicle_cmd_trans import vehicle_cmd_trans


def parse_vehicle_cmd(key):
    key = str(int(key))
    if not key in vehicle_cmd_trans.keys():
        print("vehicle_cmd: " + key + " not found")
        return

    return vehicle_cmd_trans[str(key)]


def chunks(lst, n):
    """Yield successive n-sized chunks from lst."""
    for i in range(0, len(lst), n):
        yield lst[i : i + n]


def plot_df(log_name, df, keys=[]):
    plot_all = len(keys) == 0
    if plot_all:  # Divide keys up into groups of 5
        keys = list(df.keys())
        keys.remove("timestamp")

    key_groups = list(chunks(keys, N_PLOTS_ON_A_PAGE))

    if not key_groups[0][0] in df.keys():  # We have the wrong dataframe
        return

    # Handle time axis
    t = df["timestamp"] / 1e6
    t = t - t[0]
    t_min = min(t)
    t_max = max(t)

    for key_group in key_groups:
        num_plots = len(key_group)
        fig, axs = plt.subplots(num_plots, 1)
        if not type(axs) == type(np.array([])):  # Make sure axs is a list
            axs = [axs]
        fig.suptitle(log_name, fontsize=16)

        i = 0
        for key in key_group:
            y_max = max(df[key])
            y_min = min(df[key])
            if y_max > 0:
                axs[i].set_ylim(y_min * 1.3, y_max * 1.3)
            breakpoint()
            axs[i].plot(t, df[key])
            axs[i].set_xlim(t_min, t_max)
            axs[i].set_xlabel("time [s]")
            axs[i].set_title(str(key))
            axs[i].grid()
            fig.tight_layout()
            i += 1
    return


def read_csv_file(filename):
    df = pd.read_csv(filename)  # Use panda dataframes
    return df


# Parse file name
parser = argparse.ArgumentParser(description="Show log data")
parser.add_argument("log_file", help="file path for .csv file")
parser.add_argument("-m", "--message", type=str, help="which message to plot")
parser.add_argument("-n", "--num_plots", type=int, help="number of plots on a page")
parser.add_argument(
    "-t", "--topics", nargs="+", type=str, help="specific topics to plot"
)
args = parser.parse_args()

log_file = args.log_file
desired_msg = args.message
N_PLOTS_ON_A_PAGE = args.num_plots or 3
keys = args.topics or []

# Read desired message files
os.system("ulog2csv " + log_file + " -o temp")
# ulog2csv creates an individual .csv file for each msg topic
msg_log_files = os.listdir("temp")
print("All available log files")
for msg_log_file in msg_log_files:
    print(msg_log_file)
print("\n")

logs = []
for msg_log in msg_log_files:
    if desired_msg in msg_log:
        log_df = read_csv_file("temp/" + msg_log)
        logs.append((msg_log, log_df))
        # Display keys in log
        print("Keys in " + msg_log + ":")
        for key in log_df.keys():
            print(key)
        print("\n")

if len(logs) == 0:
    print("No logs named " + desired_msg)

# Custom plotting for desired topics
keys = []
if desired_msg == "battery":
    keys = [
        "voltage_v",
        "warning",
        "capacity",
        "remaining",
    ]
if desired_msg == "position_setpoint_triplet":
    keys = [
        "previous.valid",
        "previous.alt",
        "previous.lon",
        "previous.lat",
        "current.valid",
        "current.alt",
        "current.lon",
        "current.lat",
        "next.valid",
        "next.alt",
        "next.lon",
        "next.lat",
    ]

if desired_msg == "vehicle_command":
    print("{:>12}: {:>12}".format("timestamp", "command"))
    t0 = log_df["timestamp"][0]
    for index, message in log_df.iterrows():
        timestamp = (message["timestamp"] - t0) / 1e6
        command = parse_vehicle_cmd(message["command"])
        print("{:>12}: {:>12}".format(timestamp, command))

# Plot all topics in msg
for log_name, log_df in logs:
    plot_df(log_name, log_df, keys)


plt.show()
# os.system("rm -rf temp/*")
