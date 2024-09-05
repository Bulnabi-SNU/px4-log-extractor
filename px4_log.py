import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os
import logging
from pyulog.ulog2csv import convert_ulog2csv # need to run the command 'pip install pyulog'
from pyulog import ULog
import yaml
from datetime import datetime, timedelta
import pymap3d as p3d

#def convert_ulog2csv(ulog_file_name, messages, output, delimiter, time_s, time_e,
#                     disable_str_exceptions=False):

#    Coverts and ULog file to a CSV file.

#    :param ulog_file_name: The ULog filename to open and read
#    :param messages: A list of message names -> 'sensor_combined,vehicle_gps_position'
#    :param output: Output file path
#    :param delimiter: CSV delimiter
#    :param time_s: Offset time for conversion in seconds
#    :param time_e: Limit until time for conversion in seconds
#
#    :return: None

#input file name
#ulogfilepath = '/home/gracekim/Downloads/log_334_2024-8-26-12-39-38.ulg'
#ulogfilename = 'log_334_2024-8-26-12-39-38.ulg'
ulogfilepath = input("enter file path: ")
ulogfilename = input("enter file name: ")

# auto latitude longitude altitude year month day hour min sec ms WPT
# auto: px4msgs : vehicle_status.nav_state / latitude longitude altitude : vehicle_gps_position
# utc year month day hour min sec ms WPT : vehicle_gps_position

messages_type = ['vehicle_status', 'vehicle_gps_position']

#input the directory address that you want to save the csv file.
#output_dir='/home/gracekim/px4_log/'
output_dir = input("enter directory path to save the csv file: ")

# Ensure the output directory exists
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

#convert_ulog2csv converts the ulog file to csv file
#At the message, input the values that you want to get, separating them by using commas
temp = convert_ulog2csv(ulog_file_name=ulogfilepath,
                        messages=','.join(messages_type),
                        output=output_dir,
                        delimiter=',',
                        time_s=0,
                        time_e=0,
                        disable_str_exceptions=False
                        )

for i in range(len(messages_type)):
    csv_filename = '_'.join([ulogfilename[:-4], messages_type[i], '0.csv'])
    csv_filename = ''.join([output_dir, csv_filename])
    if os.path.exists(csv_filename):
        globals()["temp{}".format(i)] = np.transpose(np.genfromtxt(csv_filename, dtype=str, delimiter=','))
        print(f"{csv_filename} found")
    else:
        print(f"{csv_filename} not found")


# logging setup
log_dir = output_dir
os.makedirs(log_dir, exist_ok=True)
log_file = os.path.join(log_dir, f'{ulogfilename[:-4]}.txt')
logging.basicConfig(filename=log_file, level=logging.INFO, format='%(message)s')
logger = logging.getLogger(__name__)

def log(*args, **kwargs) :
    print(*args, **kwargs)
    logger.info(*args, **kwargs)

# functions for phase detection
def find_keys_below_threshold(d, threshold):
    return [key for key, value in d.items() if value < threshold]

def find_keys_by_value(dictionary, target_value):
    # Using list comprehension to find all keys with the given target value
    return [key for key, value in dictionary.items() if value == target_value]

def intersection(arr1, arr2):
    return [x for x in arr1 if x in arr2]


# log info: auto latitude longitude altitude year month day hour min sec ms WPT(phase)
# initializing log info
gps_auto = []
latitude = []
longitude = []
altitude = []
utc_year = []
utc_month = []
utc_day = []
utc_hour = []
utc_minute = []
utc_sec = []
utc_ms = []

# vehicle_status
vehicle_status_dict = {}

for i in range(len(temp0)) :
    vehicle_status_dict[temp0[i][0]] = [int(x) for x in temp0[i][1:]]


# vehicle_gps_position
vehicle_gps_position_dict = {}

for i in range(len(temp1)) :
    vehicle_gps_position_dict[temp1[i][0]] = [float(x) for x in temp1[i][1:]]

# Getting Latitude / Longititude / Altitude data from vehicle_gps_position
for i in vehicle_gps_position_dict['latitude_deg'] :
    latitude.append(i)
for i in vehicle_gps_position_dict['longitude_deg'] :
    longitude.append(i)
for i in vehicle_gps_position_dict['altitude_msl_m'] :
    altitude.append(i)

# extract home position gps
home_position_gps = np.array([latitude[0], longitude[0], altitude[0]])

# convert gps position to local position
local_position = []
for i in range(len(vehicle_gps_position_dict['latitude_deg'])) :
   local_position.append(p3d.geodetic2ned(latitude[i], longitude[i], altitude[i], home_position_gps[0], home_position_gps[1], home_position_gps[2]))

# Getting time_utc_usec from vehicle_gps_position
time_utc_usec = []
utc_datetime = []

for utc_time in vehicle_gps_position_dict['time_utc_usec'] :
    utc_datetime.append(datetime(1970, 1, 1) + timedelta(microseconds=utc_time))

# Getting utc time info
for utc_datetime in utc_datetime :
    utc_year.append(utc_datetime.year)
    utc_month.append(utc_datetime.month)
    utc_day.append(utc_datetime.day)
    utc_hour.append(utc_datetime.hour)
    utc_minute.append(utc_datetime.minute)
    utc_sec.append(utc_datetime.second)
    utc_ms.append(utc_datetime.microsecond // 1000) # Convert microseconds to milliseconds


# Getting WP data from vehicle_gps_position
# WP gps location : taean_uv_land.yaml
# extract WP gps locations
gps_WP = {}
#yaml_path = '/home/gracekim/workspace/test_ws/src/vehicle_controller/config/taean_uv_land.yaml'
yaml_path = input("enter the path for WP location yaml file: ")
with open(yaml_path, 'r') as file :
    yaml_data = yaml.safe_load(file)
    for key, value in yaml_data['vehicle_controller']['ros__parameters'].items() :
        gps_WP[key] = value


# define local WP locations
WP = []
landing_height = 5.0
for i in range(len(gps_WP)) :
    wp_position = p3d.geodetic2ned(gps_WP[f"gps_WP{i+1}"][0], gps_WP[f"gps_WP{i+1}"][1], gps_WP[f"gps_WP{i+1}"][2] + home_position_gps[2], 
                               home_position_gps[0], home_position_gps[1], home_position_gps[2])
    wp_position = np.array(wp_position)
    WP.append(wp_position)

wp_position_8 = np.array([0.0 ,0.0, -landing_height]) # WP8: 5m above vertiport
wp_position_9 = np.array([0.0, 0.0, 0.0]) # WP9: vertiport

WP.append(wp_position_8)
WP.append(wp_position_9)
log(f'WP coordinates: {WP}\n')

# Getting gps_timestamp info
gps_timestamp = []
for stamp in vehicle_gps_position_dict['timestamp'] :
    gps_timestamp.append(int(stamp))

# Getting nav_state_timestamp info
nav_state_timestamp = []
for stamp in vehicle_status_dict['nav_state_timestamp'] :
    if stamp not in nav_state_timestamp :
        nav_state_timestamp.append(stamp)

# Getting corresponding nav_state info
nav_state = []
for state in vehicle_status_dict['nav_state'] :
    if state not in nav_state :
        nav_state.append(state)

# getting gps_auto info comparing gps_timestamp & nav_state_timestamp
# nav_state 3: auto mission mode, 4: auto loiter mode, 5: auto return to launch mode, 14: offboard, 17: auto takeoff, 18: auto land, 19: auto follow target, 20: auto precision land
auto_states = {3, 4, 5, 14, 17, 18, 19, 20}
for i in range(len(gps_timestamp)):
    gps_time = gps_timestamp[i]
    value_to_append = 0
    for j in range(len(nav_state_timestamp) - 1):
        if nav_state_timestamp[j] <= gps_time < nav_state_timestamp[j + 1]:
            if nav_state[j] in auto_states:
                value_to_append = 1
            else :
                value_to_append = 0
    gps_auto.append(value_to_append)


# takeoff: change phase to 1
takeoff_time = []
for time in vehicle_status_dict['takeoff_time'] :
    if (time not in takeoff_time) and (time != 0):
        takeoff_time.append(time)
takeoff_time = takeoff_time[0]

log(f"takeoff time: {takeoff_time}")

# detect phase change
nearby_acceptance_radius_mission = 30
nearby_acceptance_radius_landing = 3

# estimate horizontal error & vertical error
horizontal_error_dict = {}
vertical_error_dict = {}
error_dict = {}

# initialize phase info
phase = 0

# logging
# log frequency: 10Hz
log(f"\nA       B            C          D        E      F   G   H   I   J    K  L")

for i in range(len(gps_timestamp)):
    local_pos = np.array(local_position[i])
    waypoint = np.array(WP[phase-1]) # WP[0] is WP1 / ~ / WP[7] is 5m above vertiport / WP[8] is vertiport position
    
    horizontal_error = np.linalg.norm(local_pos[0:2] - waypoint[0:2])
    vertical_error = local_pos[2] - waypoint[2]
    error = np.linalg.norm(local_pos - waypoint)

    # takeoff: phase 0 -> 1
    if phase == 0 :
        if gps_timestamp[i] < takeoff_time :
            phase = 0
        elif gps_timestamp[i] >= takeoff_time :
            phase = 1
        log(f"{i}\t{gps_auto[i]}\t{latitude[i]:.6f}\t{longitude[i]:.6f}\t{(altitude[i]-home_position_gps[2]):.6f}\t{utc_year[i]}\t{utc_month[i]}\t{utc_day[i]}\t{utc_hour[i]}\t{utc_minute[i]}\t{utc_sec[i]}\t{utc_ms[i]}\t{phase}\n")

    # calculate phase by error info
    elif phase in range(1, 8):
        if np.linalg.norm(local_pos - waypoint) >= nearby_acceptance_radius_mission:
            if len(error_dict) == 0:
                log(f"{i}\t{gps_auto[i]}\t{latitude[i]:.6f}\t{longitude[i]:.6f}\t{(altitude[i]-home_position_gps[2]):.6f}\t{utc_year[i]}\t{utc_month[i]}\t{utc_day[i]}\t{utc_hour[i]}\t{utc_minute[i]}\t{utc_sec[i]}\t{utc_ms[i]}\t{phase}\n")
            
            elif len(error_dict) > 0:
                
                error_list = list(error_dict.values())
                error_indices = list(error_dict.keys())
                log(f"error_dict: {error_dict}")
                
                horizontal_min_indices = find_keys_below_threshold(horizontal_error_dict, 2)
                vertical_min_indices = find_keys_below_threshold(vertical_error_dict, 4)

                min_indices = intersection(horizontal_min_indices, vertical_min_indices)

                if min_indices != [] : 
                    log(f"min_indices (2,4): {min_indices}")
                    if len(min_indices) == 1 :
                        errors = [error_dict[min_indices[0]]]
                    else :
                        errors = []
                        for idx in min_indices :
                            errors.append(error_dict[idx])

                else :
                    horizontal_min_indices = find_keys_below_threshold(horizontal_error_dict, 4)
                    vertical_min_indices = find_keys_below_threshold(vertical_error_dict, 8)
                    min_indices = intersection(horizontal_min_indices, vertical_min_indices)
                    if min_indices != [] :
                        log(f"min_indices (4,8): {min_indices}")
                        if len(min_indices) == 1 :
                            errors = [error_dict[min_indices[0]]]
                        else :
                            errors = []
                            for idx in min_indices :
                                errors.append(error_dict[idx])

                    else :
                        horizontal_min_indices = find_keys_below_threshold(horizontal_error_dict, 6)
                        vertical_min_indices = find_keys_below_threshold(vertical_error_dict, 12)
                        min_indices = intersection(horizontal_min_indices, vertical_min_indices)
                        if min_indices != [] :
                            log(f"min_indices (6,12): {min_indices}")
                            if len(min_indices) == 1 :
                                errors = [error_dict[min_indices[0]]]
                            else :
                                errors = []
                                for idx in min_indices :
                                    errors.append(error_dict[idx])

                        else :
                            errors = error_list
                            log(f"min_indices: {min_indices}")
                            

                min_idx = find_keys_by_value(error_dict, min(errors))
                min_idx = min_idx[0]

                log(f"horizontal error dict: {horizontal_error_dict}")
                log(f"vertical error dict: {vertical_error_dict}")
                log(f"errors: {errors}")
                log(f"min idx: {min_idx}")
                log(f"error indices: {error_indices}")

                for idx in error_indices: 
                               
                    vertical_error = np.abs(vertical_error_dict[min_idx])
                    horizontal_error = np.abs(horizontal_error_dict[min_idx])
                    
                    if idx < min_idx:
                        log(f"{gps_auto[idx]}\t{latitude[idx]:.6f}\t{longitude[idx]:.6f}\t{(altitude[idx]-home_position_gps[2]):.6f}\t{utc_year[idx]}\t{utc_month[idx]}\t{utc_day[idx]}\t{utc_hour[idx]}\t{utc_minute[idx]}\t{utc_sec[idx]}\t{utc_ms[idx]}\t{phase}\n")
                    elif idx >= min_idx:
                        log(f"{gps_auto[idx]}\t{latitude[idx]:.6f}\t{longitude[idx]:.6f}\t{(altitude[idx]-home_position_gps[2]):.6f}\t{utc_year[idx]}\t{utc_month[idx]}\t{utc_day[idx]}\t{utc_hour[idx]}\t{utc_minute[idx]}\t{utc_sec[idx]}\t{utc_ms[idx]}\t{phase+1}\n")
                
                log(f"---------------------------------------------")
                log(f"vertical error: {vertical_error}, horizontal error: {horizontal_error}")
                log(f"---------------------------------------------\n")
                log(f"{gps_auto[i]}\t{latitude[i]:.6f}\t{longitude[i]:.6f}\t{(altitude[i]-home_position_gps[2]):.6f}\t{utc_year[i]}\t{utc_month[i]}\t{utc_day[i]}\t{utc_hour[i]}\t{utc_minute[i]}\t{utc_sec[i]}\t{utc_ms[i]}\t{phase+1}\n")
                phase += 1

            #horizontal_error_list = []
            #vertical_error_list = []
            horizontal_error_dict = {}
            vertical_error_dict = {}
            error_dict = {}

        elif np.linalg.norm(local_pos - waypoint) <= nearby_acceptance_radius_mission:
            #horizontal_error_list.append(horizontal_error)
            #vertical_error_list.append(vertical_error)
            horizontal_error_dict[i] = horizontal_error
            vertical_error_dict[i] = vertical_error
            error_dict[i] = error
    
    # calculate phase by error info
    elif phase == 8:
        if np.linalg.norm(local_pos - waypoint) >= nearby_acceptance_radius_landing:
            if len(error_dict) == 0:
                log(f"{gps_auto[i]}\t{latitude[i]:.6f}\t{longitude[i]:.6f}\t{(altitude[i]-home_position_gps[2]):.6f}\t{utc_year[i]}\t{utc_month[i]}\t{utc_day[i]}\t{utc_hour[i]}\t{utc_minute[i]}\t{utc_sec[i]}\t{utc_ms[i]}\t{phase}\n")
            
            elif len(error_dict) > 0:
                
                error_list = list(error_dict.values())
                error_indices = list(error_dict.keys())
                log(f"error_dict: {error_dict}")
                
                horizontal_min_indices = find_keys_below_threshold(horizontal_error_dict, 2)
                vertical_min_indices = find_keys_below_threshold(vertical_error_dict, 4)

                min_indices = intersection(horizontal_min_indices, vertical_min_indices)

                if min_indices != [] : 
                    log(f"min_indices (2,4): {min_indices}")
                    if len(min_indices) == 1 :
                        errors = [error_dict[min_indices[0]]]
                    else :
                        errors = []
                        for idx in min_indices :
                            errors.append(error_dict[idx])

                else :
                    horizontal_min_indices = find_keys_below_threshold(horizontal_error_dict, 4)
                    vertical_min_indices = find_keys_below_threshold(vertical_error_dict, 8)
                    min_indices = intersection(horizontal_min_indices, vertical_min_indices)
                    if min_indices != [] :
                        log(f"min_indices (4,8): {min_indices}")
                        if len(min_indices) == 1 :
                            errors = [error_dict[min_indices[0]]]
                        else :
                            errors = []
                            for idx in min_indices :
                                errors.append(error_dict[idx])

                    else :
                        horizontal_min_indices = find_keys_below_threshold(horizontal_error_dict, 6)
                        vertical_min_indices = find_keys_below_threshold(vertical_error_dict, 12)
                        min_indices = intersection(horizontal_min_indices, vertical_min_indices)
                        if min_indices != [] :
                            log(f"min_indices (6,12): {min_indices}")
                            if len(min_indices) == 1 :
                                errors = [error_dict[min_indices[0]]]
                            else :
                                errors = []
                                for idx in min_indices :
                                    errors.append(error_dict[idx])

                        else :
                            errors = error_list
                            log(f"min_indices: {min_indices}")

                min_idx = find_keys_by_value(error_dict, min(errors))
                min_idx = min_idx[0]

                log(f"horizontal error dict: {horizontal_error_dict}")
                log(f"vertical error dict: {vertical_error_dict}")
                log(f"errors: {errors}")
                log(f"min idx: {min_idx}")
                log(f"error indices: {error_indices}")

                for idx in error_indices: 
                               
                    vertical_error = np.abs(vertical_error_dict[min_idx])
                    horizontal_error = np.abs(horizontal_error_dict[min_idx])
                    
                    if idx < min_idx:
                        log(f"{gps_auto[idx]}\t{latitude[idx]:.6f}\t{longitude[idx]:.6f}\t{(altitude[idx]-home_position_gps[2]):.6f}\t{utc_year[idx]}\t{utc_month[idx]}\t{utc_day[idx]}\t{utc_hour[idx]}\t{utc_minute[idx]}\t{utc_sec[idx]}\t{utc_ms[idx]}\t{phase}\n")
                    elif idx >= min_idx:
                        log(f"{gps_auto[idx]}\t{latitude[idx]:.6f}\t{longitude[idx]:.6f}\t{(altitude[idx]-home_position_gps[2]):.6f}\t{utc_year[idx]}\t{utc_month[idx]}\t{utc_day[idx]}\t{utc_hour[idx]}\t{utc_minute[idx]}\t{utc_sec[idx]}\t{utc_ms[idx]}\t{phase+1}\n")

                
                log(f"---------------------------------------------")
                log(f"vertical error: {vertical_error}, horizontal error: {horizontal_error}")
                log(f"---------------------------------------------")
                log(f"{gps_auto[i]}\t{latitude[i]:.6f}\t{longitude[i]:.6f}\t{(altitude[i]-home_position_gps[2]):.6f}\t{utc_year[i]}\t{utc_month[i]}\t{utc_day[i]}\t{utc_hour[i]}\t{utc_minute[i]}\t{utc_sec[i]}\t{utc_ms[i]}\t{phase+1}\n")
                phase += 1

            #horizontal_error_list = []
            #vertical_error_list = []
            horizontal_error_dict = {}
            vertical_error_dict = {}
            error_dict = {}

        elif np.linalg.norm(local_pos - waypoint) <= nearby_acceptance_radius_landing:
            #horizontal_error_list.append(horizontal_error)
            #vertical_error_list.append(vertical_error)
            horizontal_error_dict[i] = horizontal_error
            vertical_error_dict[i] = vertical_error
            error_dict[i] = error
    
    # phase 9 until end of flight
    elif phase == 9 :
        log(f"{gps_auto[i]}\t{latitude[i]:.6f}\t{longitude[i]:.6f}\t{(altitude[i]-home_position_gps[2]):.6f}\t{utc_year[i]}\t{utc_month[i]}\t{utc_day[i]}\t{utc_hour[i]}\t{utc_minute[i]}\t{utc_sec[i]}\t{utc_ms[i]}\t{phase}\n")