#!/usr/bin/env python
import sys
import os

import rospy

import numpy as np
import h5py
import json

from pynput import keyboard
import datetime
import fnmatch
import glob  # Search for files' paths

from xela_server import LoadTactileData

tactile_rate = 180

config_file_path = ""
config_dict = {}

has_data_been_saved = False


# Read experimental information from json congig file and get necessary file paths


def getCompleteFilePaths():

    global config_dict
    for file_type in ['tactile_data_path', 'h5py_file_path', 'h5py_tactile_file_name']:
        config_dict[file_type] = config_dict[file_type].replace(
            "${object}", config_dict['experiment']['object'])

    rospy.loginfo("Opening Tactile data file: {}".format(config_dict['tactile_data_path']+"Grasp_"+config_dict['experiment']['object']+"_"+
        config_dict['experiment']['pose']+'_'+config_dict['experiment']['slip']+'_'+str(config_dict['experiment']['experiment_num'])+'_'))
    config_dict['tactile_data_path'] = glob.glob(config_dict['tactile_data_path']+"Grasp_"+config_dict['experiment']['object']+"_"+
        config_dict['experiment']['pose']+'_'+config_dict['experiment']['slip']+'_'+str(config_dict['experiment']['experiment_num'])+'_' + "*.bag")[0]
    rospy.loginfo("Opening Tactile data file: {}".format(
        config_dict['tactile_data_path']))

    config_dict['experiment_key'] = config_dict['experiment']['object'] + \
        '/Pose_' + str(config_dict['experiment']['pose']) + \
        '/' + str(config_dict['experiment']['slip']) + \
        '/Exp' + str(config_dict['experiment']['experiment_num'])
    return


def readConfigFile():
    global config_dict
    global config_file_path
    rospy.loginfo(
        '=================================================================================')
    rospy.loginfo('Loading Necessary Config Files and Data files')
    rospy.loginfo(
        '=================================================================================')

    if rospy.has_param('/navigate_tactile_data/json_file_name'):

        config_file_path = rospy.get_param(
            "/navigate_tactile_data/json_file_name")

        with open(config_file_path, 'r') as f:
            config_dict = json.load(f)

        # Necessary since most files include an unknown timestamp
        getCompleteFilePaths()

        rospy.loginfo('Sucessfully read from config JSON file')
        return True
    else:
        rospy.loginfo('Unable to find config JSON file')
        return False

# Check if experiment has already been recorded in hdf5 file.


def groupExistsHDF5(experiment_key, hf):
    if experiment_key in hf:
        return True
    return False

def confirmOverwriteHDF5Data(experiment_key, hf):
    confirm_key = raw_input(
        "Experiment already exists in file. Are you sure you wish to overwrite your data?\n")
    rospy.loginfo(confirm_key)
    if confirm_key != 'y':
        hf.close()
        return False
    # Delete all existing data in HDF5 file for this experiment
    else:
        del hf[experiment_key]

    return True

def overwriteHDF5Data(experiment_key, hf):
    del hf[experiment_key]

    return


def saveH5pyFile(experiment_key, file_path, tactile):
    rospy.loginfo(
        'Will attempt to save data into file located at {}'.format(file_path))

    # Get initial and end index's for tactile and vision
    tactile_start_index = tactile.initial_tactile_index
    tactile_end_index = tactile.final_tactile_index

    # Saving Tactile data
    hf = h5py.File(file_path+config_dict['h5py_tactile_file_name'], 'a')

    # Check if experiment has already been recorded in hdf5 file.
    if groupExistsHDF5(experiment_key, hf):
        # Confirm that user wants to overwrite data
        if not confirmOverwriteHDF5Data(experiment_key, hf):
            return False
        # # Overwrite data (user has already confirmed for vision)
        # overwriteHDF5Data(experiment_key, hf)

    rospy.loginfo('Saving {} samples of tactile data (start index: {}, end index: {}).'.format(
        tactile_end_index-tactile_start_index, tactile_start_index, tactile_end_index))

    group_tactile = hf.create_group(experiment_key+"/sensor1")
    # group_tactile = hf.create_group(experiment_key+'/Tactile')

    group_tactile.create_dataset(
        'tactile_timestamps', data=tactile.timestamps[tactile_start_index:tactile_end_index])
    group_tactile.create_dataset(
        'tactile_data_raw', data=tactile.np_sensor_1_readings_raw[tactile_start_index:tactile_end_index, :, :])
    # group_tactile.create_dataset(
    #     'tactile_data_raw/x', data=tactile.np_sensor_1_readings_raw[:, 0, tactile_start_index:tactile_end_index])
    # group_tactile.create_dataset(
    #     'tactile_data_raw/y', data=tactile.np_sensor_1_readings_raw[:, 1, tactile_start_index:tactile_end_index])
    # group_tactile.create_dataset(
    #     'tactile_data_raw/z', data=tactile.np_sensor_1_readings_raw[:, 2, tactile_start_index:tactile_end_index])
    group_tactile.create_dataset(
        'tactile_changes_label', data=tactile.np_sensor_1_changes_label[tactile_start_index:tactile_end_index])
    group_tactile.create_dataset(
        'tactile_changes_max_channel_label', data=tactile.np_sensor_1_changes_diff_max[tactile_start_index:tactile_end_index])


    group_tactile = hf.create_group(experiment_key+"/sensor2")
    # group_tactile = hf.create_group(experiment_key+'/Tactile')

    group_tactile.create_dataset(
        'tactile_timestamps', data=tactile.timestamps[tactile_start_index:tactile_end_index])
    group_tactile.create_dataset(
        'tactile_data_raw', data=tactile.np_sensor_2_readings_raw[tactile_start_index:tactile_end_index, :, :])
    # group_tactile.create_dataset(
        # 'tactile_data_raw/x', data=tactile.np_sensor_2_readings_raw[:, 0, tactile_start_index:tactile_end_index])
    # group_tactile.create_dataset(
        # 'tactile_data_raw/y', data=tactile.np_sensor_2_readings_raw[:, 1, tactile_start_index:tactile_end_index])
    # group_tactile.create_dataset(
        # 'tactile_data_raw/z', data=tactile.np_sensor_2_readings_raw[:, 2, tactile_start_index:tactile_end_index])
    group_tactile.create_dataset(
        'tactile_changes_label', data=tactile.np_sensor_2_changes_label[tactile_start_index:tactile_end_index])
    group_tactile.create_dataset(
        'tactile_changes_max_channel_label', data=tactile.np_sensor_2_changes_diff_max[tactile_start_index:tactile_end_index])

    hf.close()

    return True


def on_press(key):
    global has_data_been_saved

    try:

        if hasattr(key, 'char') and key.char == "h":

            rospy.loginfo(
                "==========================KeyBoard Instructions==========================")
            rospy.loginfo(
                "Press '->' key to move torwars next tactile data sample.")
            rospy.loginfo(
                "Press '<-' key to move torwars previous tactile data sample.")
            rospy.loginfo(
                "Press 'f' key to set the first tactile data sample.")
            rospy.loginfo(
                "Press 'l' key to set the last tactile data sample.")
            rospy.loginfo("Press 's' key to to save data to HDF5 file.")
            rospy.loginfo("Press 'e' key to to exit.")
            rospy.loginfo(
                "=========================================================================")

            return False

        elif hasattr(key, 'char') and key.char == "s":
            # confirm_key = raw_input("Do you want to save your data/labels? Existent data will be overwrtitten!\n")
            # rospy.loginfo(confirm_key)
            # if confirm_key == 'y':
            rospy.loginfo("Saving all relevant data to HDF5 file")

            if saveH5pyFile(config_dict['experiment_key'],
                            config_dict['h5py_file_path'],
                            tactile):
                rospy.loginfo('Data has been saved!')
                has_data_been_saved = True
                return False

            # rospy.loginfo('Data has NOT been saved!')

            # return False


        elif hasattr(key, 'char') and key.char == "l":

            rospy.loginfo("Defining final timestamp")

            tactile.setTimestampEndExperiment()

            return False

        elif hasattr(key, 'char') and key.char == "f":

            rospy.loginfo("Defining initial timestamp")

            tactile.setTimestampBeginningExperiment()

            return False


        elif key == keyboard.Key.right:

            rospy.loginfo("Jumping to next time instance")
            # Navigate tactile_frames
            tactile.updateNextIndex()

            return False

        elif key == keyboard.Key.left:

            rospy.loginfo("Jumping to previus time instance")
            # Navigate tactile_frames
            tactile.updatePreviousIndex()

            return False

        elif key == keyboard.Key.up:

            rospy.loginfo("Velocity increasead by 2x")

            # Navigate tactile_frames
            tactile.multiplyVelocity(2)

            return False

        elif key == keyboard.Key.down:

            rospy.loginfo("Velocity halved (0.5x)")

            # Navigate tactile_frames
            tactile.multiplyVelocity(0.5)

            return False

    except AttributeError as ex:
        rospy.loginfo(ex)


def on_release(key):
    if hasattr(key, 'char') and key.char == "e":
        # Stop listener
        rospy.signal_shutdown("Closing the node - 'e' key has been pressed")

        return False


def wait_for_user_input():
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()
    listener.join()  # wait till listener will stop
    # other stuff


if __name__ == '__main__':

    rospy.init_node('navigate_tactile_data', anonymous=True)
    rate = rospy.Rate(50)  # 10hz

    # Read JSON config file to retrieve all relevant paths
    if not readConfigFile():
        # rospy.signal_shutdown("Closing the node - valid JSON file was not found")
        sys.exit()

    # Load all relevant data files
    tactile = LoadTactileData(config_dict['tactile_data_path'])

    # Print general information
    rospy.loginfo(
        '=================================================================================')
    rospy.loginfo("Tactile sample frequency is: {} Hz".format(tactile_rate))

    rospy.loginfo('Begin tactile timestamp is: {}'.format(
        tactile.raw_timestamps[tactile.current_index]))


    # Print current information
    while not rospy.is_shutdown():
        # vision.showFrame()
        tactile.publishData()
        rospy.loginfo(
            '=================================================================================')
        # rospy.loginfo('Current vision timestamp: {}'.format(
            # vision.timestamps[vision.current_index]))
        rospy.loginfo('Current tactile timestamp: {}'.format(
            tactile.timestamps[tactile.current_index]))

        rospy.loginfo('Current tactile index: {}'.format(
            tactile.current_index))

        # rospy.loginfo('Current robot state is: {}'.format(
            # exp_states.getCurrentTask(tactile.timestamps[tactile.current_index])))

        try:
            rospy.loginfo('Tactile CHANGES labels for sensor 1: ( {}  {}  {}  {}  [ {} ]  {}  {}  {}  {})'.format(
                *tactile.np_sensor_1_changes_label[tactile.current_index-4:tactile.current_index+5]))
            rospy.loginfo('Tactile CHANGES labels for sensor 2: ( {}  {}  {}  {}  [ {} ]  {}  {}  {}  {})'.format(
                *tactile.np_sensor_2_changes_label[tactile.current_index-4:tactile.current_index+5]))
        except IndexError:
            rospy.loginfo('Tactile CHANGES labels for sensor 1: ( {})'.format(
                tactile.np_sensor_1_changes_label[tactile.current_index]))
            rospy.loginfo('Tactile CHANGES labels for sensor 2: ( {})'.format(
                tactile.np_sensor_2_changes_label[tactile.current_index]))

        try:
            rospy.loginfo('Tactile MAX CHANGES labels for sensor 1: ( {})'.format(
                tactile.np_sensor_1_changes_diff_max[tactile.current_index]))
            rospy.loginfo('Tactile MAX CHANGES labels for sensor 2: ( {})'.format(
                tactile.np_sensor_2_changes_diff_max[tactile.current_index]))
        except IndexError:
            rospy.loginfo('Tactile MAX CHANGES labels for sensor 1: ( {})'.format(
                tactile.np_sensor_1_changes_diff_max[tactile.current_index]))
            rospy.loginfo('Tactile MAX CHANGES labels for sensor 2: ( {})'.format(
                tactile.np_sensor_2_changes_diff_max[tactile.current_index]))


        rospy.loginfo('--------------Other Information-----------------')
        # rospy.loginfo('Last vision frame timestamp is: {}'.format(
            # vision.getTimestampEndExperiment()))
        rospy.loginfo('First tactile frame timestamp is: {}'.format(
            tactile.getTimestampBeginningExperiment()))
        rospy.loginfo('Last tactile frame timestamp is: {}'.format(
            tactile.getTimestampEndExperiment()))
        # rospy.loginfo(
            # 'Has last vision frame been updated?: {}'.format(has_last_vision_frame_been_updated))
        rospy.loginfo('Has data been saved?: {}'.format(has_data_been_saved))
        #print('current start_index is: {}, current end_index is: {}'.format(tactile.initial_tactile_index, getEndExperimentTactileIndex(manual_labels.getTimestampEndExperiment())))
        rospy.loginfo(
            '=================================================================================')
        wait_for_user_input()
        rate.sleep()

    del tactile
