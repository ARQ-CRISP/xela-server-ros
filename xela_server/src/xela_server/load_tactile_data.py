#!/usr/bin/env python
import rosbag
import rospy
import numpy as np
from xela_server.msg import xServerMsg


class LoadTactileData:

    def __init__(self, bag_file_name, first_timestamp = None):

        self.current_index = 0 # Current navigation index
        self.initial_tactile_index = 0
        self.final_tactile_index = 0
        self.velocity = 1 # Used to navigate the tactile data. Number of index's to move at a time
        self.timestamps = []
        self.raw_timestamps = [] # temporary (auxiliary) array
        self.messages = {"/sensor1": [],
                         "/sensor2": []}


        self.end_experiment_timestamp = None

        print('=================================================================================')
        print('Loading Tactile data')
        print('=================================================================================')
        # try:
        bag = rosbag.Bag(bag_file_name)

        for topic, msg, t in bag.read_messages(topics=['/xServTopic']):
            if msg.sensor == 1: # Only keep messages for sensor x
                self.messages["/sensor1"].append(msg)

            # for topic, msg, t in bag.read_messages(topics=['/uskin_xyz_values_normalized']):
            #     self.messages["/uskin_xyz_values_normalized"].append(msg)

                current_t = float(str(msg.header.stamp.secs)+"."+str(msg.header.stamp.nsecs).zfill(9))
                # Timestamps will taken from normalized data. In the future this might have to change
                self.raw_timestamps.append(current_t)
            elif msg.sensor == 2: # Only keep messages for sensor x
                self.messages["/sensor2"].append(msg)

            # for topic, msg, t in bag.read_messages(topics=['/uskin_xyz_values_normalized']):
            #     self.messages["/uskin_xyz_values_normalized"].append(msg)

                # current_t = float(str(msg.header.stamp.secs)+"."+str(msg.header.stamp.nsecs).zfill(9))
                # # Timestamps will taken from normalized data. In the future this might have to change
                # self.raw_timestamps.append(current_t)

        print("Timestamps lenght is {}. Sensor1 messages lenght is {} and Sensor2 messages lenght is {}".format(len(self.raw_timestamps), len(self.messages["/sensor1"]), len(self.messages["/sensor2"])))

        bag.close()


        # # Necessary to check if sensor1 and sensor2 provide the same number of data messages
        if (self.checkRawNormalizedDataSync() is False):
            del self
            return

        print("Tactile timestamps range from {} to {}".format(
            self.raw_timestamps[0], self.raw_timestamps[-1]))

        # It's possible to provide a timestamp that marks the beggining of the experiment (instead of the beggining of the recording)
        # The if the provided timestamp exists in the recording, timestamps will be reset to this value
        # self.timestamps is created at resetTimestamps().
        # resetTimestamps() sets initial timestamp to 0.0 (and the remaining accordingly)
        if first_timestamp is not None:

            print("Searching for tactile initial timestamp (at {})...".format(
                first_timestamp))
            print("Number of tactile frames is {}".format(len(self.raw_timestamps)))
            for index, time in enumerate(self.raw_timestamps):
                if first_timestamp < time:
                    print("Found initial timestamp {} for tactile data at: {}".format(
                        first_timestamp, time))

                    self.resetTimestamps(self.raw_timestamps[index])
                    break
                else:
                    self.current_index += 1
        else:
            self.resetTimestamps(self.raw_timestamps[0])

        self.setTimestampEndExperiment(index = len(self.timestamps)-1)

        self.getTactileDataRaw() # Load tactile data
        # self.getTactileDataNormalized() # Load tactile normalized data (used for visualization purposes)
        self.getTactileChangingEvents() # Creates array that signals when there are relevant changes in tactile data

        self.pub = rospy.Publisher(
            '/xServTopic', xServerMsg, queue_size=10) # Publishes tactile data (e.g. to be visualized by grid_map_ploter)


    def publishData(self):
        # print(len(self.messages["/uskin_xyz_values_normalized"]))
        # print(self.current_index)
        self.pub.publish(
            self.messages["/sensor1"][self.current_index])
        self.pub.publish(
            self.messages["/sensor2"][self.current_index])

    # Increase/Decrease the navigation speed
    def multiplyVelocity(self, factor):
        self.velocity *= factor
        self.velocity = int(self.velocity)

        if self.velocity < 1:
            self.velocity = 1


    def updateNextIndex(self):
        # Check that we don't go off bounds
        if self.current_index + self.velocity < len(self.messages["/sensor1"]):
            self.current_index += self.velocity
        else:
            print('End of the tactile Reached!!!')

    def updatePreviousIndex(self):
        # Check that we don't go off bounds
        if self.current_index - self.velocity >= 0:
            self.current_index -= self.velocity

    # # Check that number of messages for raw and normalized tactile data are the same
    def checkRawNormalizedDataSync(self):
        print('Tactile data array for sensor1 has size: {}'.format(len(self.messages["/sensor1"])))
        print('Tactile data array for sensor2 has size: {}'.format(len(self.messages["/sensor2"])))

        if (len(self.messages["/sensor1"]) == len(self.messages["/sensor2"])):
            print('Tactile data already have the same dimensions!')
        else:
            print('It\'s necessary to perform some syncronyzation')
            # We assume we always get the messages in the same order, Sensor1 and then Sensor2
            # If we have an extra message for Sensor1, we need to remove the last one (for which there is
            # no corresponding message from Sensor2). Otherwise, if we have an extra message from Sensor2,
            # we must remove the first message from Sensor2 (for which there is no corresponding message
            # from Sensor1)
            if len(self.messages["/sensor1"]) > len(self.messages["/sensor2"]):
                del self.messages["/sensor1"][-1] #Delete last message
                del self.raw_timestamps[-1] # since raw_timestamps is built from messages timestamps from Sensor1
            elif len(self.messages["/sensor2"]) > len(self.messages["/sensor1"]):
                del self.messages["/sensor2"][0] #Delete first message

        # Checking if data is in sync
        if (len(self.messages["/sensor1"]) != len(self.messages["/sensor2"])):
            print('Problems with data syncronization!')
            return False

        print('Tactile data array for sensor1 has size: {}'.format(len(self.messages["/sensor1"])))
        print('Tactile data array for sensor2 has size: {}'.format(len(self.messages["/sensor2"])))
        # print('Last message time difference is: {}'.format(self.messages["/uskin_xyz_values"][-1].header.stamp.nsecs - self.messages["/uskin_xyz_values_normalized"][-1].header.stamp.nsecs))

        return True


    # Ground timestamps to initial_timestamp
    def resetTimestamps(self, ground_value):
        self.timestamps = np.array(self.raw_timestamps)
        self.timestamps = (self.timestamps-ground_value)*1000

    # def getTactileDataNormalized(self):
    #     readings_no = len(self.timestamps)
    #     self.np_tactile_readings_normalized = np.empty((18, 3, readings_no), dtype=float)
    #     i = 0
    #     for tactile_frame in self.messages["/uskin_xyz_values_normalized"]:
    #         j = 0
    #         for tactile_node in tactile_frame.frame:
    #             # Remove data from damaged part of the sensor
    #             if tactile_node.header.frame_id not in ["130", "131", "132", "133", "134", "135"]:
    #               self.np_tactile_readings_normalized[j,0, i] = tactile_node.point.x
    #               self.np_tactile_readings_normalized[j,1, i] = tactile_node.point.y
    #               self.np_tactile_readings_normalized[j,2, i] = tactile_node.point.z
    #               j += 1
    #         i += 1
    #     return

    def getTactileDataRaw(self):
        readings_no = len(self.timestamps)
        self.np_sensor_1_readings_raw = np.empty((readings_no, 24, 3), dtype=float)
        self.np_sensor_2_readings_raw = np.empty((readings_no, 24, 3), dtype=float)

        i = 0
        for tactile_frame in self.messages["/sensor1"]:
            j = 0
            for tactile_node in tactile_frame.points:
                # Remove data from damaged part of the sensor
                # if tactile_node.header.frame_id not in ["130", "131", "132", "133", "134", "135"]:
                  self.np_sensor_1_readings_raw[i,j,0] = tactile_node.point.x
                  self.np_sensor_1_readings_raw[i,j,1] = tactile_node.point.y
                  self.np_sensor_1_readings_raw[i,j,2] = tactile_node.point.z
                  j += 1
            i += 1

        i = 0
        for tactile_frame in self.messages["/sensor2"]:
            j = 0
            for tactile_node in tactile_frame.points:
                # Remove data from damaged part of the sensor
                # if tactile_node.header.frame_id not in ["130", "131", "132", "133", "134", "135"]:
                  self.np_sensor_2_readings_raw[i,j,0] = tactile_node.point.x
                  self.np_sensor_2_readings_raw[i,j,1] = tactile_node.point.y
                  self.np_sensor_2_readings_raw[i,j,2] = tactile_node.point.z
                  j += 1
            i += 1
        return

    def getTactileChangingEvents(self):
        readings_no = len(self.timestamps)
        self.np_sensor_1_changes_label = np.zeros(readings_no, dtype=int)
        self.np_sensor_2_changes_label = np.zeros(readings_no, dtype=int)

        self.np_sensor_1_changes_max_label = np.zeros(readings_no, dtype=int)
        self.np_sensor_2_changes_max_label = np.zeros(readings_no, dtype=int)

        self.np_sensor_1_changes_diff_max = np.zeros((readings_no,3), dtype=int)
        self.np_sensor_2_changes_diff_max = np.zeros((readings_no,3), dtype=int)

        self.tactile_sensor_1_diff = []
        self.tactile_sensor_2_diff = []

        # Get tactile image difference. For now we are using tactile normalized values
        print("PRINTING DIFF FOR SENSOR1")
        for i in range(1,readings_no):
            self.tactile_sensor_1_diff.append(self.np_sensor_1_readings_raw[i] - self.np_sensor_1_readings_raw[i-1])
            # self.tactile_readings_diff.append(self.np_tactile_readings_normalized[:,:,i] - self.np_tactile_readings_normalized[:,:,i-1])
            # print("Std is {}".format(np.std(self.tactile_sensor_1_diff[-1])))
            # print("Mean std is {}".format(np.mean(np.std(self.tactile_sensor_1_diff, axis=(1,2)))))
            self.np_sensor_1_changes_diff_max[i] = np.max(abs(self.tactile_sensor_1_diff[-1]), axis=0)
            if np.std(self.tactile_sensor_1_diff[-1]) > 4.5:
                # print("{}\n".format(np.std(self.tactile_sensor_1_diff[-1])))
                self.np_sensor_1_changes_label[i] = 1
        print("Maximum change detected for sensor 1: {}".format(np.max(self.np_sensor_1_changes_diff_max, axis=0)))
        print("At indexes: {}".format(np.argmax(self.np_sensor_1_changes_diff_max, axis=0)))

        # Get tactile image difference. For now we are using tactile normalized values
        print("PRINTING DIFF FOR SENSOR2")
        for i in range(1,readings_no):
            self.tactile_sensor_2_diff.append(self.np_sensor_2_readings_raw[i] - self.np_sensor_2_readings_raw[i-1])
            # self.tactile_readings_diff.append(self.np_tactile_readings_normalized[:,:,i] - self.np_tactile_readings_normalized[:,:,i-1])
            # print("Std is {}".format(np.std(self.tactile_sensor_2_diff[-1])))
            # print("Mean std is {}".format(np.mean(np.std(self.tactile_sensor_2_diff, axis=(1,2)))))
            self.np_sensor_2_changes_diff_max[i] = np.max(abs(self.tactile_sensor_2_diff[-1]), axis=0)
            if np.std(self.tactile_sensor_2_diff[-1]) > 4.5:
                # print("{}\n".format(np.std(self.tactile_sensor_2_diff[-1])))
                self.np_sensor_2_changes_label[i] = 1
        print("Maximum change detected for sensor 1: {}".format(np.max(self.np_sensor_2_changes_diff_max, axis=0)))
        print("At indexes: {}".format(np.argmax(self.np_sensor_2_changes_diff_max, axis=0)))


    # Grounds experiment timestamp to currently selected timestamp
    def setTimestampBeginningExperiment(self):
        self.initial_tactile_index = self.current_index
        self.resetTimestamps(self.raw_timestamps[self.current_index])

    def setTimestampEndExperiment(self, index = None):
        # print("setTimestampEndExperiment   index is: {}".format(index))
        if index is not None:
            self.final_tactile_index = index
            self.end_experiment_timestamp = self.timestamps[index]
        else:
            self.final_tactile_index = self.current_index
            self.end_experiment_timestamp = self.timestamps[self.current_index]

    def getTimestampBeginningExperiment(self):
        return self.timestamps[self.initial_tactile_index]

    def getTimestampEndExperiment(self):
        return self.end_experiment_timestamp
