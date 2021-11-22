#!/usr/bin/env python


import rospy
from xela_server.srv import TactileRecord, TactileRecordResponse
import subprocess, shlex, psutil #Necessary to generate 'rosbag record' command (and process kill)
from pynput import keyboard


def my_function(request):
    """
        This function (feel free to change its name) must contain your code.
        It MUST have a single input argument, that will be the request part of the srv, so DO NOT change it.
        You can call other functions inside it without any problem
    """
    # Access to the msg stored in request
    experiment_number = request.input
    # The core of your code should be called or written here
    # ...
    # Change srvName by the name of your srv
    response = TactileRecordResponse()

    confirm_key = raw_input(
        "Do you wish to record a new experiment?\n")

    if confirm_key == 'y':
        command = "rosbag record -o " + tactile_record_path + "_" + str(experiment_number) + " " + "/xServTopic"
        response.returned_object = tactile_record_path + "_" + str(experiment_number)
        command = shlex.split(command)
        rosbag_proc = subprocess.Popen(command)

        rospy.sleep(10)

        # Kill Rosbag record node process
        for proc in psutil.process_iter():
            if "record" in proc.name() and set(command[2:]).issubset(proc.cmdline()):
                proc.send_signal(subprocess.signal.SIGINT)

        rosbag_proc.send_signal(subprocess.signal.SIGINT)

        response.outcome = 0

    elif confirm_key == 'e':
        #Exit task
        response.outcome = 2
        response.returned_object = ""
    else:
        # Redo experiment
        response.outcome = 1
        response.returned_object = ""

    # Return the response
    return response


if __name__ == '__main__':
    # Initialise the node with a specific name (please change it to match your service)
    rospy.init_node('tactile_rosbag_record_server')

    tactile_record_path = rospy.get_param("/tactile_record_path")

    # Set the name of the service, specify which kind of srv will trigger it and what function will be run.
    # Change the name of the server with one that matches the content of your code, set the second argument to the name
    # of the srv file, and the last one should be the name of the function that runs your code.
    service = rospy.Service("tactile_record_service", TactileRecord, my_function)
    rospy.spin()
