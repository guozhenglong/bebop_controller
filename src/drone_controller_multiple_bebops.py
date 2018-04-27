#!/usr/bin/env python

# A basic drone controlle#!/usr/bin/env python

# A basic drone controller class for the tutorial "Up and flying with the AR.Drone and ROS | Getting Started"
# https://github.com/mikehamer/ardrone_tutorials_getting_started

# This class implements basic control functionality which we will be using in future tutorials.
# It can command takeoff/landing/emergency as well as drone movement
# It also tracks the drone state based on navdata feedback

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('bebop_controller')
import rospy

# Import the messages we're interested in sending and receiving
from geometry_msgs.msg import Twist  	 # for sending commands to the drone
from std_msgs.msg import Empty       	 # for land/takeoff/emergency
# from ardrone_autonomy.msg import Navdata # for receiving navdata feedback

# An enumeration of Drone Statuses
# from drone_status import DroneStatus

# Some Constants
COMMAND_PERIOD = 100 #ms

import fileinput


class MyDrone:
    def __init__(self, id, landPath, pubTakeoffPath, pubResetPath, cmdvelPath):
        self.id = id
        self.landPath = landPath
        self.pubTakeoffPath = pubTakeoffPath
        self.pubResetPath = pubResetPath
        self.cmdvelPath = cmdvelPath
        self.status = -1

        # rospy.init_node('bebop_controller')
        # Subscribe to the /ardrone/navdata topic, of message type navdata, and call self.ReceiveNavdata when a message is received
        # self.subNavdata = rospy.Subscriber(navdataPath, Navdata, self.ReceiveNavdata)

        # Allow the controller to publish to the /ardrone/takeoff, land and reset topics
        self.pubLand = rospy.Publisher(landPath, Empty, queue_size=10)
        self.pubTakeoff = rospy.Publisher(pubTakeoffPath, Empty, queue_size=10)
        self.pubReset = rospy.Publisher(pubResetPath, Empty, queue_size=10)

        # Allow the controller to publish to the /cmd_vel topic and thus control the drone
        self.pubCommand = rospy.Publisher(cmdvelPath, Twist, queue_size=10)
	# Setup regular publishing of control packets
	self.command = Twist()
 #    def ReceiveNavdata(self, navdata):
 #        # Although there is a lot of data in this packet, we're only interested in the state at the moment
	# self.status = navdata.state
    def SendTakeoff(self):
        # Send a takeoff message to the ardrone driver
        # Note we only send a takeoff message if the drone is landed - an unexpected takeoff is not good!
        # if (self.status == DroneStatus.Landed):
		self.pubTakeoff.publish(Empty())
    def SendLand(self):
        # Send a landing message to the ardrone driver
        # Note we send this in all states, landing can do no harm
        self.pubLand.publish(Empty())

    def SendEmergency(self):
        # Send an emergency (or reset) message to the ardrone driver
        self.pubReset.publish(Empty())

    def SendCommand(self, event):
        # The previously set command is then sent out periodically if the drone is flying
        # if self.status == DroneStatus.Flying or self.status == DroneStatus.GotoHover or self.status == DroneStatus.Hovering:
            self.pubCommand.publish(self.command)


class BasicDroneController(object):
    droneList = []
    def __init__(self):
        groupNodeCount = 0
        try:
            for line in fileinput.input("/home/exbot/catkin_ws/src/bebop_controller/launch/keyboard_controller.launch"):
                line = line.strip(' \r\n\t')
                index = line.find("<group")
                if (0 == index):
                    groupNodeCount = groupNodeCount + 1
        except:
            print("read file ancounter an error!")

        # Holds the current drone status
        droneDic = {}
        groupNodeIndex = 1
        while groupNodeIndex <= groupNodeCount:
            bebopName = "bebop%d" % groupNodeIndex;
            # seq = ['/', bebopName, '/bebop', '/navdata']
            # navdataPath = ''
            # navdataPath = navdataPath.join(seq)

            seq = ['/', bebopName, '/bebop', '/land']
            landPath = ''
            landPath = landPath.join(seq)

            seq = ['/', bebopName, '/bebop', '/takeoff']
            pubTakeoffPath = ''
            pubTakeoffPath = pubTakeoffPath.join(seq)

            seq = ['/', bebopName, '/bebop', '/reset']
            pubResetPath = ''
            pubResetPath = pubResetPath.join(seq)

            seq = ['/', bebopName, '/bebop','/cmd_vel']
            cmdvelPath = ''
            cmdvelPath = cmdvelPath.join(seq)

            drone = MyDrone(groupNodeIndex, landPath, pubTakeoffPath, pubResetPath, cmdvelPath)
            self.droneList.append(drone)
            groupNodeIndex = groupNodeIndex + 1

	# Setup regular publishing of control packets
	self.command = Twist()
	self.commandTimer = rospy.Timer(rospy.Duration(COMMAND_PERIOD / 1000.0), self.SendCommand)

	# Land the drone if we are shutting down
	rospy.on_shutdown(self.SendLand)

# print(groupNodeCount)

    def SendTakeoff(self):
        # Send a takeoff message to the ardrone driver
        # Note we only send a takeoff message if the drone is landed - an unexpected takeoff is not good!
	for drone in self.droneList:
		drone.SendTakeoff()
    def SendLand(self):
        # Send a landing message to the ardrone driver
        # Note we send this in all states, landing can do no harm
        for drone in self.droneList:
            drone.SendLand()

    def SendEmergency(self):
        # Send an emergency (or reset) message to the ardrone driver
        for drone in self.droneList:
            drone.SendEmergency()

    def SetCommand(self, roll=0, pitch=0, yaw_velocity=0, z_velocity=0):
        # Called by the main program to set the current command
        self.command.linear.x = pitch
        self.command.linear.y = roll
        self.command.linear.z = z_velocity
        self.command.angular.z = yaw_velocity

    # def SendCommand(self, event):
    #     # The previously set command is then sent out periodically if the drone is flying
    #     #if self.droneList[0].status == DroneStatus.Flying orself.droneList[0].status == DroneStatus.GotoHover or self.droneList[0].status == DroneStatus.Hovering:
           # self.droneList[1].pubCommand.publish(self.command)

    def SendCommand(self, event):
        # The previously set command is then sent out periodically if the drone is flying
        for drone in self.droneList:
        	# print(len(droneList))
            drone.SendCommand(self.command)

if __name__ == "__main__":
	drone = BasicDroneController()


