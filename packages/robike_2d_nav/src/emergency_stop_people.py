#!/usr/bin/env python
import rospy
from cob_perception_msgs.msg import People
from ros_arduino_msgs.srv import *

class PeopleSafetyNode:
    def __init__(self):
        rospy.init_node('emergency_stop_people', anonymous=True)
        self.safety_margin = rospy.get_param('safety_margin', 0.2)
        rospy.wait_for_service('/arduino/analog_write')
        try:
            self.stop_motor_client = rospy.ServiceProxy('/arduino/analog_write', AnalogWrite)
	except:
	    pass
	print('init finished')
	
    def callback(self, data):
        for person in data.people:
	    print('Person distance is: ', person.position.position.x**2 + person.position.position.y**2)
            if (person.position.position.x**2 + person.position.position.y**2) < self.safety_margin:
                rospy.logwarn("EMERGENCY STOP, PERSON TOO CLOSE")
		resp = self.stop_motor_client(AnalogWriteRequest(9, 0))

    def listener(self):
        rospy.Subscriber("people", People, self.callback)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
    person_safety_node = PeopleSafetyNode()
    person_safety_node.listener()
