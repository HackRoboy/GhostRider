#!/usr/bin/env python

import rospy
import tf
from std_msgs.msg import String


desired_angle = None
current_angle = .0

min_steering_error = 0.1

def get_driving_direction(quaternion):
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll, pitch, yaw = euler
    rospy.loginfo('Steering orientation: [%s, %s, %s]' % (roll, pitch, yaw))
    return yaw


def adjust_trajectory():
    if desired_angle is None:
        return None
    steering_error = current_angle - desired_angle
    rospy.loginfo('Steering error=%.2f', steering_error)
    steering_direction = None
    if steering_error > 0.1:
        steering_direction = 'right'
    elif steering_error < -0.1:
        steering_direction = 'left'
    return steering_direction


def controller():
    rospy.init_node('driving_controller', anonymous=True)
    steering_controller = rospy.Publisher('/robike/steering', String, queue_size=1)
    steering_transform = tf.TransformListener()
    rate = rospy.Rate(4)
    while not rospy.is_shutdown():
        try:
            global current_angle
            global desired_angle
            _, rot = steering_transform.lookupTransform('map', 'camera_link', rospy.Time(0))
            current_angle = get_driving_direction(rot)
            if desired_angle is None:
                desired_angle = current_angle
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(e)
        steering_direction = adjust_trajectory()
        if steering_direction:
            rospy.loginfo(steering_direction)
            steering_controller.publish(steering_direction)
        rate.sleep()

if __name__ == '__main__':
    controller()