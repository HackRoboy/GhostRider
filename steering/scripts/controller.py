#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from roboy_communication_middleware.msg import MotorCommand, MotorStatus
from roboy_communication_middleware.srv import ControlMode

DUMMY_INIT_POS = 0

LEFT_MOTOR_ID = 1
LEFT_MOTOR_INIT_POS = 0
LEFT_MOTOR_DELTA_PULL = 50000
LEFT_MOTOR_DELTA_RELEASE = -50000

RIGHT_MOTOR_ID = 2
RIGHT_MOTOR_INIT_POS = 0
RIGHT_MOTOR_DELTA_PULL = 50000
RIGHT_MOTOR_DELTA_RELEASE = -50000

MOTOR_CONTROL_POSITION = 0
MOTOR_CONTROL_VELOCITY = 1
MOTOR_CONTROL_DISPLACEMENT = 2

fpga_id = 4 # TODO: retrieve id automatically
last_known_left_motor_position = LEFT_MOTOR_INIT_POS
last_known_right_motor_position = RIGHT_MOTOR_INIT_POS


def turn_left_command():
    command = MotorCommand()
    command.id = fpga_id
    command.motors = [LEFT_MOTOR_ID, RIGHT_MOTOR_ID]
    command.setPoints = [
        last_known_left_motor_position+LEFT_MOTOR_DELTA_PULL,
        last_known_right_motor_position+ RIGHT_MOTOR_DELTA_RELEASE
    ]
    return command


def turn_right_command():
    command = MotorCommand()
    command.id = fpga_id
    command.motors = [LEFT_MOTOR_ID, RIGHT_MOTOR_ID]
    command.setPoints = [
        last_known_left_motor_position+LEFT_MOTOR_DELTA_RELEASE,
        last_known_right_motor_position+RIGHT_MOTOR_DELTA_PULL
    ]
    return command


def init_pos_command():
    command = MotorCommand()
    command.id = fpga_id
    command.motors = [LEFT_MOTOR_ID, RIGHT_MOTOR_ID]
    command.setPoints = [LEFT_MOTOR_INIT_POS, RIGHT_MOTOR_INIT_POS]
    return command


def to_str(command):
    return 'l=%s,r=%s' % (command.setPoints[0], command.setPoints[1])


def controller():
    rospy.init_node('steering_controller', anonymous=True)
    rospy.loginfo('Create motor commands publisher.')
    fpga_publisher = rospy.Publisher('/roboy/middleware/MotorCommand',
                                     MotorCommand,
                                     queue_size=1)
    rospy.loginfo('Create motor commands service client.')
    rospy.wait_for_service('/roboy/shoulder_right/middleware/ControlMode')
    # Set steering in init_position
    set_control_mode = rospy.ServiceProxy('/roboy/shoulder_right/middleware/ControlMode',
                                          ControlMode)
    set_control_mode(MOTOR_CONTROL_POSITION, DUMMY_INIT_POS)
    rospy.loginfo('Reset motor to initial position.')
    fpga_publisher.publish(init_pos_command())

    def process_steering_commands(data):
        if data.data == 'left':
            command = turn_left_command()
            rospy.loginfo('Turn left to %s' % to_str(command))
        elif data.data == 'right':
            command = turn_right_command()
            rospy.loginfo('Turn right to %s' % to_str(command))
        elif data.data == 'reset':
            rospy.loginfo('Reset to default steering position.')
            command = init_pos_command()
        else:
            rospy.logerror('Unknown command received %s', data.data)
            return
        fpga_publisher.publish(command)

    def process_motor_status_info(data):
        global last_known_left_motor_position
        global last_known_right_motor_position
        last_known_left_motor_position = data.position[LEFT_MOTOR_ID]
        last_known_right_motor_position = data.position[RIGHT_MOTOR_ID]

    rospy.Subscriber('/roboy/middleware/MotorStatus', MotorStatus, process_motor_status_info)
    rospy.Subscriber('robike/steering', String, process_steering_commands)
    rospy.spin()

if __name__ == '__main__':
    controller()
