#!/usr/bin/python
#_*_coding:utf-8_*_

import rospy
from std_msgs.msg import Int8MultiArray,MultiArrayDimension

rospy.init_node('int8array',anonymous=False)

motor = rospy.Publisher('wstation/item_is_exist', Int8MultiArray, queue_size = 2)

while not rospy.is_shutdown():
    arg = Int8MultiArray()

    thing = MultiArrayDimension()

    thing.label = "x"
    thing.size = 4
    thing.stride = 12
    arg.layout.dim.append(thing)
    thing1 = MultiArrayDimension()

    thing1.label = "y"
    thing1.size = 3
    thing1.stride = 3
    arg.layout.dim.append(thing1)

    arg.layout.data_offset = 0
    arg.data = [0,1,0,0,
                0,0,0,0,
                0,0,0,1]

    motor.publish(arg)