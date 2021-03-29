#!/usr/bin/python
#_*_coding:utf-8_*_

import rospy
from std_msgs.msg import Int8MultiArray,MultiArrayDimension

rospy.init_node('int8array',anonymous=False)

motor = rospy.Publisher('wstation/item_status', Int8MultiArray, queue_size = 2)
rate = rospy.Rate(1) # 300hz?

while not rospy.is_shutdown():
    rate.sleep()

    arg = Int8MultiArray()

    thing = MultiArrayDimension()

    thing.label = "floors"
    thing.size = 4
    thing.stride = 12
    
    arg.layout.dim.append(thing)

    thing1 = MultiArrayDimension()

    thing1.label = "states"
    thing1.size = 3
    thing1.stride = 3
    arg.layout.dim.append(thing1)

    arg.layout.data_offset = 0
    arg.data = [1,1,1,1,1,
                0,0,0,1,1,
                0,0,1,0,1]

    motor.publish(arg)