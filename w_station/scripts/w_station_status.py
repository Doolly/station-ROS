#!/usr/bin/python
#_*_coding:utf-8_*_

import rospy
from std_msgs.msg import Int8MultiArray, Int8, String, Bool

global floor
global state
global destination



class ItemStatusSubscriber():
    def __init__(self):
        self.item_status_sub = rospy.Subscriber("wstation/item_status", Int8MultiArray, callback = self._callback)
        self.item_status_buf = None

        self.secondFloorCount = 0
        self.thirdFloorCount = 0

    def function(self):
        # priority => high floor                                                    
        if (self.secondFloorCount > self.thirdFloorCount):
            return 2
        elif (self.secondFloorCount < self.thirdFloorCount):
            return 3
        elif (self.secondFloorCount == 0 and self.thirdFloorCount == 0):
            return -1
        else:
            # same count
            return 3
    
    def _callback(self, msg):
        self.secondFloorCount = 0
        self.thirdFloorCount = 0

        matrix = [[0 for msg.layout.dim[0].stride in range(msg.layout.dim[0].size)] for msg.layout.dim[1].stride in range(msg.layout.dim[1].size)]

        for i in range(msg.layout.dim[1].size):
            for j in range(msg.layout.dim[0].size):
                matrix[i][j] = msg.data[(i * msg.layout.dim[1].size) + j]

        for i in range(msg.layout.dim[1].size):
            if (matrix[1][i] == 1):
                self.secondFloorCount += 1
            if (matrix[2][i] == 1):
                self.thirdFloorCount += 1


class LiftCurrentFloorSubscriber():
    def __init__(self):
        self.lift_current_floor_sub = rospy.Subscriber("wstation/lift_current_floor", Int8, callback=self._callback)
        self.lift_current_floor_buf = None

    def function(self):
        return self.lift_current_floor_buf
    
    def _callback(self, msg):
        self.lift_current_floor_buf = msg.data


class LiftStatusSubscriber():
    def __init__(self):
        self.lift_status_sub = rospy.Subscriber("wstation/lift_status", String, callback=self._callback)
        self.lift_status_buf = None

    def function(self):
        return self.lift_status_buf

    def _callback(self, msg):
        self.lift_status_buf = msg.data

class LiftItemStatusSubscriber():
    def __init__(self):
        self.lift_item_status_sub = rospy.Subscriber("wstation/lift_item_status", String, callback=self._callback)
        self.lift_item_status_buf = None

    def function(self):
        return self.lift_item_status_buf
    
    def _callback(self, msg):
        self.lift_item_status_buf = msg.data


class LiftItemSizeSubscriber():
    def __init__(self):
        self.lift_item_size_sub = rospy.Subscriber("wstation/lift_item_size", String, callback=self._callback)
        self.lift_item_size_buf = None

    def function(self):        
        return self.lift_item_size_buf
    
    def _callback(self, msg):
        self.lift_item_size_buf = msg.data

class EmergencySubscriber():
    def __init__(self):
        self.emergency_sub = rospy.Subscriber("wstation/emergency", Bool, callback=self._callback)
        self.emergency_buf = False

    def function(self):        
        return self.emergency_buf
    
    def _callback(self, msg):
        self.emergency_buf = msg.data


######################################################################################################################
######################################################################################################################

class TopicList:

    flooritemstatus = -1           # array, first 4 is 1st floor, and so on
    liftcurrentfloor = 0           # 1, 2, 3
    liftstatus = "arrived"         # arrived, up, down
    liftitemstatus = False         # True, False
    liftitemsize = "none"          # good, bad, none
    jamesarrived = False           # True, False # For identify James is Exist or not
    status = "none"                # 
    emergency = False


class WstationTask(TopicList):
    def __init__(self):
        print("hi")
        
    # def get_topic(self,data1,data2,data3,data4,data5,data6,data7,data8,data9):
    
    def definestatus(self):
        if self.emergency != True:
            if self.manual != True:
                if 

            if (self.jamesarrived == False) and (self.liftitemstatus == True) and (self.liftitemsize == "good") and (self.liftcurrentfloor == 1):
                status = "waitjames"
                return status

            elif (self.jamesarrived == True) and (self.liftitemstatus == True) and (self.liftitemsize == "good") and (self.liftcurrentfloor == 1):
                self.status = "tojames"
                return status

            elif (self.jamesarrived == True) and (self.liftitemstatus == True) and (self.liftitemsize == "bad") and (self.liftcurrentfloor == 1):
                self.status = "totray"
                return status

            elif (self.liftitemstatus == True) and (self.liftcurrentfloor != 1):
                self.status = "gofirstfloor"
                return status

            elif (self.liftitemstatus == False) and (self.liftcurrentfloor == 1) and (self.flooritemstatus == 2):
                self.status = "gosecondfloor"
                return status

            elif (self.liftitemstatus == False) and (self.liftcurrentfloor == 1) and (self.flooritemstatus == 3):
                self.status = "gothirdfloor"
                return status
                
            elif (self.liftitemstatus == False) and (self.liftcurrentfloor == self.flooritemstatus):
                self.status = "pushitem"
                return status

class WstationStatusPublisher(TopicList):
    def __init__(self):
        self.send_to_destination_pub = rospy.Publisher("wstation/status", String, queue_size=1)
        self.msg = String()

    def send_wstation_status(self):
        self.msg.data = self.status
        print("Status :  " + self.msg.data)
        self.send_to_destination_pub.publish(self.msg)

####################################################################################################################
####################################################################################################################


def wstation_main():
    rospy.init_node('Wstation_sub',anonymous=False)
    rate = rospy.Rate(10) # 300hz?

    item_status_sub            = ItemStatusSubscriber()             # IrSensor list
    lift_current_floor_sub     = LiftCurrentFloorSubscriber()
    lift_status_sub            = LiftStatusSubscriber()
    lift_item_status_sub       = LiftItemStatusSubscriber()
    lift_item_size_sub         = LiftItemSizeSubscriber()
    emergency_sub              = EmergencySubscriber()


    wstation_task              = WstationTask()
    wstation_status            = WstationStatusPublisher()

    print("Wstation start")
    while not rospy.is_shutdown():

        rate.sleep()


        # Subscribe from Arduino
        TopicList.itemstatus = item_status_sub.function()
        TopicList.liftcurrentfloor = lift_current_floor_sub.function()            # 1, 2, 3
        TopicList.liftstatus = lift_status_sub.function()                     # "wait", "move", "arrived"
        TopicList.liftitemstatus = lift_item_status_sub.function()            # "none", "exist"
        TopicList.liftitemsize = lift_item_size_sub.function()            # "good", "bad"
        TopicList.emergency = emergency_sub.function()


        wstation_task.definestatus()

        wstation_status.send_wstation_status()


if __name__ == '__main__':
    try:
        wstation_main()
    except rospy.ROSInterruptException:
        pass