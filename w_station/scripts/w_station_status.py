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

        sizeofarray = 5
        numofarray = 3

        matrix = [[0 for _ in range(sizeofarray)] for _ in range(numofarray)]

        for i in range(numofarray):
            for j in range(sizeofarray):
                matrix[i][j] = msg.data[(i * sizeofarray) + j]

        for i in range(sizeofarray):
            if (matrix[1][i] == 1):
                self.secondFloorCount += 1
            if (matrix[2][i] == 1):
                self.thirdFloorCount += 1

        # print("secondfloor :" + str(self.secondFloorCount))
        # print("thirdfloor :" + str(self.thirdFloorCount))


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

class LifDestinationFloorSubscriber():
    def __init__(self):
        self.lift_destination_floor_sub = rospy.Subscriber("wstation/lift_destination_floor", Int8, callback=self._callback)
        self.lift_destination_floor_buf = None

    def function(self):        
        return self.lift_destination_floor_buf
    
    def _callback(self, msg):
        self.lift_destination_floor_buf = msg.data

class EmergencySubscriber():
    def __init__(self):
        self.emergency_sub = rospy.Subscriber("wstation/emergency", Bool, callback=self._callback)
        self.emergency_buf = False

    def function(self):        
        return self.emergency_buf
    
    def _callback(self, msg):
        self.emergency_buf = msg.data


class ManualSubscriber():
    def __init__(self):
        self.manual_sub = rospy.Subscriber("wstation/manual", Bool, callback=self._callback)
        self.manual_buf = False

    def function(self):        
        return self.manual_buf
    
    def _callback(self, msg):
        self.manual_buf = msg.data


######################################################################################################################
######################################################################################################################

class TopicList:

    flooritemstatus = -1           # array, first 4 is 1st floor, and so on
    liftcurrentfloor = 0           # 1, 2, 3
    liftstatus = "arrived"         # arrived, up, down
    liftitemstatus = False         # True, False
    liftitemsize = "none"          # good, bad, none
    jamesarrived = False           # True, False # For identify James is Exist or not
    
    status = "none"                # Wstation Status
    emergency = False              # True, False
    manual = False                 # True, False


class WstationTask(TopicList):
    def __init__(self):
        print("Start Define Wsation Status")
    
    def definestatus(self):

        if self.emergency != True:

            if self.liftitemstatus == True:

                if self.liftitemsize == "good":

                    if self.liftcurrentfloor == 1:

                        if self.liftstatus == "arrived":

                            if self.jamesarrived == True:

                                self.status = "tojames"

                            else: # up, down

                                self.status = "waitjames"

                    else:
                        self.status = "gofirstfloor"

                elif self.liftitemsize == "bad":
                    
                    if self.liftcurrentfloor == 1:

                        if self.liftstatus == "arrived":

                            self.status = "totray"

                        else: # up,down

                            self.status = "gofirstfloor"

                    else:
                        self.status = "gofirstfloor"

                else: # none

                    self.status = "gofirstfloor"

            elif self.liftitemstatus == False:

                if self.liftcurrentfloor == 1:

                    if self.flooritemstatus == -1:

                        self.status = "waititem"
                    
                    elif self.flooritemstatus == 2:

                        self.status = "gosecondfloor"
                    
                    elif self.flooritemstatus == 3:

                        self.status = "gothirdfloor"
                
                elif self.liftcurrentfloor == 2:

                    if (self.flooritemstatus == -1) and (self.liftdestinationfloor == -1):

                        self.status = "gofirstfloor"
                    
                    elif (self.flooritemstatus == 2) and (self.liftitemstatus == "arrived") and (self.liftdestinationfloor == 2):

                        self.status = "pushitem"
                    
                    elif (self.flooritemstatus == 3) and (self.liftstatus =="arrived") and (self.liftdestinationfloor == 2):

                        self.status = "gofirstfloor"
                    
                    elif (self.flooritemstatus == 3) and (self.liftstatus =="up") and (self.liftdestinationfloor == 3):

                        self.status = "gothirdfloor"

                elif self.liftcurrentfloor == 3:

                    if (self.flooritemstatus == -1) and (self.liftdestinationfloor == -1):

                        self.status = "gofirstfloor"
                    
                    elif (self.flooritemstatus == 2) and (self.liftitemstatus == "arrived") and (self.liftdestinationfloor == 3):

                        self.status = "gofirstfloor"
                    
                    elif (self.flooritemstatus == 3) and (self.liftstatus =="arrived") and (self.liftdestinationfloor == 3):

                        self.status = "pushitem"
                    
                    elif (self.flooritemstatus == 2) and (self.liftstatus =="down") and (self.liftdestinationfloor == 3):

                        self.status = "gosecondfloor"

            else: # no data recieved

                self.status = "waitstatus"
                        
        elif self.emergency == True:

            if self.manual == True:

                self.status = "manual"
            
            elif self.manual == False:

                self.status = "emergency"


        # print(self.status)
        print("self.status :" + self.status)
        return self.status

class WstationStatusPublisher():
    def __init__(self):
        self.send_to_destination_pub = rospy.Publisher("wstation/status", String, queue_size=1)
        self.msg = String()

    def send_wstation_status(self,status):
        self.msg.data = status
        # print("self.status :" + status)
        print("msg.data    :" + self.msg.data)
        self.send_to_destination_pub.publish(self.msg)

####################################################################################################################
####################################################################################################################


def wstation_main():
    rospy.init_node('Wstation_sub',anonymous=False)
    rate = rospy.Rate(10)

    item_status_sub             = ItemStatusSubscriber()             # IrSensor list
    lift_current_floor_sub      = LiftCurrentFloorSubscriber()
    lift_status_sub             = LiftStatusSubscriber()
    lift_item_status_sub        = LiftItemStatusSubscriber()
    lift_item_size_sub          = LiftItemSizeSubscriber()
    lift_destination_floor_sub  = LifDestinationFloorSubscriber()
    emergency_sub               = EmergencySubscriber()
    manual_sub                  = ManualSubscriber()

    wstation_task               = WstationTask()
    wstation_status             = WstationStatusPublisher()

    print("Wstation start")
    while not rospy.is_shutdown():

        rate.sleep()

        # Subscribe
        TopicList.itemstatus        = item_status_sub.function()
        TopicList.liftcurrentfloor  = lift_current_floor_sub.function()            # 1, 2, 3
        TopicList.liftstatus        = lift_status_sub.function()                     # "wait", "move", "arrived"
        TopicList.liftitemstatus    = lift_item_status_sub.function()            # "none", "exist"
        TopicList.liftitemsize      = lift_item_size_sub.function()            # "good", "bad"

        TopicList.emergency         = emergency_sub.function()
        TopicList.manual            = manual_sub.function()

        defined_status = wstation_task.definestatus()

        wstation_status.send_wstation_status(defined_status)


if __name__ == '__main__':
    try:
        wstation_main()
    except rospy.ROSInterruptException:
        pass